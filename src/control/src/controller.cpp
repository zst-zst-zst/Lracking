#include "control/controller.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

namespace control {

namespace {
double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
}
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kMinScanDs = 1e-3;
constexpr double kDeadbandHystPx = 1.0;

std::string toLower(std::string v) {
    std::transform(v.begin(), v.end(), v.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return v;
}
}  // namespace

Controller::Controller(ControlConfig cfg) : cfg_(cfg) {}

void Controller::resetRuntimeState() {
    *this = Controller(cfg_);
}

// Core control logic for the vision pipeline: pixel error -> gimbal command.
common::GimbalCommand Controller::update(const common::TargetMeasurement& meas,
                                         const common::CameraModel& cam,
                                         const common::GimbalState& state) {
    common::GimbalCommand cmd;
    cmd.timestamp = meas.timestamp;
    cmd.pitch = state.pitch;
    cmd.yaw = state.yaw;
    cmd.pitch_rate = 0.0f;
    cmd.yaw_rate = 0.0f;
    cmd.pitch_acc = 0.0f;
    cmd.yaw_acc = 0.0f;
    cmd.mode = 1;  // 控制云台，不开火

    // Update period for rate limiting (steady-clock based to avoid frame timestamp jitter).
    const auto steady_now = std::chrono::steady_clock::now();
    double dt_s = cfg_.ctrl_dt_nominal_ms / 1000.0;
    if (has_last_update_steady_) {
        double dt_ms = std::chrono::duration<double, std::milli>(steady_now - last_update_tp_).count();
        dt_ms = clamp(dt_ms, cfg_.ctrl_dt_min_ms, cfg_.ctrl_dt_max_ms);
        dt_s = dt_ms / 1000.0;
    }
    last_update_tp_ = steady_now;
    has_last_update_steady_ = true;

    if (cfg_.startup_prep_ms > 0 && !startup_prep_done_) {
        if (!startup_prep_started_) {
            startup_prep_started_ = true;
            startup_prep_tp_ = steady_now;
            startup_state_frames_ = 0;
            last_state_ts_ = state.timestamp;
            startup_home_extend_ = false;
            std::cout << "DBG startup_prep begin total_ms=" << cfg_.startup_prep_ms
                      << " hold_ms=" << cfg_.startup_hold_ms
                      << " home_ms=" << cfg_.startup_home_ms
                      << " validate_ms=" << cfg_.startup_validate_ms << "\n";
        }
        if (state.timestamp != 0 && state.timestamp != last_state_ts_) {
            startup_state_frames_++;
            last_state_ts_ = state.timestamp;
        }
        const int prep_ms = std::max(0, cfg_.startup_prep_ms);
        int hold_ms = std::max(0, cfg_.startup_hold_ms);
        int home_ms = std::max(0, cfg_.startup_home_ms);
        int validate_ms = std::max(0, cfg_.startup_validate_ms);
        int sum_ms = hold_ms + home_ms + validate_ms;
        if (sum_ms <= 0) {
            hold_ms = 0;
            home_ms = prep_ms;
            validate_ms = 0;
        } else if (sum_ms < prep_ms) {
            validate_ms += (prep_ms - sum_ms);
        }
        const int64_t elapsed_ms =
            static_cast<int64_t>(std::chrono::duration<double, std::milli>(steady_now - startup_prep_tp_).count());
        if (elapsed_ms < prep_ms) {
            const int64_t hold_end = hold_ms;
            int64_t validate_end = hold_ms + validate_ms;
            int64_t home_end = hold_ms + validate_ms + home_ms;
            if (!cfg_.startup_validate_first) {
                validate_end = hold_ms + home_ms;
                home_end = hold_ms + home_ms + validate_ms;
            }
            double pitch_cmd = state.pitch;
            double yaw_cmd = state.yaw;
            bool exit_prep_now = false;
            if (elapsed_ms < hold_end) {
                if (elapsed_ms == 0 || (elapsed_ms / 200) != ((elapsed_ms - 1) / 200)) {
                    std::cout << "DBG startup_prep phase=hold elapsed_ms=" << elapsed_ms << "\n";
                }
            } else if (elapsed_ms < validate_end) {
                if ((elapsed_ms / 200) != ((elapsed_ms - 1) / 200)) {
                    std::cout << "DBG startup_prep phase=validate elapsed_ms=" << elapsed_ms << "\n";
                }
                if (cfg_.startup_allow_early_exit && meas.valid) {
                    std::cout << "DBG startup_prep early_exit target_acquired\n";
                    startup_prep_done_ = true;
                    exit_prep_now = true;
                }
            } else if (elapsed_ms < home_end) {
                if ((elapsed_ms / 200) != ((elapsed_ms - 1) / 200)) {
                    std::cout << "DBG startup_prep phase=home elapsed_ms=" << elapsed_ms << "\n";
                }
                double max_step = cfg_.max_angle_rate * dt_s;
                double target_pitch = cfg_.startup_home_pitch;
                double target_yaw = cfg_.startup_home_yaw;
                pitch_cmd = clamp(target_pitch, state.pitch - max_step, state.pitch + max_step);
                yaw_cmd = clamp(target_yaw, state.yaw - max_step, state.yaw + max_step);
            }
            if (!exit_prep_now) {
                cmd.pitch = static_cast<float>(pitch_cmd);
                cmd.yaw = static_cast<float>(yaw_cmd);
                last_pitch_ = pitch_cmd;
                last_yaw_ = yaw_cmd;
                has_last_ = true;
                last_update_ts_ = meas.timestamp;
                has_last_update_ = true;
                return cmd;
            }
        }
        if (cfg_.startup_require_home && !startup_prep_done_) {
            double dp = std::abs(state.pitch - cfg_.startup_home_pitch);
            double dy = std::abs(state.yaw - cfg_.startup_home_yaw);
            bool need_home = (dp > cfg_.startup_home_tol_deg) || (dy > cfg_.startup_home_tol_deg);
            if (need_home) {
                if (!startup_home_extend_) {
                    startup_home_extend_ = true;
                    startup_home_extend_tp_ = steady_now;
                    std::cout << "DBG startup_home_extend begin tol_deg=" << cfg_.startup_home_tol_deg
                              << " max_extra_ms=" << cfg_.startup_home_max_extra_ms << "\n";
                }
                const int extra_ms = std::max(0, cfg_.startup_home_max_extra_ms);
                int64_t extend_ms = static_cast<int64_t>(
                    std::chrono::duration<double, std::milli>(steady_now - startup_home_extend_tp_).count());
                if ((extend_ms / 200) != ((extend_ms - 1) / 200)) {
                    std::cout << "DBG startup_home_extend elapsed_ms=" << extend_ms
                              << " dp=" << dp << " dy=" << dy << "\n";
                }
                if (extend_ms <= extra_ms) {
                    double max_step = cfg_.max_angle_rate * dt_s;
                    double pitch_cmd = clamp(cfg_.startup_home_pitch,
                                             state.pitch - max_step, state.pitch + max_step);
                    double yaw_cmd = clamp(cfg_.startup_home_yaw,
                                           state.yaw - max_step, state.yaw + max_step);
                    cmd.pitch = static_cast<float>(pitch_cmd);
                    cmd.yaw = static_cast<float>(yaw_cmd);
                    last_pitch_ = pitch_cmd;
                    last_yaw_ = yaw_cmd;
                    has_last_ = true;
                    last_update_ts_ = meas.timestamp;
                    has_last_update_ = true;
                    return cmd;
                }
            }
            startup_home_extend_ = false;
        }
        startup_prep_done_ = true;
        std::cout << "DBG startup_prep done\n";
    } else if (startup_prep_done_ && startup_state_frames_ < cfg_.startup_min_state_frames) {
        startup_state_frames_ = cfg_.startup_min_state_frames;
    }

    // Startup: if no target within first N cycles, go home (0/0 by default).
    if (cfg_.startup_prep_ms <= 0 &&
        startup_frames_ < std::max(0, cfg_.startup_check_frames)) {
        startup_frames_++;
        if (meas.valid) {
            startup_has_meas_ = true;
        }
        if (!startup_has_meas_ && startup_frames_ >= cfg_.startup_check_frames) {
            double max_step = cfg_.max_angle_rate * dt_s;
            double pitch_cmd = clamp(cfg_.startup_home_pitch,
                                     state.pitch - max_step, state.pitch + max_step);
            double yaw_cmd = clamp(cfg_.startup_home_yaw,
                                   state.yaw - max_step, state.yaw + max_step);
            cmd.pitch = static_cast<float>(pitch_cmd);
            cmd.yaw = static_cast<float>(yaw_cmd);
            last_pitch_ = pitch_cmd;
            last_yaw_ = yaw_cmd;
            has_last_ = true;
            last_update_ts_ = meas.timestamp;
            has_last_update_ = true;
            return cmd;
        }
    }

    if (!meas.valid || cam.fx <= 0.0 || cam.fy <= 0.0) {
        in_deadband_ = false;
        if (!lost_active_) {
            lost_active_ = true;
            lost_start_ts_ = meas.timestamp;
            reacq_count_ = 0;
            scanning_ = false;
            scan_phase_ = 0.0;
        }
        double base_pitch = has_last_ ? last_pitch_ : state.pitch;
        double base_yaw = has_last_ ? last_yaw_ : state.yaw;
        double pitch_cmd = base_pitch;
        double yaw_cmd = base_yaw;

        // ── pitch 安全回归: 渐进回到目标常出现的 pitch (或 0) ──
        // 防止最后一次目标 pitch 是突变值导致极端抬头/低头
        double target_pitch = cfg_.lost_pitch_home;
        if (cfg_.lost_pitch_use_ema && has_freq_pitch_) {
            // 使用目标经常出现的 pitch 位置
            target_pitch = clamp(freq_pitch_, cfg_.lost_pitch_min, cfg_.lost_pitch_max);
        }
        {
            double pitch_step = cfg_.lost_pitch_return_rate * dt_s;
            if (pitch_cmd > target_pitch + pitch_step) {
                pitch_cmd -= pitch_step;
            } else if (pitch_cmd < target_pitch - pitch_step) {
                pitch_cmd += pitch_step;
            } else {
                pitch_cmd = target_pitch;
            }
        }
        // 绝对限幅: 不允许极端抬头/低头
        pitch_cmd = clamp(pitch_cmd, cfg_.lost_pitch_min, cfg_.lost_pitch_max);

        const double lost_ms = static_cast<double>(meas.timestamp - lost_start_ts_);
        const bool enter_search = cfg_.scan_enable &&
            (cfg_.scan_enter_delay_ms <= 0.0 || lost_ms >= cfg_.scan_enter_delay_ms);
        if (enter_search) {
            if (!scanning_) {
                scan_center_yaw_ = base_yaw;
                scan_phase_ = 0.0;
                scanning_ = true;
            }

            // 丢失时用更小幅度、更慢频率的 yaw 微扫
            const double search_amp_deg = std::clamp(cfg_.lost_yaw_scan_amp, 0.0, 1.0);
            const double search_rate_hz = std::max(1e-3, cfg_.lost_yaw_scan_rate);
            scan_phase_ += 2.0 * M_PI * search_rate_hz * dt_s;
            if (scan_phase_ > 2.0 * M_PI) {
                scan_phase_ = std::fmod(scan_phase_, 2.0 * M_PI);
            }

            // 只扫 yaw, pitch 已在上面做了安全回归
            yaw_cmd = scan_center_yaw_ + search_amp_deg * std::sin(scan_phase_);
        }

        double max_step = cfg_.max_angle_rate * dt_s;
        yaw_cmd = clamp(yaw_cmd, base_yaw - max_step, base_yaw + max_step);

        cmd.pitch = static_cast<float>(pitch_cmd);
        cmd.yaw = static_cast<float>(yaw_cmd);

        // 指令限幅: 防止绝对值突变
        if (has_prev_cmd_) {
            double dy = cmd.yaw - prev_cmd_yaw_;
            double dp = cmd.pitch - prev_cmd_pitch_;
            dy = clamp(dy, -cfg_.cmd_max_yaw_step_deg, cfg_.cmd_max_yaw_step_deg);
            dp = clamp(dp, -cfg_.cmd_max_pitch_step_deg, cfg_.cmd_max_pitch_step_deg);
            cmd.yaw = static_cast<float>(prev_cmd_yaw_ + dy);
            cmd.pitch = static_cast<float>(prev_cmd_pitch_ + dp);
        }
        prev_cmd_yaw_ = cmd.yaw;
        prev_cmd_pitch_ = cmd.pitch;
        has_prev_cmd_ = true;

        last_pitch_ = cmd.pitch;
        last_yaw_ = cmd.yaw;
        has_last_ = true;
        last_update_ts_ = meas.timestamp;
        has_last_update_ = true;
        return cmd;
    }

    if (lost_active_) {
        reacq_count_++;
        const int need = std::max(1, cfg_.scan_reacq_confirm_frames);
        if (reacq_count_ >= need) {
            scanning_ = false;
            lost_active_ = false;
            reacq_count_ = 0;
            has_last_meas_ = false;
            last_ff_yaw_rate_ = 0.0;
            last_ff_pitch_rate_ = 0.0;
            smooth_vx_ = 0.0;
            smooth_vy_ = 0.0;
        }
    }

    // ── 延迟补偿: 用跟踪器速度预测目标在 system_delay 后的位置 ──
    // 借鉴 sp_vision 的弹道飞行时间补偿思路,
    // 激光是光速不需要弹道补偿, 但系统延迟 (检测+通信+云台) ~30ms 需要补偿
    double compensated_u = meas.uv.x;
    double compensated_v = meas.uv.y;
    if (cfg_.delay_compensate && cfg_.system_delay_ms > 0.0) {
        const double delay_s = cfg_.system_delay_ms / 1000.0;
        // 优先使用跟踪器提供的速度估计 (Kalman 滤波后, 更稳定)
        double vx_px_s = meas.velocity.x;
        double vy_px_s = meas.velocity.y;
        // EMA 平滑防抖
        const double alpha = cfg_.ff_alpha;
        smooth_vx_ = alpha * vx_px_s + (1.0 - alpha) * smooth_vx_;
        smooth_vy_ = alpha * vy_px_s + (1.0 - alpha) * smooth_vy_;
        compensated_u += smooth_vx_ * delay_s;
        compensated_v += smooth_vy_ * delay_s;
    }

    // Pixel error relative to optical center.
    double du = compensated_u - cam.cx;
    double dv = compensated_v - cam.cy;
    const double abs_du = std::abs(du);
    const double abs_dv = std::abs(dv);
    const double deadband_enter = std::max(0.0, cfg_.deadband_px);
    const double deadband_exit = deadband_enter + kDeadbandHystPx;
    if (in_deadband_) {
        if (abs_du > deadband_exit || abs_dv > deadband_exit) {
            in_deadband_ = false;
        }
    } else {
        if (abs_du < deadband_enter && abs_dv < deadband_enter) {
            in_deadband_ = true;
        }
    }
    if (in_deadband_) {
        if (has_last_) {
            cmd.pitch = static_cast<float>(last_pitch_);
            cmd.yaw = static_cast<float>(last_yaw_);
        }
        last_uv_ = meas.uv;
        last_meas_ts_ = meas.timestamp;
        has_last_meas_ = true;
        last_update_ts_ = meas.timestamp;
        has_last_update_ = true;
        return cmd;
    }

    // Map pixel error to angular error (deg), with configurable sign.
    double dyaw_rad = cfg_.yaw_sign * std::atan(du / cam.fx);
    double dpitch_rad = cfg_.pitch_sign * std::atan(dv / cam.fy);
    double dyaw = dyaw_rad * kRadToDeg;
    double dpitch = dpitch_rad * kRadToDeg;

    // ── 自适应增益: 远距离(bbox小) kp 更大, 近距离(bbox大) kp 更小 ──
    double kp = cfg_.kp;
    if (cfg_.adaptive_kp && meas.bbox_area > 0.0f) {
        double area = static_cast<double>(meas.bbox_area);
        if (area >= cfg_.kp_near_area) {
            kp = cfg_.kp_near;
        } else if (area <= cfg_.kp_far_area) {
            kp = cfg_.kp_far;
        } else {
            // 线性插值
            double t = (area - cfg_.kp_far_area) / (cfg_.kp_near_area - cfg_.kp_far_area);
            kp = cfg_.kp_far + t * (cfg_.kp_near - cfg_.kp_far);
        }
    }

    // ── 激光同轴补偿 (标定模型: 角度偏差 + 物理视差) ──
    // 标定模型: 激光落点角度 = boresight_deg (常数) + atan(offset/Z) (距离相关)
    // 控制器需从目标角度中减去激光偏差，才能让激光对准目标
    double bs_yaw_deg = cfg_.boresight_yaw_deg;
    double bs_pitch_deg = cfg_.boresight_pitch_deg;
    // 物理偏移视差 (距离相关): 通过 bbox 面积估算距离
    if ((cfg_.laser_offset_x != 0.0 || cfg_.laser_offset_y != 0.0) &&
        meas.bbox_area > 0.0f && cfg_.target_height_m > 0.01) {
        double bbox_h = std::sqrt(static_cast<double>(meas.bbox_area));
        double z_est = cam.fy * cfg_.target_height_m / bbox_h;
        if (z_est > 0.5 && z_est < 200.0) {
            bs_yaw_deg += std::atan(cfg_.laser_offset_x / z_est) * kRadToDeg;
            bs_pitch_deg += std::atan(cfg_.laser_offset_y / z_est) * kRadToDeg;
        }
    }

    // Position control (absolute angle setpoint).
    // dyaw/dpitch = 目标相对相机光轴的角度误差
    // bs_*_deg = 激光相对相机光轴的角度偏差
    // 需要补偿: cmd = state + kp * (目标角度 - 激光角度)
    double yaw_cmd = state.yaw + kp * (dyaw - cfg_.yaw_sign * bs_yaw_deg);
    double pitch_cmd = state.pitch + kp * (dpitch - cfg_.pitch_sign * bs_pitch_deg);

    if (cfg_.use_damping && cfg_.damping_kd > 0.0) {
        double yaw_rate_fb = 0.0;
        double pitch_rate_fb = 0.0;
        bool has_rate = false;
        const std::string src = toLower(cfg_.damping_source);
        if (src == "gimbal") {
            yaw_rate_fb = state.yaw_rate;
            pitch_rate_fb = state.pitch_rate;
            has_rate = true;
        } else {
            if (has_last_meas_) {
                double dt_ms = static_cast<double>(meas.timestamp - last_meas_ts_);
                if (dt_ms > 0.0 && dt_ms <= cfg_.damping_dt_max_ms) {
                    double dt_s = dt_ms / 1000.0;
                    double u_dot = (meas.uv.x - last_uv_.x) / dt_s;
                    double v_dot = (meas.uv.y - last_uv_.y) / dt_s;
                    yaw_rate_fb = cfg_.yaw_sign * (u_dot / cam.fx) * kRadToDeg;
                    pitch_rate_fb = cfg_.pitch_sign * (v_dot / cam.fy) * kRadToDeg;
                    has_rate = true;
                }
            }
        }
        if (has_rate) {
            yaw_cmd -= cfg_.damping_kd * yaw_rate_fb;
            pitch_cmd -= cfg_.damping_kd * pitch_rate_fb;
        }
    }

    // ── 速度前馈: 用 Kalman 跟踪器的速度估计 ──
    // 电控速度环 PID 的 ref = 角度环输出 + 前馈速度(deg/s)
    // 前馈让电控"提前知道"目标在动, 减少追踪滞后
    // 注意: 直接用 meas.velocity (Kalman已平滑), 不再经过延迟补偿的 EMA
    if (cfg_.use_velocity_ff) {
        double vx_px_s = meas.velocity.x;
        double vy_px_s = meas.velocity.y;
        // 像素速度 → 角速度 (deg/s)
        double yaw_rate_raw = cfg_.yaw_sign * (vx_px_s / cam.fx) * kRadToDeg;
        double pitch_rate_raw = cfg_.pitch_sign * (vy_px_s / cam.fy) * kRadToDeg;
        // EMA 平滑前馈, 防止突变
        double yaw_rate_ff =
            cfg_.ff_alpha * yaw_rate_raw + (1.0 - cfg_.ff_alpha) * last_ff_yaw_rate_;
        double pitch_rate_ff =
            cfg_.ff_alpha * pitch_rate_raw + (1.0 - cfg_.ff_alpha) * last_ff_pitch_rate_;
        yaw_rate_ff = clamp(yaw_rate_ff, -cfg_.ff_rate_max, cfg_.ff_rate_max);
        pitch_rate_ff = clamp(pitch_rate_ff, -cfg_.ff_rate_max, cfg_.ff_rate_max);

        last_ff_yaw_rate_ = yaw_rate_ff;
        last_ff_pitch_rate_ = pitch_rate_ff;

        cmd.pitch_rate = static_cast<float>(pitch_rate_ff);  // deg/s → 电控速度环叠加
        cmd.yaw_rate = static_cast<float>(yaw_rate_ff);      // deg/s → 电控速度环叠加
    } else {
        last_ff_yaw_rate_ = 0.0;
        last_ff_pitch_rate_ = 0.0;
    }

    // Output smoothing.
    if (has_last_) {
        yaw_cmd = cfg_.lowpass_alpha * yaw_cmd + (1.0 - cfg_.lowpass_alpha) * last_yaw_;
        pitch_cmd = cfg_.lowpass_alpha * pitch_cmd + (1.0 - cfg_.lowpass_alpha) * last_pitch_;
    }

    // Rate limiting in deg/s.
    double max_step = cfg_.max_angle_rate * dt_s;
    yaw_cmd = clamp(yaw_cmd, state.yaw - max_step, state.yaw + max_step);
    pitch_cmd = clamp(pitch_cmd, state.pitch - max_step, state.pitch + max_step);

    cmd.pitch = static_cast<float>(pitch_cmd);
    cmd.yaw = static_cast<float>(yaw_cmd);

    // ── EMA 记录目标常出现的 pitch 位置 ──
    if (cfg_.lost_pitch_use_ema) {
        if (!has_freq_pitch_) {
            freq_pitch_ = state.pitch;
            has_freq_pitch_ = true;
        } else {
            freq_pitch_ = cfg_.lost_pitch_ema_alpha * state.pitch +
                          (1.0 - cfg_.lost_pitch_ema_alpha) * freq_pitch_;
        }
    }

    // ── 指令限幅: 防止绝对值跳变导致电控突变 ──
    if (has_prev_cmd_) {
        double dy = cmd.yaw - prev_cmd_yaw_;
        double dp = cmd.pitch - prev_cmd_pitch_;
        dy = clamp(dy, -cfg_.cmd_max_yaw_step_deg, cfg_.cmd_max_yaw_step_deg);
        dp = clamp(dp, -cfg_.cmd_max_pitch_step_deg, cfg_.cmd_max_pitch_step_deg);
        cmd.yaw = static_cast<float>(prev_cmd_yaw_ + dy);
        cmd.pitch = static_cast<float>(prev_cmd_pitch_ + dp);
    }
    prev_cmd_yaw_ = cmd.yaw;
    prev_cmd_pitch_ = cmd.pitch;
    has_prev_cmd_ = true;

    last_pitch_ = cmd.pitch;
    last_yaw_ = cmd.yaw;
    has_last_ = true;
    last_uv_ = meas.uv;
    last_meas_ts_ = meas.timestamp;
    has_last_meas_ = true;
    last_update_ts_ = meas.timestamp;
    has_last_update_ = true;
    return cmd;
}

}  // namespace control
