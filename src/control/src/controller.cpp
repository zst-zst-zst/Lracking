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

        const double lost_ms = static_cast<double>(meas.timestamp - lost_start_ts_);
        const bool enter_search = cfg_.scan_enable &&
            (cfg_.scan_enter_delay_ms <= 0.0 || lost_ms >= cfg_.scan_enter_delay_ms);

        // ── pitch 安全回归: 仅在确认丢失后 (超过 scan_enter_delay) 才开始回归 ──
        // 在延迟窗口内保持最后跟踪 pitch, 防止间歇检测导致 pitch 来回拉扯
        if (enter_search) {
            double target_pitch = cfg_.lost_pitch_home;
            if (cfg_.lost_pitch_use_ema && has_freq_pitch_) {
                target_pitch = clamp(freq_pitch_, cfg_.lost_pitch_min, cfg_.lost_pitch_max);
            }
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
            integral_yaw_ = 0.0;
            integral_pitch_ = 0.0;
        }
    }

    // ── 延迟补偿: 用跟踪器速度预测目标在 system_delay 后的位置 ──
    // 激光是光速不需要弹道补偿, 但系统延迟 (检测+通信+云台) ~10ms 需要补偿
    // 直接用 Kalman 滤波后的速度, 不再额外 EMA (避免双重平滑延迟)
    double compensated_u = meas.uv.x;
    double compensated_v = meas.uv.y;
    if (cfg_.delay_compensate && cfg_.system_delay_ms > 0.0) {
        const double delay_s = cfg_.system_delay_ms / 1000.0;
        compensated_u += static_cast<double>(meas.velocity.x) * delay_s;
        compensated_v += static_cast<double>(meas.velocity.y) * delay_s;
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
        // 积分项衰减: 在死区内不继续累积, 防止 windup
        integral_yaw_ *= 0.95;
        integral_pitch_ *= 0.95;
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
    // 物理偏移视差 (距离相关): 通过 bbox 高度估算距离 Z = fy * H / h_px
    if ((cfg_.laser_offset_x != 0.0 || cfg_.laser_offset_y != 0.0) &&
        cfg_.target_height_m > 0.01) {
        double bbox_h = (meas.bbox_height > 1.0f)
                            ? static_cast<double>(meas.bbox_height)
                            : std::sqrt(static_cast<double>(std::max(meas.bbox_area, 1.0f)));
        double z_raw = cam.fy * cfg_.target_height_m / bbox_h;
        // 自适应 EMA 平滑距离估计:
        //   bbox 噪声 (小变化) → 低 alpha 平滑
        //   距离真的变了 (pitch 大动作) → 高 alpha 快速跟上
        if (!has_z_est_) {
            z_est_smooth_ = z_raw;
            has_z_est_ = true;
        } else {
            double diff_ratio = std::abs(z_raw - z_est_smooth_) /
                                std::max(z_est_smooth_, 0.1);
            double alpha = (diff_ratio > 0.15) ? 0.6 : 0.25;
            z_est_smooth_ = alpha * z_raw + (1.0 - alpha) * z_est_smooth_;
        }
        double z_est = z_est_smooth_;
        if (z_est > 0.5 && z_est < 200.0) {
            bs_yaw_deg += std::atan(cfg_.laser_offset_x / z_est) * kRadToDeg;
            bs_pitch_deg += std::atan(cfg_.laser_offset_y / z_est) * kRadToDeg;
        }
    }

    // ── 轴独立 kp: yaw 可以用更高的增益 (横向运动更快) ──
    double kp_y = (cfg_.kp_yaw > 0.0) ? cfg_.kp_yaw : kp;
    double kp_p = (cfg_.kp_pitch > 0.0) ? cfg_.kp_pitch : kp;

    // Position control (absolute angle setpoint).
    // 跟踪误差和视差补偿分离: kp 只放大纯跟踪误差, 视差作为常数偏移
    // 这样近距离大视差 (5-7°) 不会被 kp 放大成 17°, 防止"永远向上推"
    double bs_yaw_offset = cfg_.yaw_sign * bs_yaw_deg;
    double bs_pitch_offset = cfg_.pitch_sign * bs_pitch_deg;
    double err_yaw = dyaw;     // 纯跟踪误差 (目标离画面中心多远)
    double err_pitch = dpitch; // 纯跟踪误差
    double yaw_cmd = state.yaw + kp_y * err_yaw - bs_yaw_offset;
    double pitch_cmd = state.pitch + kp_p * err_pitch - bs_pitch_offset;

    // ── 积分项: 消除稳态误差 (含视差补偿残差) ──
    // 死区以上就积累, 无上界限制 (近距离视差补偿可达 14°+)
    // ki_max_deg 限幅防 windup; 过零衰减防超调
    if (cfg_.ki_yaw > 0.0 || cfg_.ki_pitch > 0.0) {
        if (std::abs(err_yaw) > 0.02) {
            integral_yaw_ += cfg_.ki_yaw * err_yaw * dt_s;
        }
        if (std::abs(err_pitch) > 0.02) {
            integral_pitch_ += cfg_.ki_pitch * err_pitch * dt_s;
        }
        // 过零衰减: 误差符号翻转 = 超调, 积分应快速泄放
        if (integral_yaw_ * err_yaw < 0) integral_yaw_ *= 0.5;
        if (integral_pitch_ * err_pitch < 0) integral_pitch_ *= 0.5;
        integral_yaw_ = clamp(integral_yaw_, -cfg_.ki_max_deg, cfg_.ki_max_deg);
        integral_pitch_ = clamp(integral_pitch_, -cfg_.ki_max_deg, cfg_.ki_max_deg);
        yaw_cmd += integral_yaw_;
        pitch_cmd += integral_pitch_;
    }

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
    // 前馈让电控“提前知道”目标在动, 减少追踪滞后
    if (cfg_.use_velocity_ff) {
        double vx_px_s = meas.velocity.x;
        double vy_px_s = meas.velocity.y;
        // 像素速度 → 角速度 (deg/s)
        double yaw_rate_raw = cfg_.yaw_sign * (vx_px_s / cam.fx) * kRadToDeg;
        double pitch_rate_raw = cfg_.pitch_sign * (vy_px_s / cam.fy) * kRadToDeg;
        // 轴独立增益倍率 (先乘 gain, 再 EMA)
        // 重要: gain 必须在 EMA 之前, 否则 last_ff 存的是 gain 放大后的值
        // 下一帧 EMA 会再乘 gain → 实际增益 = gain*α/(1-gain*(1-α)) > gain
        double yaw_rate_scaled = yaw_rate_raw * cfg_.ff_gain_yaw;
        double pitch_rate_scaled = pitch_rate_raw * cfg_.ff_gain_pitch;
        // 轴独立 EMA 平滑前馈
        double alpha_ff_y = (cfg_.ff_alpha_yaw > 0.0) ? cfg_.ff_alpha_yaw : cfg_.ff_alpha;
        double alpha_ff_p = (cfg_.ff_alpha_pitch > 0.0) ? cfg_.ff_alpha_pitch : cfg_.ff_alpha;
        double yaw_rate_ff =
            alpha_ff_y * yaw_rate_scaled + (1.0 - alpha_ff_y) * last_ff_yaw_rate_;
        double pitch_rate_ff =
            alpha_ff_p * pitch_rate_scaled + (1.0 - alpha_ff_p) * last_ff_pitch_rate_;
        yaw_rate_ff = clamp(yaw_rate_ff, -cfg_.ff_rate_max, cfg_.ff_rate_max);
        pitch_rate_ff = clamp(pitch_rate_ff, -cfg_.ff_rate_max, cfg_.ff_rate_max);

        last_ff_yaw_rate_ = yaw_rate_ff;
        last_ff_pitch_rate_ = pitch_rate_ff;

        cmd.pitch_rate = static_cast<float>(pitch_rate_ff);
        cmd.yaw_rate = static_cast<float>(yaw_rate_ff);
    } else {
        last_ff_yaw_rate_ = 0.0;
        last_ff_pitch_rate_ = 0.0;
    }

    // Output smoothing (轴独立低通系数: yaw 更激进 = 更响应横向运动).
    if (has_last_) {
        double alpha_y = (cfg_.lowpass_alpha_yaw > 0.0) ? cfg_.lowpass_alpha_yaw : cfg_.lowpass_alpha;
        double alpha_p = (cfg_.lowpass_alpha_pitch > 0.0) ? cfg_.lowpass_alpha_pitch : cfg_.lowpass_alpha;
        yaw_cmd = alpha_y * yaw_cmd + (1.0 - alpha_y) * last_yaw_;
        pitch_cmd = alpha_p * pitch_cmd + (1.0 - alpha_p) * last_pitch_;
    }

    // 指令超前限幅: cmd 最多超前 state 固定度数 (不依赖 dt)
    // 旧版 rate*dt 在 dt 抖动时导致 yaw 不跟随; pitch 需要更大空间做视差补偿
    {
        constexpr double kMaxLeadYaw = 15.0;
        constexpr double kMaxLeadPitch = 10.0;
        yaw_cmd = clamp(yaw_cmd, state.yaw - kMaxLeadYaw, state.yaw + kMaxLeadYaw);
        pitch_cmd = clamp(pitch_cmd, state.pitch - kMaxLeadPitch, state.pitch + kMaxLeadPitch);
    }
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
    // ── pitch 速度通道辅助: 云台 pitch 位置环弱, 用速度通道推动 ──
    // 加入角速度阻尼防止振荡: 云台已在动时减小推力
    if (cfg_.pitch_rate_assist_kp > 0.0) {
        double pitch_pos_err = static_cast<double>(cmd.pitch) - state.pitch;
        double assist_rate = cfg_.pitch_rate_assist_kp * pitch_pos_err;
        // 阻尼: 云台已在往目标方向动时减小推力, 防止超调振荡
        double pitch_rate_fb = static_cast<double>(state.pitch_rate);
        assist_rate -= 0.3 * pitch_rate_fb;
        cmd.pitch_rate += static_cast<float>(assist_rate);
    }
    // pitch 速度限幅: 云台 pitch 伺服通常比 yaw 慢, 用独立限幅防止超调
    {
        double pitch_rate_limit = (cfg_.max_angle_rate_pitch > 0.0)
            ? cfg_.max_angle_rate_pitch * 0.15  // pitch rate 上限 = rate_limit 的 15%
            : cfg_.ff_rate_max;
        // 下限 20, 上限 ff_rate_max
        pitch_rate_limit = clamp(pitch_rate_limit, 20.0, cfg_.ff_rate_max);
        cmd.pitch_rate = static_cast<float>(
            clamp(static_cast<double>(cmd.pitch_rate), -pitch_rate_limit, pitch_rate_limit));
    }
    // yaw 速度限幅保持宽松
    cmd.yaw_rate = static_cast<float>(
        clamp(static_cast<double>(cmd.yaw_rate), -cfg_.ff_rate_max, cfg_.ff_rate_max));

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
