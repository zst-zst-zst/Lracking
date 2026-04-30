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
                                         const common::Boresight& bs,
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
        }
        double base_pitch = has_last_ ? last_pitch_ : state.pitch;
        double base_yaw = has_last_ ? last_yaw_ : state.yaw;
        double pitch_cmd = base_pitch;
        double yaw_cmd = base_yaw;

        const double lost_ms = static_cast<double>(meas.timestamp - lost_start_ts_);
        const bool enter_scan = !cfg_.scan_enable ? false
            : (cfg_.scan_enter_delay_ms <= 0.0 || lost_ms >= cfg_.scan_enter_delay_ms);
            if (enter_scan) {
            if (!scanning_) {
                double offset = 0.0;
                if (has_last_meas_) {
                    double du = last_uv_.x - bs.u_L;
                    double dv = last_uv_.y - bs.v_L;
                    double u = cfg_.yaw_sign * du;
                    double v = cfg_.pitch_sign * dv;
                    if (std::abs(u) > 1e-6 || std::abs(v) > 1e-6) {
                        const std::string pattern = toLower(cfg_.scan_pattern);
                        if (pattern == "spiral") {
                            double kx = std::max(1e-3, cfg_.scan_k_yaw);
                            double ky = std::max(1e-3, cfg_.scan_k_pitch);
                            u /= kx;
                            v /= ky;
                        }
                        offset = std::atan2(v, u);
                    }
                }
                scan_center_pitch_ = base_pitch;
                scan_center_yaw_ = base_yaw;
                scan_phase_ = 0.0;
                scan_phase_offset_ = offset;
                scan_dir_ = 1;
                scanning_ = true;
            }
            const std::string pattern = toLower(cfg_.scan_pattern);
            if (pattern == "spiral") {
                double theta = scan_phase_;
                const double spacing = std::max(1e-3, cfg_.scan_spacing_deg);
                const double a = spacing / (2.0 * M_PI);
                double r = a * theta;
                const double r_max = std::max(spacing, cfg_.scan_r_max_deg);
                const double kx = std::max(1e-3, cfg_.scan_k_yaw);
                const double ky = std::max(1e-3, cfg_.scan_k_pitch);
                const double psi = theta + scan_phase_offset_;

                if (cfg_.scan_spiral_return) {
                    if (scan_dir_ > 0 && r >= r_max) {
                        scan_dir_ = -1;
                    } else if (scan_dir_ < 0 && theta <= 0.0) {
                        scan_dir_ = 1;
                        theta = 0.0;
                        r = 0.0;
                    }
                }

                double dx_dtheta = 0.0;
                double dy_dtheta = 0.0;
                if (r < r_max) {
                    dx_dtheta = kx * (a * std::cos(psi) - r * std::sin(psi));
                    dy_dtheta = ky * (a * std::sin(psi) + r * std::cos(psi));
                } else {
                    r = r_max;
                    dx_dtheta = -kx * r * std::sin(psi);
                    dy_dtheta = ky * r * std::cos(psi);
                }
                double ds_dtheta = std::sqrt(dx_dtheta * dx_dtheta + dy_dtheta * dy_dtheta);
                ds_dtheta = std::max(ds_dtheta, kMinScanDs);
                const double v = std::max(1e-3, cfg_.scan_speed_deg_s);
                const double step = (v / ds_dtheta) * dt_s;
                if (cfg_.scan_spiral_return) {
                    theta += static_cast<double>(scan_dir_) * step;
                    if (theta < 0.0) {
                        theta = 0.0;
                    }
                } else {
                    theta += step;
                }

                if (theta > 1000.0 * M_PI) {
                    theta = std::fmod(theta, 2.0 * M_PI);
                }
                scan_phase_ = theta;
                r = a * theta;
                const double r_eval = std::min(r, r_max);
                pitch_cmd = scan_center_pitch_ + ky * r_eval * std::sin(theta + scan_phase_offset_);
                yaw_cmd = scan_center_yaw_ + kx * r_eval * std::cos(theta + scan_phase_offset_);
            } else {
                scan_phase_ += 2.0 * M_PI * cfg_.scan_rate_hz * dt_s;
                if (scan_phase_ > 2.0 * M_PI) {
                    scan_phase_ = std::fmod(scan_phase_, 2.0 * M_PI);
                }
                pitch_cmd = scan_center_pitch_ +
                    cfg_.scan_radius_deg * std::sin(scan_phase_ + scan_phase_offset_);
                yaw_cmd = scan_center_yaw_ +
                    cfg_.scan_radius_deg * std::cos(scan_phase_ + scan_phase_offset_);
            }
        }

        double max_step = cfg_.max_angle_rate * dt_s;
        pitch_cmd = clamp(pitch_cmd, base_pitch - max_step, base_pitch + max_step);
        yaw_cmd = clamp(yaw_cmd, base_yaw - max_step, base_yaw + max_step);

        cmd.pitch = static_cast<float>(pitch_cmd);
        cmd.yaw = static_cast<float>(yaw_cmd);

        last_pitch_ = pitch_cmd;
        last_yaw_ = yaw_cmd;
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

    // Pixel error relative to boresight.
    double du = compensated_u - bs.u_L;
    double dv = compensated_v - bs.v_L;
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

    // Position control (absolute angle setpoint).
    double yaw_cmd = state.yaw + kp * dyaw;
    double pitch_cmd = state.pitch + kp * dpitch;

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

    // Optional feedforward estimate (sent only).
    if (cfg_.use_velocity_ff && has_last_meas_) {
        double dt_ms = static_cast<double>(meas.timestamp - last_meas_ts_);
        if (dt_ms > 0.0 && dt_ms <= cfg_.ff_dt_max_ms) {
            double dt_s = dt_ms / 1000.0;
            double u_dot = (meas.uv.x - last_uv_.x) / dt_s;
            double v_dot = (meas.uv.y - last_uv_.y) / dt_s;
            double yaw_rate_raw = cfg_.yaw_sign * (u_dot / cam.fx) * kRadToDeg;
            double pitch_rate_raw = cfg_.pitch_sign * (v_dot / cam.fy) * kRadToDeg;
            double yaw_rate_ff =
                cfg_.ff_alpha * yaw_rate_raw + (1.0 - cfg_.ff_alpha) * last_ff_yaw_rate_;
            double pitch_rate_ff =
                cfg_.ff_alpha * pitch_rate_raw + (1.0 - cfg_.ff_alpha) * last_ff_pitch_rate_;
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

    last_pitch_ = pitch_cmd;
    last_yaw_ = yaw_cmd;
    has_last_ = true;
    last_uv_ = meas.uv;
    last_meas_ts_ = meas.timestamp;
    has_last_meas_ = true;
    last_update_ts_ = meas.timestamp;
    has_last_update_ = true;
    return cmd;
}

}  // namespace control
