#include "control/config.h"

#include <iostream>

#include <opencv2/core.hpp>

namespace control {

bool loadControlConfig(const std::string& path, ControlConfig* cfg,
                       common::CameraModel* cam,
                       const std::string& camera_config_path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open control config: " << path << "\n";
        return false;
    }

    // Load camera intrinsics from camera.yaml if path provided
    if (cam && !camera_config_path.empty()) {
        cv::FileStorage cam_fs(camera_config_path, cv::FileStorage::READ);
        if (cam_fs.isOpened()) {
            cv::FileNode camera_matrix = cam_fs["camera_matrix"];
            if (!camera_matrix.empty() && camera_matrix.isMap()) {
                cv::FileNode data_node = camera_matrix["data"];
                if (!data_node.empty()) {
                    std::vector<double> data_vec;
                    data_node >> data_vec;
                    if (data_vec.size() == 9) {
                        cam->fx = data_vec[0];
                        cam->fy = data_vec[4];
                        cam->cx = data_vec[2];
                        cam->cy = data_vec[5];
                    }
                }
            }
            cam_fs.release();
        } else {
            std::cerr << "Warning: Failed to open camera config: " << camera_config_path
                      << ", using fallback values from control.yaml\n";
        }
    }
    if (cfg) {
        fs["kp"] >> cfg->kp;
        fs["deadband_px"] >> cfg->deadband_px;
        fs["max_angle_rate"] >> cfg->max_angle_rate;
        if (!fs["max_angle_rate_pitch"].empty()) {
            fs["max_angle_rate_pitch"] >> cfg->max_angle_rate_pitch;
        }
        fs["lowpass_alpha"] >> cfg->lowpass_alpha;
        if (!fs["yaw_sign"].empty()) {
            fs["yaw_sign"] >> cfg->yaw_sign;
        }
        if (!fs["pitch_sign"].empty()) {
            fs["pitch_sign"] >> cfg->pitch_sign;
        }
        if (!fs["ctrl_dt_nominal_ms"].empty()) {
            fs["ctrl_dt_nominal_ms"] >> cfg->ctrl_dt_nominal_ms;
        }
        if (!fs["ctrl_dt_min_ms"].empty()) {
            fs["ctrl_dt_min_ms"] >> cfg->ctrl_dt_min_ms;
        }
        if (!fs["ctrl_dt_max_ms"].empty()) {
            fs["ctrl_dt_max_ms"] >> cfg->ctrl_dt_max_ms;
        }
        if (!fs["use_velocity_ff"].empty()) {
            fs["use_velocity_ff"] >> cfg->use_velocity_ff;
        }
        if (!fs["ff_alpha"].empty()) {
            fs["ff_alpha"] >> cfg->ff_alpha;
        }
        if (!fs["ff_rate_max"].empty()) {
            fs["ff_rate_max"] >> cfg->ff_rate_max;
        }
        if (!fs["ff_dt_max_ms"].empty()) {
            fs["ff_dt_max_ms"] >> cfg->ff_dt_max_ms;
        }
        if (!fs["use_damping"].empty()) {
            fs["use_damping"] >> cfg->use_damping;
        }
        if (!fs["damping_source"].empty()) {
            fs["damping_source"] >> cfg->damping_source;
        }
        if (!fs["damping_kd"].empty()) {
            fs["damping_kd"] >> cfg->damping_kd;
        }
        if (!fs["damping_dt_max_ms"].empty()) {
            fs["damping_dt_max_ms"] >> cfg->damping_dt_max_ms;
        }
        if (!fs["scan_enable"].empty()) {
            fs["scan_enable"] >> cfg->scan_enable;
        }
        if (!fs["scan_radius_deg"].empty()) {
            fs["scan_radius_deg"] >> cfg->scan_radius_deg;
        }
        if (!fs["scan_rate_hz"].empty()) {
            fs["scan_rate_hz"] >> cfg->scan_rate_hz;
        }
        if (!fs["scan_pattern"].empty()) {
            fs["scan_pattern"] >> cfg->scan_pattern;
        }
        if (!fs["scan_spacing_deg"].empty()) {
            fs["scan_spacing_deg"] >> cfg->scan_spacing_deg;
        }
        if (!fs["scan_speed_deg_s"].empty()) {
            fs["scan_speed_deg_s"] >> cfg->scan_speed_deg_s;
        }
        if (!fs["scan_r_max_deg"].empty()) {
            fs["scan_r_max_deg"] >> cfg->scan_r_max_deg;
        }
        if (!fs["scan_spiral_return"].empty()) {
            fs["scan_spiral_return"] >> cfg->scan_spiral_return;
        }
        if (!fs["scan_k_yaw"].empty()) {
            fs["scan_k_yaw"] >> cfg->scan_k_yaw;
        }
        if (!fs["scan_k_pitch"].empty()) {
            fs["scan_k_pitch"] >> cfg->scan_k_pitch;
        }
        if (!fs["scan_enter_delay_ms"].empty()) {
            fs["scan_enter_delay_ms"] >> cfg->scan_enter_delay_ms;
        }
        if (!fs["scan_reacq_confirm_frames"].empty()) {
            fs["scan_reacq_confirm_frames"] >> cfg->scan_reacq_confirm_frames;
        }
        if (!fs["startup_check_frames"].empty()) {
            fs["startup_check_frames"] >> cfg->startup_check_frames;
        }
        if (!fs["startup_home_pitch"].empty()) {
            fs["startup_home_pitch"] >> cfg->startup_home_pitch;
        }
        if (!fs["startup_home_yaw"].empty()) {
            fs["startup_home_yaw"] >> cfg->startup_home_yaw;
        }
        if (!fs["startup_prep_ms"].empty()) {
            fs["startup_prep_ms"] >> cfg->startup_prep_ms;
        }
        if (!fs["startup_hold_ms"].empty()) {
            fs["startup_hold_ms"] >> cfg->startup_hold_ms;
        }
        if (!fs["startup_home_ms"].empty()) {
            fs["startup_home_ms"] >> cfg->startup_home_ms;
        }
        if (!fs["startup_validate_ms"].empty()) {
            fs["startup_validate_ms"] >> cfg->startup_validate_ms;
        }
        if (!fs["startup_min_state_frames"].empty()) {
            fs["startup_min_state_frames"] >> cfg->startup_min_state_frames;
        }
        if (!fs["startup_require_home"].empty()) {
            fs["startup_require_home"] >> cfg->startup_require_home;
        }
        if (!fs["startup_home_tol_deg"].empty()) {
            fs["startup_home_tol_deg"] >> cfg->startup_home_tol_deg;
        }
        if (!fs["startup_home_max_extra_ms"].empty()) {
            fs["startup_home_max_extra_ms"] >> cfg->startup_home_max_extra_ms;
        }
        if (!fs["startup_validate_first"].empty()) {
            fs["startup_validate_first"] >> cfg->startup_validate_first;
        }
        if (!fs["startup_allow_early_exit"].empty()) {
            fs["startup_allow_early_exit"] >> cfg->startup_allow_early_exit;
        }
        // ── 丢失目标安全策略 ──
        if (!fs["lost_pitch_home"].empty()) {
            fs["lost_pitch_home"] >> cfg->lost_pitch_home;
        }
        if (!fs["lost_pitch_return_rate"].empty()) {
            fs["lost_pitch_return_rate"] >> cfg->lost_pitch_return_rate;
        }
        if (!fs["lost_pitch_max"].empty()) {
            fs["lost_pitch_max"] >> cfg->lost_pitch_max;
        }
        if (!fs["lost_pitch_min"].empty()) {
            fs["lost_pitch_min"] >> cfg->lost_pitch_min;
        }
        if (!fs["lost_yaw_scan_rate"].empty()) {
            fs["lost_yaw_scan_rate"] >> cfg->lost_yaw_scan_rate;
        }
        if (!fs["lost_yaw_scan_amp"].empty()) {
            fs["lost_yaw_scan_amp"] >> cfg->lost_yaw_scan_amp;
        }
        if (!fs["lost_pitch_use_ema"].empty()) {
            fs["lost_pitch_use_ema"] >> cfg->lost_pitch_use_ema;
        }
        if (!fs["lost_pitch_ema_alpha"].empty()) {
            fs["lost_pitch_ema_alpha"] >> cfg->lost_pitch_ema_alpha;
        }
        // ── 指令限幅 ──
        if (!fs["cmd_max_yaw_step_deg"].empty()) {
            fs["cmd_max_yaw_step_deg"] >> cfg->cmd_max_yaw_step_deg;
        }
        if (!fs["cmd_max_pitch_step_deg"].empty()) {
            fs["cmd_max_pitch_step_deg"] >> cfg->cmd_max_pitch_step_deg;
        }
        // ── 延迟补偿 ──
        if (!fs["delay_compensate"].empty()) {
            fs["delay_compensate"] >> cfg->delay_compensate;
        }
        if (!fs["system_delay_ms"].empty()) {
            fs["system_delay_ms"] >> cfg->system_delay_ms;
        }
        // ── 激光同轴补偿 ──
        if (!fs["laser_offset_x"].empty()) {
            fs["laser_offset_x"] >> cfg->laser_offset_x;
        }
        if (!fs["laser_offset_y"].empty()) {
            fs["laser_offset_y"] >> cfg->laser_offset_y;
        }
        if (!fs["target_height_m"].empty()) {
            fs["target_height_m"] >> cfg->target_height_m;
        }
        if (!fs["boresight_yaw_deg"].empty()) {
            fs["boresight_yaw_deg"] >> cfg->boresight_yaw_deg;
        }
        if (!fs["boresight_pitch_deg"].empty()) {
            fs["boresight_pitch_deg"] >> cfg->boresight_pitch_deg;
        }
        // ── 轴独立增益 (精细 yaw 控制) ──
        if (!fs["kp_yaw"].empty()) {
            fs["kp_yaw"] >> cfg->kp_yaw;
        }
        if (!fs["kp_pitch"].empty()) {
            fs["kp_pitch"] >> cfg->kp_pitch;
        }
        if (!fs["lowpass_alpha_yaw"].empty()) {
            fs["lowpass_alpha_yaw"] >> cfg->lowpass_alpha_yaw;
        }
        if (!fs["lowpass_alpha_pitch"].empty()) {
            fs["lowpass_alpha_pitch"] >> cfg->lowpass_alpha_pitch;
        }
        if (!fs["ki_yaw"].empty()) {
            fs["ki_yaw"] >> cfg->ki_yaw;
        }
        if (!fs["ki_pitch"].empty()) {
            fs["ki_pitch"] >> cfg->ki_pitch;
        }
        if (!fs["ki_max_deg"].empty()) {
            fs["ki_max_deg"] >> cfg->ki_max_deg;
        }
        if (!fs["ff_gain_yaw"].empty()) {
            fs["ff_gain_yaw"] >> cfg->ff_gain_yaw;
        }
        if (!fs["ff_gain_pitch"].empty()) {
            fs["ff_gain_pitch"] >> cfg->ff_gain_pitch;
        }
        if (!fs["ff_alpha_yaw"].empty()) {
            fs["ff_alpha_yaw"] >> cfg->ff_alpha_yaw;
        }
        if (!fs["ff_alpha_pitch"].empty()) {
            fs["ff_alpha_pitch"] >> cfg->ff_alpha_pitch;
        }
        if (!fs["pitch_rate_assist_kp"].empty()) {
            fs["pitch_rate_assist_kp"] >> cfg->pitch_rate_assist_kp;
        }
        // ── 自适应增益 ──
        if (!fs["adaptive_kp"].empty()) {
            fs["adaptive_kp"] >> cfg->adaptive_kp;
        }
        if (!fs["kp_near"].empty()) {
            fs["kp_near"] >> cfg->kp_near;
        }
        if (!fs["kp_far"].empty()) {
            fs["kp_far"] >> cfg->kp_far;
        }
        if (!fs["kp_near_area"].empty()) {
            fs["kp_near_area"] >> cfg->kp_near_area;
        }
        if (!fs["kp_far_area"].empty()) {
            fs["kp_far_area"] >> cfg->kp_far_area;
        }
    }
    if (cam && camera_config_path.empty()) {
        // Fallback: read from control.yaml if camera.yaml not provided
        fs["fx"] >> cam->fx;
        fs["fy"] >> cam->fy;
        fs["cx"] >> cam->cx;
        fs["cy"] >> cam->cy;
    }
    return true;
}

}  // namespace control
