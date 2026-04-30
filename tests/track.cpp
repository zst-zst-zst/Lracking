// ============================================================================
// 测试5: 识别激光接收装置 + 跟踪照射 + 红蓝色算法
// ============================================================================
// 功能:
//   1. 打开相机, 用 Layer 2 YOLO 检测激光接收装置
//   2. Kalman + IoU 跟踪器维护目标轨迹
//   3. 红蓝色 HSV 颜色检测算法 (可视化颜色掩码)
//   4. 向云台发送跟踪目标坐标 (闭环照射)
//   近距离桌面测试, 不走 Layer 1 级联
// 用法: ./build/track [--config ../config/camera.yaml]
//                     [--model ../src/detect/model/export/layer2_laser_rx_fp16.engine]
//                               [--port /dev/ttyUSB0] [--color blue]
//                               [--video test.mp4]
//      ./build/track --color blue   # 蓝色敌方
//      ./build/track --color red    # 红色敌方
//      ./build/track --no-gimbal   # 禁用云台控制
// ============================================================================

#include "cascade_detector.h"
#include "target_tracker.h"
#include "galaxy_camera/galaxy_camera.h"
#include "gimbal_serial/protocol.h"
#include "gimbal_serial/serial_port.h"
#include "common/time_utils.h"
#include "control/config.h"
#include "control/controller.h"
#include "api.h"
#include "plotter.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {
std::atomic<bool> g_exit{false};
void onSignal(int) { g_exit.store(true); }

// ── 颜色检测结果 ──
struct ColorResult {
    cv::Mat mask;       // 颜色掩码 (单通道)
    float ratio;        // 颜色像素占比 (0~1)
    int pixel_count;    // 颜色像素数
};

// 检测指定颜色 (red/blue/purple)
ColorResult detectColor(const cv::Mat& bgr_roi, const std::string& color,
                       int min_sat = 80, int min_val = 80) {
    ColorResult result;
    result.ratio = 0.0f;
    result.pixel_count = 0;

    if (bgr_roi.empty()) {
        result.mask = cv::Mat();
        return result;
    }

    cv::Mat hsv;
    cv::cvtColor(bgr_roi, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    if (color == "blue") {
        // 蓝色 H∈[90, 130]
        cv::inRange(hsv, cv::Scalar(90, min_sat, min_val),
                    cv::Scalar(130, 255, 255), mask);
    } else if (color == "red") {
        // 红色 H∈[0,15] ∪ [165,180]
        cv::Mat mask1, mask2;
        cv::inRange(hsv, cv::Scalar(0, min_sat, min_val),
                    cv::Scalar(15, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(165, min_sat, min_val),
                    cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;
    } else if (color == "purple") {
        // 紫色 H∈[130, 165]，允许低饱和度（紫白色）
        // 标准紫色：H∈[130, 165], S≥min_sat
        // 紫白色：H∈[130, 165], S≥30 (允许更低的饱和度)
        cv::Mat mask_purple, mask_pale;
        cv::inRange(hsv, cv::Scalar(130, min_sat, min_val),
                    cv::Scalar(165, 255, 255), mask_purple);
        cv::inRange(hsv, cv::Scalar(130, 30, min_val),
                    cv::Scalar(165, min_sat - 1, 255), mask_pale);
        mask = mask_purple | mask_pale;
    } else {
        result.mask = cv::Mat();
        return result;
    }

    result.mask = mask;
    result.pixel_count = cv::countNonZero(mask);
    int total = mask.rows * mask.cols;
    result.ratio = total > 0 ? static_cast<float>(result.pixel_count) / total : 0.0f;
    return result;
}

// ── 颜色确认逻辑 ──
struct ColorConfirmation {
    std::string detected_color;  // red, blue, purple, unknown
    bool is_enemy;                // 是否为敌方颜色
    float confidence;             // 确信度
};

ColorConfirmation confirmEnemyColor(const cv::Mat& bgr_roi, const std::string& enemy_color,
                                    float enemy_threshold = 0.15f, float purple_threshold = 0.10f) {
    ColorConfirmation conf;
    conf.is_enemy = false;
    conf.confidence = 0.0f;

    if (bgr_roi.empty()) {
        conf.detected_color = "unknown";
        return conf;
    }

    // 检测三种颜色
    auto red_res = detectColor(bgr_roi, "red");
    auto blue_res = detectColor(bgr_roi, "blue");
    auto purple_res = detectColor(bgr_roi, "purple");

    // 找出占比最高的颜色
    float max_ratio = std::max({red_res.ratio, blue_res.ratio, purple_res.ratio});
    conf.confidence = max_ratio;

    if (max_ratio < 0.05f) {
        conf.detected_color = "unknown";
        return conf;
    }

    // 紫色优先：紫色说明追踪效果好
    if (purple_res.ratio >= purple_threshold && purple_res.ratio >= max_ratio * 0.8f) {
        conf.detected_color = "purple";
        // 紫色不是红蓝，但说明追踪稳定
        return conf;
    }

    // 判断是否为敌方颜色
    if (enemy_color == "red") {
        if (red_res.ratio >= enemy_threshold && red_res.ratio >= blue_res.ratio) {
            conf.detected_color = "red";
            conf.is_enemy = true;
        } else if (blue_res.ratio >= enemy_threshold) {
            conf.detected_color = "blue";
            conf.is_enemy = false;  // 敌方是红色，检测到蓝色
        } else {
            conf.detected_color = "unknown";
        }
    } else {  // enemy_color == "blue"
        if (blue_res.ratio >= enemy_threshold && blue_res.ratio >= red_res.ratio) {
            conf.detected_color = "blue";
            conf.is_enemy = true;
        } else if (red_res.ratio >= enemy_threshold) {
            conf.detected_color = "red";
            conf.is_enemy = false;  // 敌方是蓝色，检测到红色
        } else {
            conf.detected_color = "unknown";
        }
    }

    return conf;
}

// 分析轮廓形状: 是否有紧凑光斑 (激光模块) vs 细长条 (灯带)
struct ShapeAnalysis {
    bool has_compact = false;     // 有紧凑轮廓 (可能是激光模块)
    bool has_elongated = false;   // 有细长轮廓 (可能是灯带)
    float max_elongation = 0.0f;  // 最大长宽比
    float min_elongation = 999.0f;// 最小长宽比
};

ShapeAnalysis analyzeContourShapes(const cv::Mat& mask, float elongation_thresh = 2.5f) {
    ShapeAnalysis sa;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& c : contours) {
        if (cv::contourArea(c) < 20) continue;  // 忽略太小的轮廓
        cv::RotatedRect rr = cv::minAreaRect(c);
        float w = std::max(rr.size.width, rr.size.height);
        float h = std::min(rr.size.width, rr.size.height);
        float elongation = (h > 0) ? w / h : 0.0f;
        sa.max_elongation = std::max(sa.max_elongation, elongation);
        sa.min_elongation = std::min(sa.min_elongation, elongation);
        if (elongation > elongation_thresh) {
            sa.has_elongated = true;
        } else {
            sa.has_compact = true;
        }
    }
    return sa;
}

}  // namespace

int main(int argc, char** argv) {
    std::string camera_config = "config/camera.yaml";
    std::string model_path = "src/detect/model/export/layer2_laser_rx_fp16.engine";
    std::string control_config = "config/control.yaml";
    std::string serial_port = "/dev/ttyUSB0";   
    int serial_baud = 115200;
    std::string enemy_color = "red";
    std::string video_path;
    bool enable_gimbal = true;
    bool show_color_mask = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) camera_config = argv[++i];
        else if (arg == "--model" && i + 1 < argc) model_path = argv[++i];
        else if (arg == "--control" && i + 1 < argc) control_config = argv[++i];
        else if (arg == "--port" && i + 1 < argc) serial_port = argv[++i];
        else if (arg == "--baud" && i + 1 < argc) serial_baud = std::stoi(argv[++i]);
        else if (arg == "--color" && i + 1 < argc) enemy_color = argv[++i];
        else if (arg == "--video" && i + 1 < argc) video_path = argv[++i];
        else if (arg == "--no-gimbal") enable_gimbal = false;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::cout << "=== 跟踪照射 + 红蓝色算法测试 ===\n";
    std::cout << "敌方颜色: " << enemy_color << "\n";
    std::cout << "云台控制: " << (enable_gimbal ? "开启" : "关闭") << "\n\n";

    // ── TRT 引擎 ──
    TRTInferX::Api yolo;
    {
        TRTInferX::EngineConfig ecfg;
        ecfg.engine_path = model_path;
        ecfg.target_w = 256;
        ecfg.target_h = 256;
        ecfg.max_batch = 1;
        ecfg.num_classes = 3;
        ecfg.prep = TRTInferX::PreprocessMode::LETTERBOX;
        ecfg.out_mode = TRTInferX::OutputMode::RAW_ONLY;
        if (!yolo.load(ecfg)) {
            std::cerr << "✗ 模型加载失败: " << model_path << "\n";
            return 1;
        }
    }
    TRTInferX::InferOptions yolo_opt;
    yolo_opt.conf = 0.40f;
    yolo_opt.iou = 0.45f;
    std::cout << "✓ YOLO 模型已加载\n";

    // ── 跟踪器 ──
    solve::TargetTracker tracker;
    // 从 cascade.yaml 加载跟踪器配置
    std::string cascade_config = "config/cascade.yaml";
    if (tracker.loadConfig(cascade_config)) {
        std::cout << "✓ 跟踪器配置已加载: " << cascade_config << "\n";
    } else {
        std::cout << "✗ 跟踪器配置加载失败，使用默认参数\n";
    }
    std::cout << "✓ 跟踪器已初始化\n";

    // ── 串口 + 控制器 ──
    gimbal_serial::SerialPort serial;
    gimbal_serial::FrameParser parser;
    gimbal_serial::GimbalState gimbal_state;
    bool has_gimbal_state = false;
    control::ControlConfig ctrl_cfg;
    common::CameraModel cam_model;
    common::Boresight boresight;

    double laser_offset_y = 0.033;   // 默认值
    double laser_offset_x = 0.0;
    double target_height_m = 0.050;

    if (enable_gimbal) {
        if (serial.open(serial_port, serial_baud)) {
            std::cout << "✓ 串口已打开: " << serial_port << "\n";
        } else {
            std::cerr << "✗ 串口打开失败, 云台控制已禁用\n";
            enable_gimbal = false;
        }
        control::loadControlConfig(control_config, &ctrl_cfg, &cam_model, &boresight, camera_config);

        // 从 control.yaml 读取激光偏移参数
        cv::FileStorage fs(control_config, cv::FileStorage::READ);
        if (fs.isOpened()) {
            if (!fs["laser_offset_y"].empty()) fs["laser_offset_y"] >> laser_offset_y;
            if (!fs["laser_offset_x"].empty()) fs["laser_offset_x"] >> laser_offset_x;
            if (!fs["target_height_m"].empty()) fs["target_height_m"] >> target_height_m;
        }
        std::cout << "✓ 激光偏移: y=" << laser_offset_y << "m x=" << laser_offset_x
                  << "m 目标高度=" << target_height_m << "m\n";
    }
    control::Controller controller(ctrl_cfg);
    tools::Plotter plotter;  // UDP JSON → PlotJuggler (127.0.0.1:9870)

    // ── 视频源 ──
    cv::VideoCapture cap;
    galaxy_camera::GalaxyCamera galaxy;
    bool use_galaxy = video_path.empty();

    if (use_galaxy) {
        galaxy_camera::CameraConfig gcfg;
        galaxy_camera::loadCameraConfig(camera_config, &gcfg);
        if (!galaxy.open(gcfg) || !galaxy.startGrabbing()) {
            std::cerr << "✗ Galaxy 相机失败, 尝试 OpenCV(0)\n";
            use_galaxy = false;
            cap.open(0);
        } else {
            std::cout << "✓ Galaxy 相机已打开\n";
        }
    } else {
        cap.open(video_path);
    }
    if (!use_galaxy && !cap.isOpened()) {
        std::cerr << "✗ 视频源打开失败\n";
        return 1;
    }

    cv::namedWindow("Track & Illuminate", cv::WINDOW_NORMAL);
    int frame_count = 0;
    auto fps_start = std::chrono::steady_clock::now();
    std::vector<uint8_t> rx_buf(2048);

    std::cout << "正在运行... (q/ESC退出, c=切换颜色掩码显示)\n";

    while (!g_exit.load()) {
        cv::Mat frame;
        if (use_galaxy) {
            galaxy_camera::Frame gf;
            if (!galaxy.read(&gf, 50)) continue;
            if (gf.bgr.empty()) continue;
            frame = gf.bgr.clone();
        } else {
            if (!cap.read(frame) || frame.empty()) break;
        }

        int64_t ts = common::nowMs();

        // ── 串口接收 ──
        if (enable_gimbal) {
            int n = serial.read(rx_buf.data(), static_cast<int>(rx_buf.size()), 5);
            static int serial_frame_cnt = 0;
            serial_frame_cnt++;
            bool parsed = false;
            if (n > 0) {
                parsed = parser.push(rx_buf.data(), static_cast<size_t>(n), &gimbal_state);
                if (parsed) {
                    has_gimbal_state = true;
                }
            }
            // 前100帧每帧打印, 之后每30帧打印
            if (serial_frame_cnt <= 100 || serial_frame_cnt % 30 == 0) {
                const auto& pst = parser.stats();
                std::cout << "DBG serial[" << serial_frame_cnt << "] n=" << n
                          << " parsed=" << parsed
                          << " has_state=" << has_gimbal_state
                          << " bad=" << pst.bad_frames
                          << " discard=" << pst.discarded_bytes;
                if (has_gimbal_state) {
                    std::cout << " yaw=" << gimbal_state.yaw
                              << " pitch=" << gimbal_state.pitch;
                }
                std::cout << "\n";
            }
        }

        // ── 检测 ──
        TRTInferX::ImageInput img;
        img.data = frame.data;
        img.width = frame.cols;
        img.height = frame.rows;
        img.stride_bytes = static_cast<int>(frame.step[0]);
        img.color = TRTInferX::ColorSpace::BGR;
        img.layout = TRTInferX::Layout::HWC;
        img.dtype = TRTInferX::DType::UINT8;
        img.prep = TRTInferX::PreprocessMode::LETTERBOX;
        img.target_w = 256;
        img.target_h = 256;
        img.timestamp_ms = ts;

        auto infer_results = yolo.infer({img}, yolo_opt);
        static const std::vector<TRTInferX::Det> empty_dets;
        const auto& dets = infer_results.empty() ? empty_dets : infer_results[0];

        // 调试：显示检测数量
        static int det_debug_count = 0;
        if (++det_debug_count % 60 == 0) {
            std::cout << "DBG detections=" << dets.size() << "\n";
        }

        // ── 跟踪 ──
        std::vector<std::pair<cv::Rect2f, float>> tracker_input;
        for (const auto& d : dets) {
            // Accept all 3 classes: blue (0), purple (1), red (2)
            tracker_input.emplace_back(
                cv::Rect2f(d.x1, d.y1, d.x2 - d.x1, d.y2 - d.y1),
                d.score);
        }
        auto tracked = tracker.update(tracker_input, ts);

        // ── 颜色分析 (对每个检测结果) ──
        for (const auto& d : dets) {
            // Accept all 3 classes: blue (0), purple (1), red (2)
            cv::Rect det_bbox(static_cast<int>(d.x1), static_cast<int>(d.y1),
                              static_cast<int>(d.x2 - d.x1), static_cast<int>(d.y2 - d.y1));
            cv::Rect roi = det_bbox & cv::Rect(0, 0, frame.cols, frame.rows);
            if (roi.width < 5 || roi.height < 5) continue;

            cv::Mat roi_img = frame(roi);
            auto conf = confirmEnemyColor(roi_img, enemy_color);
            auto sa = analyzeContourShapes(detectColor(roi_img, conf.detected_color).mask);

            // 颜色确认状态显示
            std::string status;
            cv::Scalar status_color;
            if (conf.detected_color == "purple") {
                status = "PURPLE";
                status_color = cv::Scalar(200, 0, 200);  // 紫色
            } else if (conf.detected_color == enemy_color && conf.is_enemy) {
                status = "ENEMY";
                status_color = (enemy_color == "red") ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            } else if (conf.detected_color != enemy_color && conf.detected_color != "unknown") {
                status = "FRIEND";
                status_color = (conf.detected_color == "red") ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            } else {
                status = "UNKNOWN";
                status_color = cv::Scalar(180, 180, 180);  // 灰色
            }

            // 在检测框旁标注颜色确认结果（HSV 检测）
            std::string hsv_info = cv::format("HSV:%s %.0f%%",
                                          status.c_str(),
                                          conf.confidence * 100.0f);
            cv::putText(frame, hsv_info,
                        cv::Point(det_bbox.x, det_bbox.y + det_bbox.height + 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, status_color, 1);

            // 颜色掩码叠加显示
            if (show_color_mask && !conf.detected_color.empty() && conf.detected_color != "unknown") {
                cv::Mat color_overlay;
                auto color_mask = detectColor(roi_img, conf.detected_color).mask;
                cv::cvtColor(color_mask, color_overlay, cv::COLOR_GRAY2BGR);
                if (conf.detected_color == "blue") {
                    color_overlay.setTo(cv::Scalar(255, 0, 0), color_mask);
                } else if (conf.detected_color == "red") {
                    color_overlay.setTo(cv::Scalar(0, 0, 255), color_mask);
                } else if (conf.detected_color == "purple") {
                    color_overlay.setTo(cv::Scalar(200, 0, 200), color_mask);
                }
                cv::addWeighted(roi_img, 0.6, color_overlay, 0.4, 0, roi_img);
            }
        }

        // ── 绘制检测 (按类别颜色) ──
        for (const auto& d : dets) {
            // Color by class: blue (0)=blue, purple (1)=purple, red (2)=red
            cv::Scalar color;
            if (d.cls == 0) color = cv::Scalar(255, 0, 0);      // blue
            else if (d.cls == 1) color = cv::Scalar(255, 0, 255); // purple
            else color = cv::Scalar(0, 0, 255);                // red

            cv::Rect det_bbox(static_cast<int>(d.x1), static_cast<int>(d.y1),
                              static_cast<int>(d.x2 - d.x1), static_cast<int>(d.y2 - d.y1));
            cv::rectangle(frame, det_bbox, color, 1);
        }

        // ── 绘制跟踪 ──
        for (const auto& tt : tracked) {
            cv::Rect bbox(tt.bbox.x, tt.bbox.y, tt.bbox.width, tt.bbox.height);
            cv::Point center(tt.center.x, tt.center.y);
            cv::Scalar color = (tt.state == solve::TrackState::CONFIRMED)
                                   ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);
            cv::rectangle(frame, bbox, color, 2);
            cv::drawMarker(frame, center, color, cv::MARKER_CROSS, 20, 2);

            // 显示跟踪状态
            std::string state_str = (tt.state == solve::TrackState::CONFIRMED) ? "CONFIRMED" :
                                   (tt.state == solve::TrackState::TENTATIVE) ? "TENTATIVE" : "LOST";
            cv::putText(frame, cv::format("ID:%d %s", tt.track_id, state_str.c_str()),
                        cv::Point(bbox.x, bbox.y - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);

            // 速度箭头
            float speed = std::sqrt(tt.velocity.x * tt.velocity.x + tt.velocity.y * tt.velocity.y);
            if (speed > 1.0f) {
                cv::arrowedLine(frame, center,
                                cv::Point(center.x + tt.velocity.x * 3,
                                          center.y + tt.velocity.y * 3),
                                color, 2, cv::LINE_AA, 0, 0.3);
            }
        }

        // ── 云台控制: 跟踪主目标 ──
        bool has_target = tracker.hasPrimaryTarget();

        // 调试：显示目标状态和电控反馈
        static int target_debug_count = 0;
        if (++target_debug_count % 60 == 0) {
            std::cout << "DBG tracks=" << tracked.size() << " has_target=" << has_target
                      << " has_gimbal_state=" << has_gimbal_state << "\n";
        }

        if (enable_gimbal && has_target) {
            auto primary = tracker.primaryTarget();
            common::TargetMeasurement meas;
            meas.valid = true;
            meas.timestamp = ts;
            meas.uv = primary.center;
            meas.confidence = primary.confidence;

            // ── 角度域补偿 ──
            // 控制器用光心 (cx,cy) 作为 boresight，做纯跟踪
            // 然后在输出角度上加激光视差补偿: dpitch = atan(laser_offset / Z)
            // laser_offset_y, laser_offset_x, target_height_m 从 control.yaml 读取
            constexpr double kRadToDeg = 180.0 / M_PI;

            // boresight 设为光心, 控制器只管纯跟踪
            common::Boresight center_bs;
            center_bs.u_L = cam_model.cx;
            center_bs.v_L = cam_model.cy;

            auto cmd = controller.update(meas, cam_model, center_bs, gimbal_state);
            cmd.timestamp = common::nowMs();

            // 角度域激光补偿: 根据 bbox 高度估算距离, 再算角度偏移
            float bbox_h = primary.bbox.height;
            double Z_est = 999.0;  // 默认远距离(不补偿)
            double dpitch_deg = 0.0;
            double dyaw_deg = 0.0;
            if (bbox_h > 2.0f) {
                Z_est = cam_model.fy * target_height_m / bbox_h;
                dpitch_deg = std::atan(laser_offset_y / Z_est) * kRadToDeg;
                if (std::abs(laser_offset_x) > 1e-6)
                    dyaw_deg = std::atan(laser_offset_x / Z_est) * kRadToDeg;
            }
            // pitch_sign=-1 → 相机Y朝下, 激光在下方需要云台向下补偿
            cmd.pitch += static_cast<float>(ctrl_cfg.pitch_sign * dpitch_deg);
            cmd.yaw   += static_cast<float>(ctrl_cfg.yaw_sign * dyaw_deg);

            // 调试输出
            static int debug_count = 0;
            if (++debug_count % 30 == 0) {
                std::cout << "DBG gimbal cmd: mode=" << static_cast<int>(cmd.mode)
                          << " yaw=" << cmd.yaw << " pitch=" << cmd.pitch
                          << " dpitch=" << dpitch_deg << "° Z=" << Z_est << "m"
                          << " uv=(" << primary.center.x << "," << primary.center.y << ")"
                          << " bbox_h=" << bbox_h << "\n";
            }

            // PlotJuggler 实时数据
            {
                nlohmann::json j;
                j["cmd_yaw"] = cmd.yaw;
                j["cmd_pitch"] = cmd.pitch;
                j["state_yaw"] = gimbal_state.yaw;
                j["state_pitch"] = gimbal_state.pitch;
                j["target_u"] = primary.center.x;
                j["target_v"] = primary.center.y;
                j["Z_est"] = Z_est;
                j["dpitch_deg"] = dpitch_deg;
                j["error_yaw"] = cmd.yaw - gimbal_state.yaw;
                j["error_pitch"] = cmd.pitch - gimbal_state.pitch;
                j["bbox_h"] = bbox_h;
                plotter.plot(j);
            }

            uint8_t tx_frame[gimbal_serial::kTxFrameSize]{};
            if (has_gimbal_state) {
                gimbal_serial::packGimbalCommand(cmd, gimbal_state, tx_frame);
            } else {
                gimbal_serial::packGimbalCommand(cmd, tx_frame);
            }
            int written = serial.write(tx_frame, static_cast<int>(sizeof(tx_frame)));

            // 调试：显示串口写入
            static int serial_debug_count = 0;
            if (++serial_debug_count % 30 == 0) {
                std::cout << "DBG serial wrote=" << written << " bytes\n";
            }

            // 瞄准十字丝 (黄色)
            cv::drawMarker(frame, cv::Point(primary.center.x, primary.center.y),
                           cv::Scalar(0, 255, 255), cv::MARKER_TILTED_CROSS, 30, 3, cv::LINE_AA);
        } else if (enable_gimbal) {
            // 无目标: 发送 idle
            common::TargetMeasurement meas;
            meas.valid = false;
            meas.timestamp = ts;
            auto cmd = controller.update(meas, cam_model, boresight, gimbal_state);
            cmd.timestamp = common::nowMs();
            uint8_t tx_frame[gimbal_serial::kTxFrameSize]{};
            gimbal_serial::packGimbalCommand(cmd, tx_frame);
            serial.write(tx_frame, static_cast<int>(sizeof(tx_frame)));
        }

        // ── HUD ──
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_start).count();
        double fps = elapsed > 0 ? frame_count * 1000.0 / elapsed : 0;
        if (elapsed >= 3000) { frame_count = 0; fps_start = now; }

        int y = 20;
        auto putHud = [&](const std::string& text, cv::Scalar c) {
            cv::putText(frame, text, cv::Point(10, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv::LINE_AA);
            y += 18;
        };
        putHud(cv::format("FPS: %.1f  Color: %s  Tracks: %d",
                          fps, enemy_color.c_str(), static_cast<int>(tracked.size())),
               cv::Scalar(0, 255, 255));
        putHud(cv::format("Gimbal: %s  Target: %s",
                          enable_gimbal ? "ON" : "OFF",
                          has_target ? "LOCKED" : "SEARCHING"),
               has_target ? cv::Scalar(0, 255, 0) : cv::Scalar(100, 100, 100));
        if (has_gimbal_state) {
            putHud(cv::format("Gimbal pitch=%.1f yaw=%.1f",
                              gimbal_state.pitch, gimbal_state.yaw),
                   cv::Scalar(200, 200, 0));
        }
        putHud(show_color_mask ? "[c] Color mask: ON" : "[c] Color mask: OFF",
               cv::Scalar(180, 180, 180));

        cv::imshow("Track & Illuminate", frame);
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') break;
        if (key == 'c' || key == 'C') show_color_mask = !show_color_mask;
    }

    if (enable_gimbal) serial.close();
    if (use_galaxy) { galaxy.stopGrabbing(); galaxy.close(); }
    tracker.saveTrajectoryLog();
    cv::destroyAllWindows();
    std::cout << "=== 测试结束 ===\n";
    return 0;
}
