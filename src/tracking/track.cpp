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
//      ./src/build/tracking/tracking --color red    # 红色敌方
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

#include "color_voting.h"
#include "control_learner.h"
#include "visualization.h"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

// 引入解耦后的模块: 可视化绘图 / 颜色判定 / 控制学习器
// (visualization.h, color_voting.h, control_learner.h 已在文件头 include)
using tracking_app::drawPredictedTrail;
using tracking_app::ColorResult;
using tracking_app::ColorConfirmation;
using tracking_app::ColorObservation;
using tracking_app::DetColorInfo;
using tracking_app::PurpleTransitionRecord;
using tracking_app::ControlLearner;
using tracking_app::detectColor;
using tracking_app::observeEnemyColor;

namespace {
std::atomic<bool> g_exit{false};
void onSignal(int) { g_exit.store(true); }


// ── 亚像素精化: 对检测框中心做亚像素修正 ──
// 目标特征: 上下发光灯带, 中间约3cm不发光
// 策略: 用二值化 mask 的几何矩(非强度加权)找质心
//   → 上下两个亮区对称 → 质心自然在正中间
//   → 即使亮度不均, 二值化后面积相等 → 不会偏向更亮一侧
// 对 2-3px 的远距离小目标可将精度从 ±1px 提升到 ±0.3px
cv::Point2f subpixelRefineCenter(const cv::Mat& bgr, const cv::Rect2f& bbox_f) {
    // bbox 中心 (YOLO 粗略值)
    cv::Point2f coarse_center(bbox_f.x + bbox_f.width * 0.5f,
                              bbox_f.y + bbox_f.height * 0.5f);

    // 扩展 ROI (1.5×), 多取周围像素以提高矩计算稳定性
    const float pad = 0.25f;
    int x1 = static_cast<int>(std::floor(bbox_f.x - bbox_f.width * pad));
    int y1 = static_cast<int>(std::floor(bbox_f.y - bbox_f.height * pad));
    int x2 = static_cast<int>(std::ceil(bbox_f.x + bbox_f.width * (1.0f + pad)));
    int y2 = static_cast<int>(std::ceil(bbox_f.y + bbox_f.height * (1.0f + pad)));
    x1 = std::max(0, x1);
    y1 = std::max(0, y1);
    x2 = std::min(bgr.cols, x2);
    y2 = std::min(bgr.rows, y2);
    if (x2 - x1 < 2 || y2 - y1 < 2) return coarse_center;

    cv::Mat roi = bgr(cv::Rect(x1, y1, x2 - x1, y2 - y1));

    // 转灰度
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

    // 自适应阈值: 分离发光区域(上下灯带) vs 暗区(中间+背景)
    double mean_val = cv::mean(gray)[0];
    double thresh = std::max(mean_val * 1.5, 40.0);  // 较高阈值, 只保留真正发光的像素
    cv::Mat mask;
    cv::threshold(gray, mask, thresh, 255, cv::THRESH_BINARY);

    // 用二值 mask 的几何矩 (非强度加权!)
    // 这样上下灯带即使亮度不同, 只要面积差不多, 质心就在正中间
    cv::Moments m_mask = cv::moments(mask, true);  // binary=true → 面积加权
    if (m_mask.m00 < 1.0) {
        // mask 为空, 降级到强度加权矩
        cv::Moments m_gray = cv::moments(gray, false);
        if (m_gray.m00 < 1e-6) return coarse_center;
        float refined_x = static_cast<float>(m_gray.m10 / m_gray.m00) + x1;
        float refined_y = static_cast<float>(m_gray.m01 / m_gray.m00) + y1;
        float dx = refined_x - coarse_center.x;
        float dy = refined_y - coarse_center.y;
        float max_shift = std::max(bbox_f.width, bbox_f.height) * 0.5f;
        if (std::abs(dx) > max_shift || std::abs(dy) > max_shift) return coarse_center;
        return cv::Point2f(refined_x, refined_y);
    }

    // 几何质心 = 上下发光区域的面积中心 → 自然在目标正中间
    float refined_x = static_cast<float>(m_mask.m10 / m_mask.m00) + x1;
    float refined_y = static_cast<float>(m_mask.m01 / m_mask.m00) + y1;

    // 安全限制: 精化结果不能偏离粗略中心太远 (防异常)
    const float max_shift = std::max(bbox_f.width, bbox_f.height) * 0.5f;
    float dx = refined_x - coarse_center.x;
    float dy = refined_y - coarse_center.y;
    if (std::abs(dx) > max_shift || std::abs(dy) > max_shift) {
        return coarse_center;
    }

    return cv::Point2f(refined_x, refined_y);
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
    std::string record_dir;
    bool enable_gimbal = true;
    bool enable_record = true;
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
        else if (arg == "--record-dir" && i + 1 < argc) record_dir = argv[++i];
        else if (arg == "--no-record") enable_record = false;
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
    bool gimbal_state_stale = false;  // state 数据过期 (长时间未成功解析)
    control::ControlConfig ctrl_cfg;
    common::CameraModel cam_model;

    if (enable_gimbal) {
        if (serial.open(serial_port, serial_baud)) {
            std::cout << "✓ 串口已打开: " << serial_port << "\n";
        } else {
            std::cerr << "✗ 串口打开失败, 云台控制已禁用\n";
            enable_gimbal = false;
        }
        control::loadControlConfig(control_config, &ctrl_cfg, &cam_model, camera_config);
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

    cv::VideoWriter writer;
    std::string record_path;
    if (enable_record) {
        if (record_dir.empty()) record_dir = "records/tracking";
        std::filesystem::create_directories(record_dir);
        auto now_t = std::chrono::system_clock::now();
        auto time_c = std::chrono::system_clock::to_time_t(now_t);
        std::tm ltm{};
        localtime_r(&time_c, &ltm);
        std::ostringstream fn;
        fn << "tracking_" << std::put_time(&ltm, "%Y-%m-%d_%H-%M-%S") << ".mp4";
        record_path = (std::filesystem::path(record_dir) / fn.str()).string();
    }

    cv::namedWindow("Track & Illuminate", cv::WINDOW_NORMAL);
    int frame_count = 0;
    auto fps_start = std::chrono::steady_clock::now();
    std::vector<uint8_t> rx_buf(2048);
    std::map<int, std::deque<cv::Point2f>> trail_history;
    std::map<int, int> trail_last_seen;
    std::map<int, std::string> trail_color;  // 每个 track_id 当前的颜色
    std::map<int, float> trail_ema_size;     // 每个 track_id 的 EMA 平滑边长 (像素)
    const int kTrailMaxLen = 180;
    const int kTrailKeepFrames = 90;

    const std::filesystem::path purple_log_path = "logs/tracking_purple_state.csv";
    std::filesystem::create_directories(purple_log_path.parent_path());
    const bool purple_log_new_file = !std::filesystem::exists(purple_log_path) ||
                                     std::filesystem::file_size(purple_log_path) == 0;
    std::ofstream purple_log(purple_log_path, std::ios::out | std::ios::app);
    if (purple_log.is_open() && purple_log_new_file) {
        purple_log << "ts_ms,track_id,prev_color,current_color,confidence,red_ratio,blue_ratio,purple_ratio,prediction_horizon_s,predicted_x,predicted_y,prediction_error_x,prediction_error_y,bbox_x,bbox_y,bbox_w,bbox_h,center_x,center_y,velocity_x,velocity_y,cmd_yaw,cmd_pitch,state_yaw,state_pitch,tracked_count,fps\n";
        purple_log.flush();
    }

    std::string last_primary_color = "unknown";
    PurpleTransitionRecord pending_purple_log;
    bool has_pending_purple_log = false;

    // ── 在线自学习系统 (角度空间) ──
    // 紫色 = 命中, 目标中心 = 激光落点
    // 残差 = 目标角度 - (boresight + 视差) → 非零说明 boresight 有偏
    // 用积分控制律修正: correction += alpha * residual
    // baseline + correction = 当前 boresight → 无累积漂移
    // 自适应学习率: 偏差大时快速收敛, 偏差小时精细微调
    // |correction| > 0.3° → alpha=0.15 (大步快走)
    // |correction| < 0.05° → alpha=0.03 (精细调)
    constexpr double kBsAlphaMax = 0.15;
    constexpr double kBsAlphaMin = 0.03;
    constexpr double kBsAlphaDecay = 5.0;  // 衰减系数
    const int bs_learn_min_hits = 3;     // 最少命中次数才开始修正
    int purple_hit_count = 0;
    double bs_correction_yaw = 0.0;    // 总修正量 yaw (deg)
    double bs_correction_pitch = 0.0;  // 总修正量 pitch (deg)
    // baseline: 标定值 (不变, 学习只改 correction)
    const double bs_baseline_yaw = ctrl_cfg.boresight_yaw_deg;
    const double bs_baseline_pitch = ctrl_cfg.boresight_pitch_deg;
    // 加载历史学习成果
    const std::filesystem::path bs_learn_path = "logs/bs_learned.yaml";
    {
        cv::FileStorage lfs(bs_learn_path.string(), cv::FileStorage::READ);
        if (lfs.isOpened()) {
            double ly = 0.0, lp = 0.0;
            int lhits = 0;
            lfs["correction_yaw_deg"] >> ly;
            lfs["correction_pitch_deg"] >> lp;
            lfs["total_hits"] >> lhits;
            if (lhits >= bs_learn_min_hits) {
                bs_correction_yaw = ly;
                bs_correction_pitch = lp;
                purple_hit_count = lhits;
                ctrl_cfg.boresight_yaw_deg = bs_baseline_yaw + ly;
                ctrl_cfg.boresight_pitch_deg = bs_baseline_pitch + lp;
                controller.updateConfig(ctrl_cfg);  // 关键: 把学习成果应用到控制器!
                std::cout << "BS_LEARN loaded: hits=" << lhits
                          << " corr_deg=(" << ly << "," << lp << ")"
                          << " bs_deg=(" << ctrl_cfg.boresight_yaw_deg << "," << ctrl_cfg.boresight_pitch_deg << ")\n";
            }
            lfs.release();
        }
    }
    // 确认激光补偿参数已加载
    std::cout << "CTRL_CONFIG: kp=" << ctrl_cfg.kp
              << " kp_yaw=" << ctrl_cfg.kp_yaw
              << " laser_offset_y=" << ctrl_cfg.laser_offset_y
              << " laser_offset_x=" << ctrl_cfg.laser_offset_x
              << " bs_yaw=" << ctrl_cfg.boresight_yaw_deg
              << " bs_pitch=" << ctrl_cfg.boresight_pitch_deg
              << " target_h=" << ctrl_cfg.target_height_m << "\n";

    // ── 控制参数自学习: 加载历史训练成果 ──
    const std::filesystem::path ctrl_learn_path = "logs/learned_control.yaml";
    ControlLearner ctrl_learn;
    const double base_ff_gain_yaw = ctrl_cfg.ff_gain_yaw;
    const double base_ff_gain_pitch = ctrl_cfg.ff_gain_pitch;
    {
        if (ctrl_learn.load(ctrl_learn_path.string())) {
            ctrl_learn.session_count++;
            ctrl_cfg.ff_gain_yaw = ctrl_learn.ff_gain_yaw;
            ctrl_cfg.ff_gain_pitch = ctrl_learn.ff_gain_pitch;
            controller.updateConfig(ctrl_cfg);
            std::cout << "CTRL_LEARN loaded: session=" << ctrl_learn.session_count
                      << " ff_yaw=" << ctrl_learn.ff_gain_yaw
                      << " ff_pitch=" << ctrl_learn.ff_gain_pitch
                      << " hit_ratio=" << ctrl_learn.ema_hit_ratio << "\n";
        } else {
            ctrl_learn.session_count = 1;
            ctrl_learn.ff_gain_yaw = base_ff_gain_yaw;
            ctrl_learn.ff_gain_pitch = base_ff_gain_pitch;
        }
    }

    // 参数学习记录 (记录每次命中时的全部状态, 供离线分析/训练神经网络)
    const std::filesystem::path learn_log_path = "logs/learning_samples.csv";
    const bool learn_log_new = !std::filesystem::exists(learn_log_path) ||
                               std::filesystem::file_size(learn_log_path) == 0;
    std::ofstream learn_log(learn_log_path, std::ios::out | std::ios::app);
    if (learn_log.is_open() && learn_log_new) {
        learn_log << "ts_ms,hit_count,target_u,target_v,bs_u,bs_v,err_u,err_v,"
                  << "corr_u,corr_v,bbox_area,vel_x,vel_y,"
                  << "cmd_yaw,cmd_pitch,state_yaw,state_pitch,"
                  << "ff_yaw_rate,ff_pitch_rate,kp_used\n";
        learn_log.flush();
    }

    // VideoWriter 推迟到拿到第一帧后再打开 (用真实 frame.size())
    // 关键 bug 修复: Galaxy 相机时 cap 未开, 旧代码兜底 1280x720 与实际 1280x1024 不匹配,
    // 导致 writer.write() 静默失败, 输出 mp4 只有头没有帧 (~258 字节, 无法播放).
    double record_fps = 30.0;
    if (cap.isOpened()) {
        double f = cap.get(cv::CAP_PROP_FPS);
        if (f > 1.0 && f < 240.0) record_fps = f;
    }
    if (enable_record) {
        std::cout << "录制(待开): " << record_path << " (尺寸=首帧, fps≈" << record_fps << ")\n";
    }

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

        // 首帧到达时按实际尺寸打开 VideoWriter (修复 Galaxy 1280x1024 ≠ 兜底 1280x720 的录制失败)
        if (enable_record && !writer.isOpened()) {
            writer.open(record_path,
                        cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                        record_fps, frame.size());
            if (writer.isOpened()) {
                std::cout << "录制已开: " << record_path
                          << " (" << frame.cols << "x" << frame.rows
                          << " @ " << record_fps << "fps)\n";
            } else {
                std::cerr << "✗ VideoWriter 打开失败, 录制关闭: " << record_path << "\n";
                enable_record = false;
            }
        }

        int64_t ts = common::nowMs();

        // ── 串口接收 ──
        if (enable_gimbal) {
            int64_t t_rx_start = common::nowMs();
            int n = serial.read(rx_buf.data(), static_cast<int>(rx_buf.size()), 5);
            int64_t t_rx_end = common::nowMs();
            static int serial_frame_cnt = 0;
            serial_frame_cnt++;
            static int64_t last_parsed_ts = 0;  // 上次成功解析的时间
            bool parsed = false;
            if (n > 0) {
                common::GimbalState new_state;
                parsed = parser.push(rx_buf.data(), static_cast<size_t>(n), &new_state);
                if (parsed) {
                    // 健全性检查: 拒绝物理上不可能的跳变 (>15°/帧 ≈ 450°/s)
                    // 串口数据损坏会导致 state 随机跳变, 直接用则控制发散
                    bool state_ok = true;
                    if (has_gimbal_state) {
                        double djump_yaw = std::abs(new_state.yaw - gimbal_state.yaw);
                        double djump_pitch = std::abs(new_state.pitch - gimbal_state.pitch);
                        if (djump_yaw > 15.0 || djump_pitch > 15.0) {
                            state_ok = false;
                            if (serial_frame_cnt % 30 == 0) {
                                std::cout << "WARN gimbal state jump rejected:"
                                          << " yaw " << gimbal_state.yaw << "->" << new_state.yaw
                                          << " pitch " << gimbal_state.pitch << "->" << new_state.pitch << "\n";
                            }
                        }
                    }
                    if (state_ok) {
                        gimbal_state = new_state;
                        has_gimbal_state = true;
                        last_parsed_ts = t_rx_end;
                        gimbal_state_stale = false;
                    }
                }
            }
            // 检测 state 过期: 超过 500ms 没有成功解析 → state 不可信
            if (has_gimbal_state && last_parsed_ts > 0 &&
                (t_rx_end - last_parsed_ts) > 500) {
                gimbal_state_stale = true;
            }
            // 前100帧每帧打印, 之后每30帧打印
            if (serial_frame_cnt <= 100 || serial_frame_cnt % 30 == 0) {
                const auto& pst = parser.stats();
                std::cout << "DBG serial[" << serial_frame_cnt << "] n=" << n
                          << " parsed=" << parsed
                          << " has_state=" << has_gimbal_state
                          << " bad=" << pst.bad_frames
                          << " discard=" << pst.discarded_bytes
                          << " rx_ms=" << (t_rx_end - t_rx_start);
                if (has_gimbal_state) {
                    std::cout << " yaw=" << gimbal_state.yaw
                              << " pitch=" << gimbal_state.pitch;
                }
                if (gimbal_state_stale) std::cout << " STALE!";
                // 前10帧: hex dump 帮助诊断协议不匹配
                if (serial_frame_cnt <= 10 && n > 0) {
                    std::cout << " hex[" << std::min(n, 20) << "]=";
                    for (int i = 0; i < std::min(n, 20); ++i) {
                        char buf[4]; snprintf(buf, sizeof(buf), "%02X", rx_buf[i]);
                        std::cout << buf;
                        if (i < std::min(n, 20) - 1) std::cout << " ";
                    }
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
            std::cout << "DBG detections=" << dets.size();
            for (const auto& d : dets) {
                cv::Rect2f bb(d.x1, d.y1, d.x2 - d.x1, d.y2 - d.y1);
                cv::Point2f coarse(bb.x + bb.width * 0.5f, bb.y + bb.height * 0.5f);
                cv::Point2f ref = subpixelRefineCenter(frame, bb);
                std::cout << " [" << bb.width << "x" << bb.height
                          << " shift=" << (ref.x - coarse.x) << "," << (ref.y - coarse.y) << "]";
            }
            std::cout << "\n";
        }

        // ── 跟踪 (亚像素精化后送入 Kalman) ──
        std::vector<std::pair<cv::Rect2f, float>> tracker_input;
        for (const auto& d : dets) {
            // Accept all 3 classes: blue (0), purple (1), red (2)
            cv::Rect2f raw_bbox(d.x1, d.y1, d.x2 - d.x1, d.y2 - d.y1);
            // 亚像素精化: 用图像矩找到更精确的中心
            cv::Point2f refined = subpixelRefineCenter(frame, raw_bbox);
            // 将 bbox 平移使中心对齐到精化后的位置
            float shift_x = refined.x - (raw_bbox.x + raw_bbox.width * 0.5f);
            float shift_y = refined.y - (raw_bbox.y + raw_bbox.height * 0.5f);
            cv::Rect2f refined_bbox(raw_bbox.x + shift_x, raw_bbox.y + shift_y,
                                    raw_bbox.width, raw_bbox.height);
            tracker_input.emplace_back(refined_bbox, d.score);
        }
        auto tracked = tracker.update(tracker_input, ts);

        // ── 颜色分析 (对每个检测结果) ──
        std::vector<DetColorInfo> det_color_infos;
        det_color_infos.reserve(dets.size());
        for (const auto& d : dets) {
            // Accept all 3 classes: blue (0), purple (1), red (2)
            cv::Rect det_bbox(static_cast<int>(d.x1), static_cast<int>(d.y1),
                              static_cast<int>(d.x2 - d.x1), static_cast<int>(d.y2 - d.y1));
            cv::Rect roi = det_bbox & cv::Rect(0, 0, frame.cols, frame.rows);
            if (roi.width < 5 || roi.height < 5) continue;

            cv::Mat roi_img = frame(roi);
            auto obs = observeEnemyColor(roi_img, enemy_color);
            [[maybe_unused]] const auto sa = analyzeContourShapes(detectColor(roi_img, obs.conf.detected_color).mask);

            // 颜色确认状态显示
            std::string status;
            cv::Scalar status_color;
            if (obs.conf.detected_color == "purple") {
                status = "PURPLE";
                status_color = cv::Scalar(200, 0, 200);  // 紫色
            } else if (obs.conf.detected_color == enemy_color && obs.conf.is_enemy) {
                status = "ENEMY";
                status_color = (enemy_color == "red") ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            } else if (obs.conf.detected_color != enemy_color && obs.conf.detected_color != "unknown") {
                status = "FRIEND";
                status_color = (obs.conf.detected_color == "red") ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            } else {
                status = "UNKNOWN";
                status_color = cv::Scalar(180, 180, 180);  // 灰色
            }

            // 在检测框旁标注颜色确认结果（HSV 检测）
            std::string hsv_info = cv::format("HSV:%s %.0f%%",
                                          status.c_str(),
                                          obs.conf.confidence * 100.0f);
            cv::putText(frame, hsv_info,
                        cv::Point(det_bbox.x, det_bbox.y + det_bbox.height + 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, status_color, 1);

            // 颜色掩码叠加显示
            if (show_color_mask && !obs.conf.detected_color.empty() && obs.conf.detected_color != "unknown") {
                cv::Mat color_overlay;
                auto color_mask = detectColor(roi_img, obs.conf.detected_color).mask;
                cv::cvtColor(color_mask, color_overlay, cv::COLOR_GRAY2BGR);
                if (obs.conf.detected_color == "blue") {
                    color_overlay.setTo(cv::Scalar(255, 0, 0), color_mask);
                } else if (obs.conf.detected_color == "red") {
                    color_overlay.setTo(cv::Scalar(0, 0, 255), color_mask);
                } else if (obs.conf.detected_color == "purple") {
                    color_overlay.setTo(cv::Scalar(200, 0, 200), color_mask);
                }
                cv::addWeighted(roi_img, 0.6, color_overlay, 0.4, 0, roi_img);
            }

            det_color_infos.push_back(DetColorInfo{
                static_cast<int>(det_color_infos.size()),
                cv::Point2f(det_bbox.x + det_bbox.width * 0.5f,
                            det_bbox.y + det_bbox.height * 0.5f),
                det_bbox,
                obs,
                status
            });
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

        // ── 绘制跟踪 (颜色编码轨迹: 红/蓝/紫/绿) ──
        for (const auto& tt : tracked) {
            cv::Rect bbox(tt.bbox.x, tt.bbox.y, tt.bbox.width, tt.bbox.height);
            cv::Point center(tt.center.x, tt.center.y);

            // 匹配此 track_id 最近的颜色检测结果
            std::string det_color_str = "unknown";
            float best_match_dist = std::numeric_limits<float>::max();
            for (const auto& dci : det_color_infos) {
                float dx = dci.center.x - tt.center.x;
                float dy = dci.center.y - tt.center.y;
                float d2 = dx * dx + dy * dy;
                if (d2 < best_match_dist) {
                    best_match_dist = d2;
                    det_color_str = dci.obs.conf.detected_color;
                }
            }
            if (best_match_dist < 10000.0f && det_color_str != "unknown") {
                trail_color[tt.track_id] = det_color_str;
            }

            // 根据检测颜色选择轨迹颜色
            cv::Scalar trail_col(0, 255, 0);  // 默认绿色 (confirmed)
            const auto& tc = trail_color[tt.track_id];
            if (tc == "red")         trail_col = cv::Scalar(60, 60, 255);
            else if (tc == "blue")   trail_col = cv::Scalar(255, 100, 60);
            else if (tc == "purple") trail_col = cv::Scalar(255, 0, 255);

            cv::Scalar box_color = (tt.state == solve::TrackState::CONFIRMED)
                                       ? trail_col : cv::Scalar(128, 128, 128);

            // ── 视觉稳定化: 锁方形 + 尺寸 EMA + 聚焦环 ──
            // laser_rx 物理 50×50mm 近似正方形, 强制绘制为方框, 避免 YOLO 扭曲
            // 彻底平滑尺寸抗抖动, 同时 5帧指数 ≈ 足以跟上真实远近变化
            float raw_side = std::max(bbox.width, bbox.height) * 1.0f;
            auto &ema = trail_ema_size[tt.track_id];
            if (ema <= 0.f) ema = raw_side;
            ema = 0.7f * ema + 0.3f * raw_side;
            int side = std::max(8, static_cast<int>(ema));
            cv::Rect square_bbox(center.x - side / 2, center.y - side / 2, side, side);
            cv::rectangle(frame, square_bbox, box_color, 2);
            // 中心十字 + 固定小圆环, 不管远近都方便肉眼定位
            cv::drawMarker(frame, center, box_color, cv::MARKER_CROSS, 20, 2);
            cv::circle(frame, center, 10, box_color, 1, cv::LINE_AA);

            auto& trail = trail_history[tt.track_id];
            trail.push_back(tt.center);
            while (trail.size() > static_cast<size_t>(kTrailMaxLen)) {
                trail.pop_front();
            }
            trail_last_seen[tt.track_id] = frame_count;
            drawPredictedTrail(frame, trail, tt.velocity_px_s, trail_col, kTrailMaxLen);

            // 显示跟踪状态 + 颜色
            std::string state_str = (tt.state == solve::TrackState::CONFIRMED) ? "CONFIRMED" :
                                   (tt.state == solve::TrackState::TENTATIVE) ? "TENTATIVE" : "LOST";
            cv::putText(frame, cv::format("ID:%d %s [%s]", tt.track_id, state_str.c_str(), tc.c_str()),
                        cv::Point(bbox.x, bbox.y - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, box_color, 1);

            // 速度箭头
            float speed = std::sqrt(tt.velocity.x * tt.velocity.x + tt.velocity.y * tt.velocity.y);
            if (speed > 1.0f) {
                cv::arrowedLine(frame, center,
                                cv::Point(center.x + tt.velocity.x * 3,
                                          center.y + tt.velocity.y * 3),
                                box_color, 2, cv::LINE_AA, 0, 0.3);
            }
        }

        for (auto it = trail_history.begin(); it != trail_history.end();) {
            const int track_id = it->first;
            auto last_seen_it = trail_last_seen.find(track_id);
            if (last_seen_it != trail_last_seen.end() &&
                frame_count - last_seen_it->second > kTrailKeepFrames) {
                trail_last_seen.erase(last_seen_it);
                trail_color.erase(track_id);
                trail_ema_size.erase(track_id);
                it = trail_history.erase(it);
            } else {
                ++it;
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

        // 如果 gimbal state 过期 (串口解析持续失败), 用上次指令位置代替
        // 避免用冻结的 state 做闭环导致开环振荡
        static float dead_reckon_yaw = 0.0f, dead_reckon_pitch = 0.0f;
        common::GimbalState ctrl_state = gimbal_state;
        if (gimbal_state_stale || !has_gimbal_state) {
            ctrl_state.yaw = dead_reckon_yaw;
            ctrl_state.pitch = dead_reckon_pitch;
            ctrl_state.yaw_rate = 0.0f;
            ctrl_state.pitch_rate = 0.0f;
        }

        if (enable_gimbal && has_target) {
            auto primary = tracker.primaryTarget();
            common::TargetMeasurement meas;
            meas.valid = true;
            meas.timestamp = ts;
            meas.uv = primary.center;
            meas.confidence = primary.confidence;
            meas.bbox_area = primary.bbox.width * primary.bbox.height;
            meas.bbox_height = primary.bbox.height;
            meas.velocity = primary.velocity_px_s;

            auto cmd = controller.update(meas, cam_model, ctrl_state);
            cmd.timestamp = common::nowMs();

            const DetColorInfo* best_color_info = nullptr;
            float best_dist_sq = std::numeric_limits<float>::max();
            for (const auto& info : det_color_infos) {
                const float dx = info.center.x - primary.center.x;
                const float dy = info.center.y - primary.center.y;
                const float dist_sq = dx * dx + dy * dy;
                if (dist_sq < best_dist_sq) {
                    best_dist_sq = dist_sq;
                    best_color_info = &info;
                }
            }

            std::string current_primary_color = "unknown";
            float current_primary_confidence = 0.0f;
            float current_red_ratio = 0.0f;
            float current_blue_ratio = 0.0f;
            float current_purple_ratio = 0.0f;
            if (best_color_info && best_dist_sq < 10000.0f) {
                current_primary_color = best_color_info->obs.conf.detected_color;
                current_primary_confidence = best_color_info->obs.conf.confidence;
                current_red_ratio = best_color_info->obs.red_ratio;
                current_blue_ratio = best_color_info->obs.blue_ratio;
                current_purple_ratio = best_color_info->obs.purple_ratio;
            }

            const std::string prev_primary_color = last_primary_color;
            const bool should_log_purple =
                current_primary_color == "purple" &&
                (prev_primary_color == "red" || prev_primary_color == "blue");
            if (should_log_purple) {
                has_pending_purple_log = true;
                pending_purple_log.valid = true;
                pending_purple_log.ts_ms = ts;
                pending_purple_log.track_id = primary.track_id;
                pending_purple_log.prev_color = prev_primary_color;
                pending_purple_log.current_color = current_primary_color;
                pending_purple_log.confidence = current_primary_confidence;
                pending_purple_log.red_ratio = current_red_ratio;
                pending_purple_log.blue_ratio = current_blue_ratio;
                pending_purple_log.purple_ratio = current_purple_ratio;
                pending_purple_log.prediction_horizon_s = 0.5f;
                const cv::Point2f predicted_center = primary.center + primary.velocity_px_s * pending_purple_log.prediction_horizon_s;
                pending_purple_log.predicted_x = predicted_center.x;
                pending_purple_log.predicted_y = predicted_center.y;
                pending_purple_log.prediction_error_x = predicted_center.x - primary.center.x;
                pending_purple_log.prediction_error_y = predicted_center.y - primary.center.y;
                pending_purple_log.bbox = cv::Rect(cvRound(primary.bbox.x), cvRound(primary.bbox.y),
                                                   cvRound(primary.bbox.width), cvRound(primary.bbox.height));
                pending_purple_log.center = primary.center;
                pending_purple_log.velocity = primary.velocity;
                pending_purple_log.cmd_yaw = cmd.yaw;
                pending_purple_log.cmd_pitch = cmd.pitch;
                pending_purple_log.state_yaw = gimbal_state.yaw;
                pending_purple_log.state_pitch = gimbal_state.pitch;
                pending_purple_log.tracked_count = static_cast<int>(tracked.size());
                pending_purple_log.fps = 0.0;
            }

            last_primary_color = current_primary_color;

            // ── 控制参数在线学习: 每帧采集跟踪误差信号 ──
            // state 过期时暂停学习, 防止开环误差把参数推到极值
            if (!gimbal_state_stale && has_gimbal_state) {
                constexpr double kR2D_cl = 180.0 / M_PI;
                double err_yaw_cl = std::atan((primary.center.x - cam_model.cx) / cam_model.fx) * kR2D_cl;
                double err_pitch_cl = std::atan((primary.center.y - cam_model.cy) / cam_model.fy) * kR2D_cl;
                // 减去 boresight+视差 得到残差 (理想=0)
                double comp_yaw_cl = ctrl_cfg.boresight_yaw_deg;
                double comp_pitch_cl = ctrl_cfg.boresight_pitch_deg;
                float ba_cl = primary.bbox.width * primary.bbox.height;
                if (ba_cl > 0 && ctrl_cfg.target_height_m > 0.01) {
                    double bh_cl = std::sqrt(static_cast<double>(ba_cl));
                    double z_cl = cam_model.fy * ctrl_cfg.target_height_m / bh_cl;
                    if (z_cl > 0.5 && z_cl < 200.0) {
                        comp_yaw_cl += std::atan(ctrl_cfg.laser_offset_x / z_cl) * kR2D_cl;
                        comp_pitch_cl += std::atan(ctrl_cfg.laser_offset_y / z_cl) * kR2D_cl;
                    }
                }
                err_yaw_cl -= comp_yaw_cl;
                err_pitch_cl -= comp_pitch_cl;
                double vel_yaw_cl = ctrl_cfg.yaw_sign * (primary.velocity_px_s.x / cam_model.fx) * kR2D_cl;
                double vel_pitch_cl = ctrl_cfg.pitch_sign * (primary.velocity_px_s.y / cam_model.fy) * kR2D_cl;

                ctrl_learn.session_tracking_frames++;
                if (current_primary_color == "purple") ctrl_learn.session_hit_frames++;

                // ── 在线 EMA 自适应学习: 每帧更新 ──
                const double aF = ControlLearner::kEmaFast;
                const double aS = ControlLearner::kEmaSlow;

                // 1) EMA 更新误差
                ctrl_learn.ema_err_yaw = aF * std::abs(err_yaw_cl) + (1.0 - aF) * ctrl_learn.ema_err_yaw;
                ctrl_learn.ema_err_pitch = aF * std::abs(err_pitch_cl) + (1.0 - aF) * ctrl_learn.ema_err_pitch;

                // 2) EMA 更新振荡率 (误差符号翻转 → 1, 否则 → 0)
                double osc_yaw_sample = (ctrl_learn.last_err_yaw * err_yaw_cl < 0) ? 1.0 : 0.0;
                double osc_pitch_sample = (ctrl_learn.last_err_pitch * err_pitch_cl < 0) ? 1.0 : 0.0;
                ctrl_learn.ema_osc_yaw = aF * osc_yaw_sample + (1.0 - aF) * ctrl_learn.ema_osc_yaw;
                ctrl_learn.ema_osc_pitch = aF * osc_pitch_sample + (1.0 - aF) * ctrl_learn.ema_osc_pitch;
                ctrl_learn.last_err_yaw = err_yaw_cl;
                ctrl_learn.last_err_pitch = err_pitch_cl;

                // 3) EMA 更新 FF 梯度 (err*vel 相关性, 仅目标在动时)
                if (std::abs(vel_yaw_cl) > 0.5) {
                    double grad_y = err_yaw_cl * vel_yaw_cl / (vel_yaw_cl * vel_yaw_cl + 1e-6);
                    ctrl_learn.ema_ff_grad_yaw = aS * grad_y + (1.0 - aS) * ctrl_learn.ema_ff_grad_yaw;
                }
                if (std::abs(vel_pitch_cl) > 0.5) {
                    double grad_p = err_pitch_cl * vel_pitch_cl / (vel_pitch_cl * vel_pitch_cl + 1e-6);
                    ctrl_learn.ema_ff_grad_pitch = aS * grad_p + (1.0 - aS) * ctrl_learn.ema_ff_grad_pitch;
                }

                // 4) 每 30 帧 (~1秒) 应用 FF 增益调整
                if (ctrl_learn.session_tracking_frames >= 60 &&
                    ctrl_learn.session_tracking_frames % 30 == 0) {
                    // FF yaw: 梯度方向调整, 步长和梯度成正比
                    double ff_step_y = 0.01 * std::tanh(ctrl_learn.ema_ff_grad_yaw * 5.0);
                    ctrl_learn.ff_gain_yaw = std::clamp(ctrl_learn.ff_gain_yaw + ff_step_y, 0.5, 2.5);

                    // FF pitch
                    double ff_step_p = 0.01 * std::tanh(ctrl_learn.ema_ff_grad_pitch * 5.0);
                    ctrl_learn.ff_gain_pitch = std::clamp(ctrl_learn.ff_gain_pitch + ff_step_p, 0.5, 2.5);

                    // 应用 FF 学习结果
                    ctrl_cfg.ff_gain_yaw = ctrl_learn.ff_gain_yaw;
                    ctrl_cfg.ff_gain_pitch = ctrl_learn.ff_gain_pitch;
                    controller.updateConfig(ctrl_cfg);

                    // 定期打印
                    if (ctrl_learn.session_tracking_frames % 450 == 0) {
                        double hr = ctrl_learn.session_tracking_frames > 0 ?
                            static_cast<double>(ctrl_learn.session_hit_frames) / ctrl_learn.session_tracking_frames : 0;
                        std::cout << "CTRL_LEARN track=" << ctrl_learn.session_tracking_frames
                                  << " hits=" << ctrl_learn.session_hit_frames
                                  << " ratio=" << cv::format("%.2f", hr)
                                  << " ff_yaw=" << cv::format("%.3f", ctrl_learn.ff_gain_yaw)
                                  << " ff_pitch=" << cv::format("%.3f", ctrl_learn.ff_gain_pitch)
                                  << " osc_y=" << cv::format("%.2f", ctrl_learn.ema_osc_yaw)
                                  << " osc_p=" << cv::format("%.2f", ctrl_learn.ema_osc_pitch)
                                  << " err_y=" << cv::format("%.3f", ctrl_learn.ema_err_yaw)
                                  << " err_p=" << cv::format("%.3f", ctrl_learn.ema_err_pitch) << "\n";
                    }
                }
            }

            // ── 在线自学习: 紫色=命中, 计算残差并积分修正 boresight ──
            // 门控策略:
            //   1. 需要足够跟踪帧数热身 (EMA 冷启动时=0, 会假稳定)
            //   2. 紫色=激光已命中, 只检查振荡 (不需要误差小, 误差小了才能紫色)
            bool ema_warmed = ctrl_learn.session_tracking_frames >= 30;
            bool not_oscillating = (ctrl_learn.ema_osc_yaw < 0.25 && ctrl_learn.ema_osc_pitch < 0.25);
            if (current_primary_color == "purple" && ema_warmed && not_oscillating) {
                purple_hit_count++;
                constexpr double kR2D = 180.0 / M_PI;
                // 目标相对光轴的角度 (deg)
                double target_yaw_deg = std::atan((primary.center.x - cam_model.cx) / cam_model.fx) * kR2D;
                double target_pitch_deg = std::atan((primary.center.y - cam_model.cy) / cam_model.fy) * kR2D;
                // 当前激光补偿量: boresight + 视差
                double laser_yaw_deg = ctrl_cfg.boresight_yaw_deg;
                double laser_pitch_deg = ctrl_cfg.boresight_pitch_deg;
                float bbox_area_learn = primary.bbox.width * primary.bbox.height;
                if (bbox_area_learn > 0 && ctrl_cfg.target_height_m > 0.01) {
                    double bh = std::sqrt(static_cast<double>(bbox_area_learn));
                    double z_est = cam_model.fy * ctrl_cfg.target_height_m / bh;
                    if (z_est > 0.5 && z_est < 200.0) {
                        laser_yaw_deg += std::atan(ctrl_cfg.laser_offset_x / z_est) * kR2D;
                        laser_pitch_deg += std::atan(ctrl_cfg.laser_offset_y / z_est) * kR2D;
                    }
                }
                // 残差: 目标角度 - 激光补偿角度
                // 正 → boresight 偏小, 需增大; 零 → 完美
                double residual_yaw = target_yaw_deg - laser_yaw_deg;
                double residual_pitch = target_pitch_deg - laser_pitch_deg;

                // 记录完整状态到 CSV (供离线分析/训练神经网络)
                if (learn_log.is_open()) {
                    learn_log << ts << "," << purple_hit_count << ","
                              << primary.center.x << "," << primary.center.y << ","
                              << cam_model.cx << "," << cam_model.cy << ","
                              << residual_yaw << "," << residual_pitch << ","
                              << bs_correction_yaw << "," << bs_correction_pitch << ","
                              << bbox_area_learn << ","
                              << primary.velocity.x << "," << primary.velocity.y << ","
                              << cmd.yaw << "," << cmd.pitch << ","
                              << gimbal_state.yaw << "," << gimbal_state.pitch << ","
                              << cmd.yaw_rate << "," << cmd.pitch_rate << ","
                              << 0 << "\n";
                    learn_log.flush();
                }

                // 积分修正 boresight: correction += alpha * residual
                // 绝对赋值 boresight = baseline + correction → 无漂移
                // 自适应学习率: 偏差大→大步, 偏差小→精调
                if (purple_hit_count >= bs_learn_min_hits) {
                    double corr_mag = std::max(std::abs(residual_yaw), std::abs(residual_pitch));
                    double bs_alpha = kBsAlphaMin + (kBsAlphaMax - kBsAlphaMin) *
                                      (1.0 - std::exp(-kBsAlphaDecay * corr_mag));
                    bs_correction_yaw += bs_alpha * residual_yaw;
                    bs_correction_pitch += bs_alpha * residual_pitch;
                    ctrl_cfg.boresight_yaw_deg = bs_baseline_yaw + bs_correction_yaw;
                    ctrl_cfg.boresight_pitch_deg = bs_baseline_pitch + bs_correction_pitch;
                    controller.updateConfig(ctrl_cfg);
                }

                // 每3次命中打印 + 保存到磁盘 (下次启动自动加载)
                if (purple_hit_count % 3 == 0) {
                    std::cout << "BS_LEARN hits=" << purple_hit_count
                              << " residual_deg=(" << residual_yaw << "," << residual_pitch << ")"
                              << " corr_deg=(" << bs_correction_yaw << "," << bs_correction_pitch << ")"
                              << " bs_deg=(" << ctrl_cfg.boresight_yaw_deg << "," << ctrl_cfg.boresight_pitch_deg << ")\n";
                    // 持久化到 YAML
                    std::filesystem::create_directories(bs_learn_path.parent_path());
                    cv::FileStorage sfs(bs_learn_path.string(), cv::FileStorage::WRITE);
                    if (sfs.isOpened()) {
                        sfs << "correction_yaw_deg" << bs_correction_yaw;
                        sfs << "correction_pitch_deg" << bs_correction_pitch;
                        sfs << "total_hits" << purple_hit_count;
                        sfs.release();
                    }
                }
            }

            // 调试输出
            static int debug_count = 0;
            if (++debug_count % 30 == 0) {
                int64_t pipeline_delay_ms = cmd.timestamp - ts;
                std::cout << "DBG gimbal cmd: mode=" << static_cast<int>(cmd.mode)
                          << " yaw=" << cmd.yaw << " pitch=" << cmd.pitch
                          << " p_rate=" << cmd.pitch_rate
                          << " delay=" << pipeline_delay_ms << "ms"
                          << " uv=(" << primary.center.x << "," << primary.center.y << ")"
                          << " bbox_h=" << primary.bbox.height << "\n";
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
                j["error_yaw"] = cmd.yaw - gimbal_state.yaw;
                j["error_pitch"] = cmd.pitch - gimbal_state.pitch;
                j["bbox_h"] = primary.bbox.height;
                plotter.plot(j);
            }

            // 更新 dead reckoning 位置
            // 仅在云台状态新鲜时更新 (防止 stale 下指令无限累积)
            // stale 时云台没在动, state 不应跟随 cmd
            if (!gimbal_state_stale && has_gimbal_state) {
                dead_reckon_yaw = cmd.yaw;
                dead_reckon_pitch = cmd.pitch;
            }

            uint8_t tx_frame[gimbal_serial::kTxFrameSize]{};
            if (has_gimbal_state) {
                gimbal_serial::packGimbalCommand(cmd, gimbal_state, tx_frame);
            } else {
                gimbal_serial::packGimbalCommand(cmd, tx_frame);
            }
            int64_t t_write_start = common::nowMs();
            int written = serial.write(tx_frame, static_cast<int>(sizeof(tx_frame)));
            int64_t t_write_end = common::nowMs();

            // 调试：显示串口写入 + 串口通信耗时
            static int serial_debug_count = 0;
            static int64_t serial_write_sum = 0;
            static int serial_write_count = 0;
            serial_write_sum += (t_write_end - t_write_start);
            serial_write_count++;
            if (++serial_debug_count % 30 == 0) {
                double avg_write_ms = (serial_write_count > 0)
                    ? static_cast<double>(serial_write_sum) / serial_write_count : 0;
                std::cout << "DBG serial wrote=" << written << " bytes"
                          << " tx_ms=" << (t_write_end - t_write_start)
                          << " avg_tx=" << cv::format("%.1f", avg_write_ms) << "ms\n";
                serial_write_sum = 0;
                serial_write_count = 0;
            }

            // 瞄准十字丝 (黄色)
            cv::drawMarker(frame, cv::Point(primary.center.x, primary.center.y),
                           cv::Scalar(0, 255, 255), cv::MARKER_TILTED_CROSS, 30, 3, cv::LINE_AA);
        } else if (enable_gimbal) {
            // 发送 idle
            common::TargetMeasurement meas;
            meas.valid = false;
            meas.timestamp = ts;
            auto cmd = controller.update(meas, cam_model, ctrl_state);
            cmd.timestamp = common::nowMs();
            if (!gimbal_state_stale && has_gimbal_state) {
                dead_reckon_yaw = cmd.yaw;
                dead_reckon_pitch = cmd.pitch;
            }
            uint8_t tx_frame[gimbal_serial::kTxFrameSize]{};
            gimbal_serial::packGimbalCommand(cmd, tx_frame);
            serial.write(tx_frame, static_cast<int>(sizeof(tx_frame)));
            last_primary_color = "unknown";
            has_pending_purple_log = false;
        }

        // ── HUD ──
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_start).count();
        double fps = elapsed > 0 ? frame_count * 1000.0 / elapsed : 0;
        if (elapsed >= 3000) { frame_count = 0; fps_start = now; }

        // ── 结构化状态行 (供 control_panel StatusTab 解析, 每 ~30 帧一条) ──
        static int status_emit_counter = 0;
        if (++status_emit_counter >= 30) {
            status_emit_counter = 0;
            int track_count = static_cast<int>(tracked.size());
            int primary_id = -1;
            std::string primary_color = "none";
            float primary_conf = 0.0f;
            if (!tracked.empty()) {
                primary_id = tracked.front().track_id;
                auto it = trail_color.find(primary_id);
                if (it != trail_color.end()) primary_color = it->second;
            }
            std::cout << "STATUS"
                      << " fps=" << std::fixed << std::setprecision(1) << fps
                      << " tracks=" << track_count
                      << " primary_id=" << primary_id
                      << " color=" << primary_color
                      << " conf=" << std::setprecision(2) << primary_conf
                      << " enemy=" << enemy_color
                      << "\n" << std::flush;
        }

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
        if (purple_log.is_open() && has_pending_purple_log) {
            pending_purple_log.fps = fps;
            purple_log << pending_purple_log.ts_ms << ','
                       << pending_purple_log.track_id << ','
                       << pending_purple_log.prev_color << ','
                       << pending_purple_log.current_color << ','
                       << pending_purple_log.confidence << ','
                       << pending_purple_log.red_ratio << ','
                       << pending_purple_log.blue_ratio << ','
                       << pending_purple_log.purple_ratio << ','
                       << pending_purple_log.prediction_horizon_s << ','
                       << pending_purple_log.predicted_x << ','
                       << pending_purple_log.predicted_y << ','
                       << pending_purple_log.prediction_error_x << ','
                       << pending_purple_log.prediction_error_y << ','
                       << pending_purple_log.bbox.x << ','
                       << pending_purple_log.bbox.y << ','
                       << pending_purple_log.bbox.width << ','
                       << pending_purple_log.bbox.height << ','
                       << pending_purple_log.center.x << ','
                       << pending_purple_log.center.y << ','
                       << pending_purple_log.velocity.x << ','
                       << pending_purple_log.velocity.y << ','
                       << pending_purple_log.cmd_yaw << ','
                       << pending_purple_log.cmd_pitch << ','
                       << pending_purple_log.state_yaw << ','
                       << pending_purple_log.state_pitch << ','
                       << pending_purple_log.tracked_count << ','
                       << pending_purple_log.fps << '\n';
            purple_log.flush();
            has_pending_purple_log = false;
        }

        cv::imshow("Track & Illuminate", frame);
        if (enable_record && writer.isOpened()) {
            writer.write(frame);
        }
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') break;
        if (key == 'c' || key == 'C') show_color_mask = !show_color_mask;
    }

    if (writer.isOpened()) writer.release();
    if (enable_gimbal) serial.close();
    if (use_galaxy) { galaxy.stopGrabbing(); galaxy.close(); }
    tracker.saveTrajectoryLog();
    // 退出时保存自学习成果 (防止 Ctrl+C 丢失)
    if (purple_hit_count >= bs_learn_min_hits) {
        std::filesystem::create_directories(bs_learn_path.parent_path());
        cv::FileStorage sfs(bs_learn_path.string(), cv::FileStorage::WRITE);
        if (sfs.isOpened()) {
            sfs << "correction_yaw_deg" << bs_correction_yaw;
            sfs << "correction_pitch_deg" << bs_correction_pitch;
            sfs << "total_hits" << purple_hit_count;
            sfs.release();
            std::cout << "BS_LEARN saved on exit: hits=" << purple_hit_count
                      << " corr_deg=(" << bs_correction_yaw << "," << bs_correction_pitch << ")\n";
        }
    }
    // 退出时保存控制参数学习成果
    ctrl_learn.lifetime_tracking_frames += ctrl_learn.session_tracking_frames;
    ctrl_learn.lifetime_hit_frames += ctrl_learn.session_hit_frames;
    if (ctrl_learn.session_tracking_frames > 100) {
        double session_hr = static_cast<double>(ctrl_learn.session_hit_frames) / ctrl_learn.session_tracking_frames;
        ctrl_learn.ema_hit_ratio = 0.2 * session_hr + 0.8 * ctrl_learn.ema_hit_ratio;
    }
    ctrl_learn.save(ctrl_learn_path.string());
    std::cout << "CTRL_LEARN saved: sessions=" << ctrl_learn.session_count
              << " ff_yaw=" << ctrl_learn.ff_gain_yaw
              << " ff_pitch=" << ctrl_learn.ff_gain_pitch
              << " hit_ratio=" << ctrl_learn.ema_hit_ratio << "\n";

    cv::destroyAllWindows();
    std::cout << "=== 测试结束 ===\n";
    return 0;
}
