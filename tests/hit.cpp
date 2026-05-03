// ============================================================================
// 测试7: 追踪打中测试 (全流程)
// ============================================================================
// 功能:
//   完整流程: 级联检测 → 跟踪 → 偏移推断 → 云台控制 → 评估命中
//   包含不发光时的偏移推断, 评估激光是否能持续命中目标
//   记录命中率统计, 用于验证整体系统性能
//
// 用法: ./build/hit [--config ../src/detect/config/cascade.yaml]
//                   [--camera ../config/camera.yaml]
//                   [--control ../src/control/config/control.yaml]
//                           [--port /dev/ttyUSB0] [--video test.mp4]
//       按 b=模拟不发光, r=重置统计, q/ESC=退出
// ============================================================================

#include "cascade_detector.h"
#include "target_tracker.h"
#include "match_analyzer.h"
#include "galaxy_camera/galaxy_camera.h"
#include "gimbal_serial/protocol.h"
#include "gimbal_serial/serial_port.h"
#include "control/config.h"
#include "control/controller.h"
#include "common/time_utils.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <deque>
#include <iostream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {
std::atomic<bool> g_exit{false};
void onSignal(int) { g_exit.store(true); }

struct HitStats {
    int total_frames = 0;        // 总帧数 (有目标时)
    int on_target_frames = 0;    // 瞄准质量OK的帧数
    int continuous_hit = 0;      // 当前连续命中帧数
    int max_continuous_hit = 0;  // 最大连续命中帧数
    int continuous_miss = 0;     // 当前连续丢失帧数
    float total_error_px = 0;    // 累计误差 (像素)

    void reset() { *this = HitStats{}; }

    float hitRate() const {
        return total_frames > 0
            ? static_cast<float>(on_target_frames) / total_frames * 100.0f : 0;
    }
    float avgError() const {
        return total_frames > 0 ? total_error_px / total_frames : 0;
    }
};

}  // namespace

int main(int argc, char** argv) {
    std::string cascade_config = "config/cascade.yaml";
    std::string enemy_config = "config/enemy.yaml";
    std::string camera_config = "config/camera.yaml";
    std::string control_config = "config/control.yaml";
    std::string serial_port = "/dev/ttyUSB0";
    int serial_baud = 115200;
    std::string video_path;
    bool enable_gimbal = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) cascade_config = argv[++i];
        else if (arg == "--enemy" && i + 1 < argc) enemy_config = argv[++i];
        else if (arg == "--camera" && i + 1 < argc) camera_config = argv[++i];
        else if (arg == "--control" && i + 1 < argc) control_config = argv[++i];
        else if (arg == "--port" && i + 1 < argc) serial_port = argv[++i];
        else if (arg == "--baud" && i + 1 < argc) serial_baud = std::stoi(argv[++i]);
        else if (arg == "--video" && i + 1 < argc) video_path = argv[++i];
        else if (arg == "--no-gimbal") enable_gimbal = false;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::cout << "=== 追踪打中测试 (全流程) ===\n";

    // ── 级联检测器 ──
    detect::CascadeDetector detector;
    if (!detector.loadConfig(cascade_config)) {
        std::cerr << "✗ 级联配置加载失败\n";
        return 1;
    }
    detector.loadConfig(enemy_config);

    // ── 跟踪器 + 分析器 ──
    solve::TargetTracker tracker;
    tracker.loadConfig(cascade_config);

    solve::MatchAnalyzer analyzer;
    analyzer.loadConfig(cascade_config);
    analyzer.loadConfig(enemy_config);

    // ── 串口 + 控制器 ──
    gimbal_serial::SerialPort serial;
    gimbal_serial::FrameParser parser;
    gimbal_serial::GimbalState gimbal_state;
    bool has_gimbal_state = false;
    control::ControlConfig ctrl_cfg;
    common::CameraModel cam_model;

    if (enable_gimbal) {
        if (serial.open(serial_port, serial_baud)) {
            std::cout << "✓ 串口: " << serial_port << "\n";
        } else {
            std::cerr << "✗ 串口失败, 云台禁用\n";
            enable_gimbal = false;
        }
        control::loadControlConfig(control_config, &ctrl_cfg, &cam_model, camera_config);
    }
    control::Controller controller(ctrl_cfg);

    // ── 视频源 ──
    cv::VideoCapture cap;
    galaxy_camera::GalaxyCamera galaxy;
    bool use_galaxy = video_path.empty();

    if (use_galaxy) {
        galaxy_camera::CameraConfig gcfg;
        galaxy_camera::loadCameraConfig(camera_config, &gcfg);
        if (!galaxy.open(gcfg) || !galaxy.startGrabbing()) {
            use_galaxy = false;
            cap.open(0);
        }
    } else {
        cap.open(video_path);
    }
    if (!use_galaxy && !cap.isOpened()) {
        std::cerr << "✗ 视频源失败\n";
        return 1;
    }

    cv::namedWindow("Tracking Hit Test", cv::WINDOW_NORMAL);
    bool simulate_dark = false;
    int frame_count = 0;
    auto fps_start = std::chrono::steady_clock::now();
    std::vector<uint8_t> rx_buf(2048);
    HitStats stats;

    // 命中率历史 (绘图用)
    std::deque<float> hit_rate_history;
    const int kHistoryLen = 300;

    std::cout << "按 [b]=模拟不发光  [r]=重置统计  [q/ESC]=退出\n\n";

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
            int n = serial.read(rx_buf.data(), static_cast<int>(rx_buf.size()), 1);
            if (n > 0 && parser.push(rx_buf.data(), static_cast<size_t>(n), &gimbal_state)) {
                has_gimbal_state = true;
            }
        }

        // ── 级联检测 ──
        auto result = detector.detectCascade(frame, ts);

        // ── 跟踪 (模拟不发光时屏蔽 laser_rx) ──
        std::vector<std::pair<cv::Rect2f, float>> tracker_input;
        if (!simulate_dark) {
            for (const auto& lrx : result.laser_rxs) {
                tracker_input.emplace_back(
                    cv::Rect2f(lrx.bbox.x, lrx.bbox.y, lrx.bbox.width, lrx.bbox.height),
                    lrx.confidence);
            }
        }
        auto tracked = tracker.update(tracker_input, ts);

        // ── 分析器更新 ──
        bool drone_det = !result.planes.empty();
        cv::Point2f drone_c(0, 0);
        float drone_bh = 0;
        if (drone_det) {
            drone_c = result.planes.front().center;
            drone_bh = static_cast<float>(result.planes.front().bbox.height);
        }
        bool has_tracked = tracker.hasPrimaryTarget();
        auto primary = has_tracked ? tracker.primaryTarget() : solve::TargetTracker::TrackedTarget{};
        analyzer.update(ts, drone_det, drone_c, drone_bh,
                        has_tracked && !simulate_dark,
                        primary.center, primary.confidence,
                        frame.cols, frame.rows);

        // ── 命中判定 ──
        // 使用光心作为激光出射点近似, 计算目标中心到光心的距离
        // 如果距离在目标尺寸的一定范围内, 认为命中
        cv::Point2f aim_point(static_cast<float>(cam_model.cx),
                              static_cast<float>(cam_model.cy));
        bool hit_this_frame = false;
        float error_px = 0;

        if (has_tracked) {
            error_px = std::sqrt(
                std::pow(primary.center.x - aim_point.x, 2) +
                std::pow(primary.center.y - aim_point.y, 2));

            // 命中半径: 目标bbox对角线的一半 (近似模块检测区域)
            float hit_radius = std::sqrt(primary.bbox.width * primary.bbox.width +
                                         primary.bbox.height * primary.bbox.height) / 2.0f;
            hit_radius = std::max(hit_radius, 15.0f);  // 最小 15px

            hit_this_frame = (error_px <= hit_radius);

            stats.total_frames++;
            stats.total_error_px += error_px;
            if (hit_this_frame) {
                stats.on_target_frames++;
                stats.continuous_hit++;
                stats.continuous_miss = 0;
                stats.max_continuous_hit = std::max(stats.max_continuous_hit, stats.continuous_hit);
            } else {
                stats.continuous_miss++;
                stats.continuous_hit = 0;
            }
        }

        // 记录命中率历史
        if (stats.total_frames > 0 && stats.total_frames % 5 == 0) {
            hit_rate_history.push_back(stats.hitRate());
            while (static_cast<int>(hit_rate_history.size()) > kHistoryLen) {
                hit_rate_history.pop_front();
            }
        }

        // ── 云台控制 ──
        if (enable_gimbal) {
            common::TargetMeasurement meas;
            meas.valid = has_tracked;
            meas.timestamp = ts;
            if (has_tracked) {
                meas.uv = primary.center;
                meas.confidence = primary.confidence;
            }
            auto cmd = controller.update(meas, cam_model, gimbal_state);
            cmd.timestamp = common::nowMs();
            uint8_t tx_frame[gimbal_serial::kTxFrameSize]{};
            if (has_gimbal_state) {
                gimbal_serial::packGimbalCommand(cmd, gimbal_state, tx_frame);
            } else {
                gimbal_serial::packGimbalCommand(cmd, tx_frame);
            }
            serial.write(tx_frame, static_cast<int>(sizeof(tx_frame)));
        }

        // ── 绘制 ──

        // 无人机 (绿色)
        for (const auto& p : result.planes) {
            cv::rectangle(frame, p.bbox, cv::Scalar(0, 255, 0), 2);
        }

        // 检测 (红色, 模拟不发光时灰色)
        for (const auto& lrx : result.laser_rxs) {
            cv::Scalar c = simulate_dark ? cv::Scalar(80, 80, 80) : cv::Scalar(0, 0, 255);
            cv::rectangle(frame, lrx.bbox, c, simulate_dark ? 1 : 2);
        }

        // 跟踪目标 + 命中圆
        if (has_tracked) {
            cv::Point tc(primary.center.x, primary.center.y);
            cv::Scalar tc_color = hit_this_frame ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
            cv::drawMarker(frame, tc, tc_color, cv::MARKER_CROSS, 20, 2);

            float hit_r = std::sqrt(primary.bbox.width * primary.bbox.width +
                                    primary.bbox.height * primary.bbox.height) / 2.0f;
            hit_r = std::max(hit_r, 15.0f);
            cv::circle(frame, tc, static_cast<int>(hit_r), tc_color, 1, cv::LINE_AA);
        }

        // Boresight 十字丝 (黄色)
        cv::drawMarker(frame, cv::Point(aim_point.x, aim_point.y),
                       cv::Scalar(0, 255, 255), cv::MARKER_TILTED_CROSS, 30, 2, cv::LINE_AA);

        // 瞄准质量指示
        bool on_target = analyzer.isOnTarget();
        std::string aim_str = solve::aimStrategyStr(analyzer.currentStrategy());

        // ── HUD ──
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_start).count();
        double fps = elapsed > 0 ? frame_count * 1000.0 / elapsed : 0;
        if (elapsed >= 3000) { frame_count = 0; fps_start = now; }

        int y = 20;
        auto hud = [&](const std::string& t, cv::Scalar c) {
            cv::putText(frame, t, cv::Point(10, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv::LINE_AA);
            y += 18;
        };

        hud(cv::format("FPS: %.1f  Aim: %s  OnTarget: %s",
                        fps, aim_str.c_str(), on_target ? "YES" : "no"),
            on_target ? cv::Scalar(0, 255, 0) : cv::Scalar(100, 100, 100));

        hud(cv::format("Phase: %s  Tier: %d  P: %.1f  Locks: %d",
                        solve::flightPhaseStr(analyzer.currentPhase()),
                        analyzer.currentTier(),
                        analyzer.estimatedP(),
                        analyzer.lockCount()),
            cv::Scalar(200, 200, 0));

        hud(cv::format("命中率: %.1f%%  (%d/%d)  连续命中: %d  最大: %d",
                        stats.hitRate(), stats.on_target_frames, stats.total_frames,
                        stats.continuous_hit, stats.max_continuous_hit),
            stats.hitRate() > 70 ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 128, 255));

        hud(cv::format("平均误差: %.1fpx  当前: %.1fpx",
                        stats.avgError(), error_px),
            cv::Scalar(0, 200, 200));

        hud(simulate_dark ? ">>> 模拟不发光 [b] <<<" : "正常模式 [b]=模拟不发光",
            simulate_dark ? cv::Scalar(0, 0, 255) : cv::Scalar(180, 180, 180));

        // ── 命中率曲线 (右下角) ──
        if (hit_rate_history.size() >= 2) {
            int pw = 200, ph = 60;
            int px0 = frame.cols - pw - 10;
            int py0 = frame.rows - ph - 10;
            cv::rectangle(frame, cv::Rect(px0, py0, pw, ph), cv::Scalar(20, 20, 20), cv::FILLED);
            cv::rectangle(frame, cv::Rect(px0, py0, pw, ph), cv::Scalar(80, 80, 80), 1);
            int n_pts = static_cast<int>(hit_rate_history.size());
            for (int i = 1; i < n_pts; ++i) {
                int x1 = px0 + (i - 1) * pw / (n_pts - 1);
                int x2 = px0 + i * pw / (n_pts - 1);
                int y1 = py0 + ph - static_cast<int>(hit_rate_history[i - 1] / 100.0f * ph);
                int y2 = py0 + ph - static_cast<int>(hit_rate_history[i] / 100.0f * ph);
                cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2),
                         cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }
            cv::putText(frame, "Hit%", cv::Point(px0 + 2, py0 + 12),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(180, 180, 180), 1);
        }

        cv::imshow("Tracking Hit Test", frame);
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') break;
        if (key == 'b' || key == 'B') {
            simulate_dark = !simulate_dark;
            std::cout << (simulate_dark ? ">>> 模拟不发光" : ">>> 恢复正常") << "\n";
        }
        if (key == 'r' || key == 'R') {
            stats.reset();
            hit_rate_history.clear();
            std::cout << ">>> 统计已重置\n";
        }
    }

    if (enable_gimbal) serial.close();
    if (use_galaxy) { galaxy.stopGrabbing(); galaxy.close(); }
    analyzer.finalize();
    tracker.saveTrajectoryLog();
    cv::destroyAllWindows();

    // ── 最终报告 ──
    std::cout << "\n=== 追踪打中测试结果 ===\n";
    std::cout << "总帧数: " << stats.total_frames << "\n";
    std::cout << "命中帧: " << stats.on_target_frames << "\n";
    std::cout << "命中率: " << stats.hitRate() << "%\n";
    std::cout << "最大连续命中: " << stats.max_continuous_hit << " 帧\n";
    std::cout << "平均误差: " << stats.avgError() << " px\n";
    return 0;
}
