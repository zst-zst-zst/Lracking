// ============================================================================
// 测试6: 不发光时偏移推断测试
// ============================================================================
// 功能:
//   基于测试5, 模拟第3次锁定后激光模块停止发光的场景:
//   1. 先正常检测+跟踪, 积累模块相对无人机的偏移量
//   2. 按 'b' 模拟模块不发光 (屏蔽 Layer2 检测, 只保留 Layer1)
//   3. 此时系统用偏移推断预测模块位置
//   4. 可视化: 实际位置(绿) vs 预测位置(黄) 的偏差
//
// 用法: ./build/offset [--config ../src/detect/config/cascade.yaml]
//                      [--camera ../config/camera.yaml]
//                             [--video test.mp4]
//       按 b=切换模拟不发光, q/ESC=退出
// ============================================================================

#include "cascade_detector.h"
#include "target_tracker.h"
#include "match_analyzer.h"
#include "galaxy_camera/galaxy_camera.h"
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
}  // namespace

int main(int argc, char** argv) {
    std::string cascade_config = "config/cascade.yaml";
    std::string enemy_config = "config/enemy.yaml";
    std::string camera_config = "config/camera.yaml";
    std::string video_path;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) cascade_config = argv[++i];
        else if (arg == "--enemy" && i + 1 < argc) enemy_config = argv[++i];
        else if (arg == "--camera" && i + 1 < argc) camera_config = argv[++i];
        else if (arg == "--video" && i + 1 < argc) video_path = argv[++i];
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::cout << "=== 偏移推断测试 (模拟不发光) ===\n";
    std::cout << "按 [b] 切换模拟不发光模式\n\n";

    // ── 加载级联检测器 ──
    detect::CascadeDetector detector;
    if (!detector.loadConfig(cascade_config)) {
        std::cerr << "✗ 级联配置加载失败\n";
        return 1;
    }
    detector.loadConfig(enemy_config);
    std::cout << "✓ 级联检测器已加载\n";

    // ── 跟踪器 + 分析器 ──
    solve::TargetTracker tracker;
    tracker.loadConfig(cascade_config);

    solve::MatchAnalyzer analyzer;
    analyzer.loadConfig(cascade_config);
    analyzer.loadConfig(enemy_config);

    // ── 视频源 ──
    cv::VideoCapture cap;
    galaxy_camera::GalaxyCamera galaxy;
    bool use_galaxy = video_path.empty();

    if (use_galaxy) {
        galaxy_camera::CameraConfig gcfg;
        galaxy_camera::loadCameraConfig(camera_config, &gcfg);
        if (!galaxy.open(gcfg) || !galaxy.startGrabbing()) {
            std::cerr << "✗ Galaxy 失败, 尝试 OpenCV(0)\n";
            use_galaxy = false;
            cap.open(0);
        }
    } else {
        cap.open(video_path);
    }
    if (!use_galaxy && !cap.isOpened()) {
        std::cerr << "✗ 视频源打开失败\n";
        return 1;
    }

    cv::namedWindow("Offset Predict", cv::WINDOW_NORMAL);
    bool simulate_dark = false;  // 模拟不发光
    int frame_count = 0;
    auto fps_start = std::chrono::steady_clock::now();

    // 偏移量统计
    int offset_samples = 0;
    cv::Point2f ema_offset(0, 0);
    float ema_alpha = 0.1f;

    // 预测误差记录
    std::deque<float> predict_errors;
    const int kMaxErrors = 200;

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
        auto result = detector.detectCascade(frame, ts);

        // ── 正常模式: 检测 + 跟踪 + 积累偏移 ──
        // ── 模拟不发光: 屏蔽 laser_rx 检测, 只用无人机bbox + 偏移预测 ──

        cv::Point2f actual_laser_center(-1, -1);
        bool has_actual = false;

        if (!result.laser_rxs.empty()) {
            actual_laser_center = result.laser_rxs.front().center;
            has_actual = true;
        }

        // 跟踪器输入
        std::vector<std::pair<cv::Rect2f, float>> tracker_input;
        if (!simulate_dark) {
            for (const auto& lrx : result.laser_rxs) {
                tracker_input.emplace_back(
                    cv::Rect2f(lrx.bbox.x, lrx.bbox.y, lrx.bbox.width, lrx.bbox.height),
                    lrx.confidence);
            }
        }
        // 模拟不发光时: tracker_input 为空, 跟踪器只做 Kalman 预测
        auto tracked = tracker.update(tracker_input, ts);

        // ── 积累偏移量 (正常模式下) ──
        if (!simulate_dark && has_actual && !result.planes.empty()) {
            auto& plane = result.planes.front();
            cv::Point2f plane_center = plane.center;
            float bw = static_cast<float>(plane.bbox.width);
            float bh = static_cast<float>(plane.bbox.height);

            if (bw > 1 && bh > 1) {
                // 归一化偏移 = (模块中心 - 无人机中心) / 无人机尺寸
                cv::Point2f norm_offset(
                    (actual_laser_center.x - plane_center.x) / bw,
                    (actual_laser_center.y - plane_center.y) / bh
                );

                if (offset_samples == 0) {
                    ema_offset = norm_offset;
                } else {
                    ema_offset.x = ema_alpha * norm_offset.x + (1.0f - ema_alpha) * ema_offset.x;
                    ema_offset.y = ema_alpha * norm_offset.y + (1.0f - ema_alpha) * ema_offset.y;
                }
                offset_samples++;
            }
        }

        // ── 偏移预测 (模拟不发光时) ──
        cv::Point2f predicted_center(-1, -1);
        bool has_predict = false;

        if (simulate_dark && offset_samples >= 10 && !result.planes.empty()) {
            auto& plane = result.planes.front();
            float bw = static_cast<float>(plane.bbox.width);
            float bh = static_cast<float>(plane.bbox.height);
            predicted_center.x = plane.center.x + ema_offset.x * bw;
            predicted_center.y = plane.center.y + ema_offset.y * bh;
            has_predict = true;

            // 如果实际也能检测到, 计算误差 (验证用)
            if (has_actual) {
                float err = std::sqrt(
                    std::pow(predicted_center.x - actual_laser_center.x, 2) +
                    std::pow(predicted_center.y - actual_laser_center.y, 2));
                predict_errors.push_back(err);
                while (static_cast<int>(predict_errors.size()) > kMaxErrors) {
                    predict_errors.pop_front();
                }
            }
        }

        // ── 绘制 ──

        // 无人机 (绿色)
        for (const auto& p : result.planes) {
            cv::rectangle(frame, p.bbox, cv::Scalar(0, 255, 0), 2);
        }

        // 实际检测 (红色, 仅非模拟时显示; 模拟时灰色虚线表示"真值")
        if (has_actual) {
            cv::Scalar det_color = simulate_dark ? cv::Scalar(100, 100, 100) : cv::Scalar(0, 0, 255);
            int thick = simulate_dark ? 1 : 2;
            for (const auto& lrx : result.laser_rxs) {
                cv::rectangle(frame, lrx.bbox, det_color, thick);
            }
            if (simulate_dark) {
                // 模拟不发光但实际检测到: 显示真值十字 (灰色)
                cv::drawMarker(frame, cv::Point(actual_laser_center.x, actual_laser_center.y),
                               cv::Scalar(100, 100, 100), cv::MARKER_CROSS, 15, 1);
            }
        }

        // 偏移预测位置 (黄色大十字)
        if (has_predict) {
            cv::drawMarker(frame, cv::Point(predicted_center.x, predicted_center.y),
                           cv::Scalar(0, 255, 255), cv::MARKER_TILTED_CROSS, 25, 3, cv::LINE_AA);

            // 预测→真值连线 (紫色, 显示误差)
            if (has_actual) {
                cv::line(frame,
                         cv::Point(predicted_center.x, predicted_center.y),
                         cv::Point(actual_laser_center.x, actual_laser_center.y),
                         cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
            }
        }

        // 跟踪目标 (蓝色)
        for (const auto& tt : tracked) {
            if (tt.state == solve::TrackState::CONFIRMED) {
                cv::drawMarker(frame, cv::Point(tt.center.x, tt.center.y),
                               cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 15, 2);
            }
        }

        // ── HUD ──
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_start).count();
        double fps = elapsed > 0 ? frame_count * 1000.0 / elapsed : 0;
        if (elapsed >= 3000) { frame_count = 0; fps_start = now; }

        int y = 20;
        auto hud = [&](const std::string& text, cv::Scalar c) {
            cv::putText(frame, text, cv::Point(10, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv::LINE_AA);
            y += 18;
        };

        hud(cv::format("FPS: %.1f", fps), cv::Scalar(0, 255, 255));
        hud(simulate_dark
                ? ">>> 模拟不发光 (按[b]恢复) <<<"
                : "正常模式 (按[b]模拟不发光)",
            simulate_dark ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));
        hud(cv::format("偏移样本: %d  offset=(%.3f, %.3f)",
                        offset_samples, ema_offset.x, ema_offset.y),
            cv::Scalar(200, 200, 0));

        if (!predict_errors.empty()) {
            float avg_err = 0;
            for (float e : predict_errors) avg_err += e;
            avg_err /= predict_errors.size();
            float max_err = *std::max_element(predict_errors.begin(), predict_errors.end());
            hud(cv::format("预测误差: avg=%.1fpx  max=%.1fpx  (%d帧)",
                           avg_err, max_err, static_cast<int>(predict_errors.size())),
                avg_err < 10 ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 128, 255));
        }

        hud("颜色: 绿=无人机  红=检测  黄=偏移预测  灰=真值  紫=误差线",
            cv::Scalar(180, 180, 180));

        cv::imshow("Offset Predict", frame);
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') break;
        if (key == 'b' || key == 'B') {
            simulate_dark = !simulate_dark;
            std::cout << (simulate_dark ? ">>> 模拟不发光 <<<" : ">>> 恢复正常 <<<") << "\n";
            if (!simulate_dark) predict_errors.clear();
        }
    }

    if (use_galaxy) { galaxy.stopGrabbing(); galaxy.close(); }
    tracker.saveTrajectoryLog();
    cv::destroyAllWindows();

    // ── 最终报告 ──
    std::cout << "\n=== 偏移推断测试结果 ===\n";
    std::cout << "积累偏移样本: " << offset_samples << "\n";
    std::cout << "归一化偏移: (" << ema_offset.x << ", " << ema_offset.y << ")\n";
    if (!predict_errors.empty()) {
        float avg = 0, mx = 0;
        for (float e : predict_errors) { avg += e; mx = std::max(mx, e); }
        avg /= predict_errors.size();
        std::cout << "预测误差: 平均=" << avg << "px  最大=" << mx << "px\n";
    }
    return 0;
}
