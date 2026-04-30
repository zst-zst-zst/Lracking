#include "cascade_detector.h"
#include "solve/match_analyzer.h"
#include "solve/target_tracker.h"
#include "plotter.h"

#include <chrono>
#include <ctime>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <nlohmann/json.hpp>

namespace {
int64_t nowMs() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}

void drawCrosshair(cv::Mat& img, cv::Point p, cv::Scalar color, int size, int thick) {
    cv::line(img, {p.x - size, p.y}, {p.x + size, p.y}, {0,0,0}, thick + 2, cv::LINE_AA);
    cv::line(img, {p.x, p.y - size}, {p.x, p.y + size}, {0,0,0}, thick + 2, cv::LINE_AA);
    cv::line(img, {p.x - size, p.y}, {p.x + size, p.y}, color, thick, cv::LINE_AA);
    cv::line(img, {p.x, p.y - size}, {p.x, p.y + size}, color, thick, cv::LINE_AA);
}

void drawTrail(cv::Mat& img, const std::deque<cv::Point2f>& trail,
               cv::Scalar color, int max_len = 120) {
    int n = static_cast<int>(trail.size());
    int start = std::max(0, n - max_len);
    for (int i = start + 1; i < n; ++i) {
        float alpha = static_cast<float>(i - start) / (n - start);
        int a = static_cast<int>(40 + alpha * 215);
        cv::Scalar c(color[0] * a / 255, color[1] * a / 255, color[2] * a / 255);
        cv::line(img,
                 {static_cast<int>(trail[i-1].x), static_cast<int>(trail[i-1].y)},
                 {static_cast<int>(trail[i].x),   static_cast<int>(trail[i].y)},
                 c, 2, cv::LINE_AA);
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string config_path = "config/cascade.yaml";
    std::string enemy_path = "config/enemy.yaml";
    std::string control_path = "config/control.yaml";
    std::string record_dir;  // 手动指定录制目录 (覆盖自动)
    std::string video_path;
    int cam_id = 0;
    bool show = true;
    bool enable_plot = true;
    bool enable_record = true;
    bool is_match_mode = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--enemy" && i + 1 < argc) {
            enemy_path = argv[++i];
        } else if (arg == "--match") {
            is_match_mode = true;
        } else if (arg == "--control" && i + 1 < argc) {
            control_path = argv[++i];
        } else if (arg == "--video" && i + 1 < argc) {
            video_path = argv[++i];
        } else if (arg == "--record-dir" && i + 1 < argc) {
            record_dir = argv[++i];
        } else if (arg == "--cam" && i + 1 < argc) {
            cam_id = std::stoi(argv[++i]);
        } else if (arg == "--no-show") {
            show = false;
        } else if (arg == "--no-plot") {
            enable_plot = false;
        } else if (arg == "--no-record") {
            enable_record = false;
        }
    }

    // 录制模式: --match → 比赛(全分辨率), 否则 → 测试(半分辨率)
    int record_scale = is_match_mode ? 1 : 2;
    if (record_dir.empty()) {
        record_dir = is_match_mode ? "../../../records/match"
                                   : "../../../records/test";
    }
    std::cout << (is_match_mode ? "★ 比赛模式 ★" : "测试模式")
              << " 录制: " << (is_match_mode ? "全分辨率" : "半分辨率") << "\n";

    // 加载配置
    detect::CascadeDetector detector;
    if (!detector.loadConfig(config_path)) {
        std::cerr << "级联检测器配置加载失败: " << config_path << "\n";
        return 1;
    }
    // 加载敌方信息 (覆盖 cascade.yaml 中的敌方参数)
    if (!detector.loadConfig(enemy_path)) {
        std::cerr << "警告: 敌方配置加载失败: " << enemy_path << ", 使用默认值\n";
    }

    solve::MatchAnalyzer analyzer;
    if (!analyzer.loadConfig(config_path)) {
        std::cerr << "警告: 比赛分析器配置加载失败, 继续运行\n";
    }
    // 加载敌方信息 (覆盖 analyzer 中的敌方参数)
    if (!analyzer.loadConfig(enemy_path)) {
        std::cerr << "警告: 敌方配置加载失败: " << enemy_path << ", 使用默认值\n";
    }

    solve::TargetTracker tracker;
    if (!tracker.loadConfig(config_path)) {
        std::cerr << "警告: 跟踪器配置加载失败, 使用默认值\n";
    }

    tools::Plotter plotter;
    if (enable_plot) {
        std::cout << "Plotter: UDP → 127.0.0.1:9870 (PlotJuggler)\n";
    }

    // ── 加载 Boresight (u_L, v_L) ──
    double u_L = -1, v_L = -1;
    {
        cv::FileStorage fs(control_path, cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["u_L"] >> u_L;
            fs["v_L"] >> v_L;
            std::cout << "Boresight: u_L=" << u_L << " v_L=" << v_L << "\n";
        } else {
            std::cerr << "警告: 无法加载 boresight (" << control_path << "), 不绘制激光落点\n";
        }
    }
    bool has_boresight = (u_L > 0 && v_L > 0);

    cv::VideoCapture cap;
    if (!video_path.empty()) {
        cap.open(video_path);
    } else {
        cap.open(cam_id);
    }
    if (!cap.isOpened()) {
        std::cerr << "视频源打开失败\n";
        return 1;
    }

    // ── 自动录制 ──
    cv::VideoWriter writer;
    std::string record_path;
    if (enable_record) {
        // 创建录制目录
        std::filesystem::create_directories(record_dir);
        // 生成时间戳文件名: YYYY-MM-DD_HH-MM-SS.mp4
        auto now_t = std::chrono::system_clock::now();
        auto time_c = std::chrono::system_clock::to_time_t(now_t);
        std::tm ltm{};
        localtime_r(&time_c, &ltm);
        std::ostringstream fn;
        fn << std::put_time(&ltm, "%Y-%m-%d_%H-%M-%S") << ".mp4";
        record_path = (std::filesystem::path(record_dir) / fn.str()).string();

        int fw = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int fh = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        double fps = cap.get(cv::CAP_PROP_FPS);
        if (fps <= 0) fps = 30.0;
        int rw = fw / record_scale;
        int rh = fh / record_scale;
        writer.open(record_path, cv::VideoWriter::fourcc('m','p','4','v'),
                    fps, cv::Size(rw, rh));
        if (writer.isOpened()) {
            std::cout << "录制: " << record_path << " (" << rw << "x" << rh
                      << " @ " << fps << "fps)\n";
        } else {
            std::cerr << "视频录制打开失败: " << record_path << "\n";
        }
    }

    // ── 轨迹历史 ──
    const int kTrailLen = 180;  // 保留最近 N 帧
    std::deque<cv::Point2f> target_trail;  // 目标检测中心
    std::deque<cv::Point2f> laser_trail;   // 激光预估落点

    int frame_count = 0;
    auto fps_start = std::chrono::steady_clock::now();

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            break;
        }

        int64_t ts = nowMs();
        auto result = detector.detectCascade(frame, ts);

        frame_count++;

        // 将检测结果输入跟踪器
        std::vector<std::pair<cv::Rect2f, float>> tracker_dets;
        for (const auto& lrx : result.laser_rxs) {
            tracker_dets.emplace_back(
                cv::Rect2f(lrx.bbox.x, lrx.bbox.y, lrx.bbox.width, lrx.bbox.height),
                lrx.confidence);
        }
        // 直接检测失败时, 使用偏移预测结果 (第3次锁定后模块停止发光)
        if (tracker_dets.empty() && result.has_predicted_target) {
            tracker_dets.emplace_back(
                cv::Rect2f(result.predicted_center.x - 10, result.predicted_center.y - 10, 20, 20),
                result.predicted_confidence);
        }
        auto tracked = tracker.update(tracker_dets, ts);

        // 用跟踪器的主目标输入比赛分析器 (比原始检测更稳定)
        bool drone_det = !result.planes.empty();
        cv::Point2f drone_c(0, 0);
        float drone_bh = 0;
        if (drone_det) {
            drone_c = result.planes.front().center;
            drone_bh = static_cast<float>(result.planes.front().bbox.height);
        }

        bool has_tracked_target = tracker.hasPrimaryTarget();
        auto primary = tracker.primaryTarget();
        analyzer.update(ts, drone_det, drone_c, drone_bh,
                        has_tracked_target, primary.center,
                        primary.confidence,
                        frame.cols, frame.rows);

        if (show) {
            // 绘制无人机 bbox (绿色)
            for (const auto& plane : result.planes) {
                cv::rectangle(frame, plane.bbox, cv::Scalar(0, 255, 0), 2);
                cv::putText(frame,
                            "plane " + cv::format("%.2f", plane.confidence),
                            cv::Point(plane.bbox.x, std::max(25, plane.bbox.y - 8)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 255, 0), 2);
            }

            // 绘制 ROI 区域 (青色)
            for (const auto& lrx : result.laser_rxs) {
                cv::rectangle(frame, lrx.roi, cv::Scalar(255, 255, 0), 1);
            }

            // 绘制偏移预测位置 (紫色, 模块停止发光时)
            if (result.laser_rxs.empty() && result.has_predicted_target) {
                cv::Point pred_pt(static_cast<int>(result.predicted_center.x),
                                  static_cast<int>(result.predicted_center.y));
                cv::drawMarker(frame, pred_pt, cv::Scalar(255, 0, 255),
                               cv::MARKER_DIAMOND, 24, 2);
                cv::putText(frame,
                            cv::format("PRED %.2f", result.predicted_confidence),
                            cv::Point(pred_pt.x + 14, pred_pt.y),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
            }

            // 绘制原始激光模块检测 (红色, 细线)
            for (const auto& lrx : result.laser_rxs) {
                cv::rectangle(frame, lrx.bbox, cv::Scalar(0, 0, 255), 1);
            }

            // 绘制跟踪目标
            for (const auto& tt : tracked) {
                cv::Rect bbox(static_cast<int>(tt.bbox.x), static_cast<int>(tt.bbox.y),
                              static_cast<int>(tt.bbox.width), static_cast<int>(tt.bbox.height));
                cv::Point center(static_cast<int>(tt.center.x),
                                 static_cast<int>(tt.center.y));

                cv::Scalar color;
                int thickness = 2;
                if (tt.state == solve::TrackState::CONFIRMED) {
                    color = tt.detected ? cv::Scalar(0, 0, 255)      // 红色 = 已确认 + 检测到
                                        : cv::Scalar(0, 128, 255);  // 橙色 = 已确认 + Kalman预测
                    thickness = 2;
                } else {
                    color = cv::Scalar(128, 128, 128);  // 灰色 = 暂定
                    thickness = 1;
                }

                cv::rectangle(frame, bbox, color, thickness);
                cv::drawMarker(frame, center, color, cv::MARKER_CROSS, 20, 2);
                cv::putText(frame,
                            cv::format("T%d %.2f", tt.track_id, tt.confidence),
                            cv::Point(bbox.x, std::max(25, bbox.y - 8)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);

                // 绘制速度箭头
                float speed = std::sqrt(tt.velocity.x * tt.velocity.x +
                                        tt.velocity.y * tt.velocity.y);
                if (speed > 1.0f) {
                    cv::Point vel_end(center.x + static_cast<int>(tt.velocity.x * 3),
                                      center.y + static_cast<int>(tt.velocity.y * 3));
                    cv::arrowedLine(frame, center, vel_end, color, 2, cv::LINE_AA, 0, 0.3);
                }
            }

            // ── 轨迹拖尾: 目标(绿) + 激光落点预估(红) ──
            if (tracker.hasPrimaryTarget()) {
                // 记录当前帧
                target_trail.push_back(primary.center);
                if (static_cast<int>(target_trail.size()) > kTrailLen)
                    target_trail.pop_front();

                if (has_boresight) {
                    // 激光落点 = 目标位置 + boresight偏移
                    // (假设云台完美跟随, 目标居中, 激光落在 u_L,v_L)
                    float cx = frame.cols / 2.0f;
                    float cy = frame.rows / 2.0f;
                    float off_u = static_cast<float>(u_L) - cx;
                    float off_v = static_cast<float>(v_L) - cy;
                    cv::Point2f laser_pt(primary.center.x + off_u,
                                         primary.center.y + off_v);
                    laser_trail.push_back(laser_pt);
                    if (static_cast<int>(laser_trail.size()) > kTrailLen)
                        laser_trail.pop_front();
                }
            } else {
                // 丢失目标时清空轨迹
                target_trail.clear();
                laser_trail.clear();
            }

            // 绿色轨迹: 目标检测中心
            drawTrail(frame, target_trail, cv::Scalar(0, 255, 0), kTrailLen);

            // 红色轨迹: 激光预估落点
            if (has_boresight) {
                drawTrail(frame, laser_trail, cv::Scalar(0, 0, 255), kTrailLen);
            }

            // 偏移连线: 目标 → 激光落点 (黄色虚线)
            if (has_boresight && tracker.hasPrimaryTarget() && !laser_trail.empty()) {
                cv::Point tgt(static_cast<int>(primary.center.x),
                              static_cast<int>(primary.center.y));
                cv::Point lsr(static_cast<int>(laser_trail.back().x),
                              static_cast<int>(laser_trail.back().y));
                // 虚线效果
                for (int d = 0; d < 100; d += 6) {
                    float t0 = d / 100.0f;
                    float t1 = std::min(1.0f, (d + 3) / 100.0f);
                    cv::Point a(tgt.x + static_cast<int>((lsr.x - tgt.x) * t0),
                                tgt.y + static_cast<int>((lsr.y - tgt.y) * t0));
                    cv::Point b(tgt.x + static_cast<int>((lsr.x - tgt.x) * t1),
                                tgt.y + static_cast<int>((lsr.y - tgt.y) * t1));
                    cv::line(frame, a, b, cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
                }
                double dist = cv::norm(primary.center -
                                       cv::Point2f(laser_trail.back().x, laser_trail.back().y));
                cv::putText(frame, cv::format("%.1fpx", dist),
                            cv::Point((tgt.x + lsr.x) / 2, (tgt.y + lsr.y) / 2 - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4,
                            cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
            }

            // Boresight 固定十字 (红色)
            if (has_boresight) {
                drawCrosshair(frame,
                              cv::Point(static_cast<int>(u_L), static_cast<int>(v_L)),
                              cv::Scalar(0, 0, 255), 15, 2);
            }

            // 绘制平滑目标十字丝 (黄色=有效照射中, 品红=跟踪中)
            if (analyzer.hasSmoothedTarget()) {
                auto st = analyzer.smoothedTarget();
                cv::Point sc(static_cast<int>(st.x), static_cast<int>(st.y));
                cv::Scalar color = analyzer.isOnTarget()
                                       ? cv::Scalar(0, 255, 255)   // 黄色 = 有效照射中
                                       : cv::Scalar(255, 0, 255);  // 品红 = 跟踪中
                cv::drawMarker(frame, sc, color,
                               cv::MARKER_TILTED_CROSS, 30, 3, cv::LINE_AA);
            }

            // ── HUD: 比赛分析器状态 ──
            int y = 20;
            auto putLine = [&](const std::string& text, cv::Scalar color) {
                cv::putText(frame, text, cv::Point(10, y),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
                y += 20;
            };

            putLine(cv::format("Phase: %s",
                               solve::flightPhaseStr(analyzer.currentPhase())),
                    cv::Scalar(255, 255, 255));
            putLine(cv::format("Tier: %d  Aim: %s",
                               analyzer.currentTier(),
                               solve::aimStrategyStr(analyzer.currentStrategy())),
                    cv::Scalar(200, 200, 0));
            putLine(cv::format("Shake: %.1fpx  P: %.1f  Locks: %d/3",
                               analyzer.measuredShakePx(),
                               analyzer.estimatedP(),
                               analyzer.lockCount()),
                    cv::Scalar(0, 200, 200));
            putLine(cv::format("Continuous: %.2fs  OnTarget: %s",
                               analyzer.continuousIlluminateS(),
                               analyzer.isOnTarget() ? "YES" : "no"),
                    analyzer.isOnTarget()
                        ? cv::Scalar(0, 255, 0)
                        : cv::Scalar(100, 100, 100));

            // 跟踪器信息
            if (tracker.hasPrimaryTarget()) {
                putLine(cv::format("Track: T%d  vel=(%.1f,%.1f)  %s",
                                   primary.track_id,
                                   primary.velocity.x, primary.velocity.y,
                                   primary.detected ? "DET" : "PRED"),
                        cv::Scalar(200, 150, 255));
            } else {
                putLine("Track: none", cv::Scalar(100, 100, 100));
            }

            // ── FPS ──
            auto now_clk = std::chrono::steady_clock::now();
            double elapsed_s = std::chrono::duration<double>(now_clk - fps_start).count();
            double fps_val = elapsed_s > 0 ? frame_count / elapsed_s : 0;
            if (elapsed_s > 3.0) { frame_count = 0; fps_start = now_clk; }
            putLine(cv::format("FPS: %.1f", fps_val), cv::Scalar(255, 255, 255));

            // ── 图例 ──
            int legend_y = frame.rows - 40;
            cv::line(frame, {10, legend_y}, {30, legend_y}, {0, 255, 0}, 2);
            cv::putText(frame, "Target", {35, legend_y + 4},
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, {200, 200, 200}, 1);
            if (has_boresight) {
                cv::line(frame, {100, legend_y}, {120, legend_y}, {0, 0, 255}, 2);
                cv::putText(frame, "Laser", {125, legend_y + 4},
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, {200, 200, 200}, 1);
            }

            // ── 录制 ──
            if (writer.isOpened()) {
                cv::Mat small;
                cv::resize(frame, small,
                           cv::Size(frame.cols / record_scale,
                                    frame.rows / record_scale));
                writer.write(small);
            }

            cv::imshow("Cascade Detector", frame);
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') {
                break;
            }
        }

        // ── Plotter: 发送数据到 PlotJuggler ──
        if (enable_plot) {
            nlohmann::json j;
            j["timestamp"] = ts;
            j["phase"] = static_cast<int>(analyzer.currentPhase());
            j["tier"] = analyzer.currentTier();
            j["shake_px"] = analyzer.measuredShakePx();
            j["P_value"] = analyzer.estimatedP();
            j["lock_count"] = analyzer.lockCount();
            j["continuous_s"] = analyzer.continuousIlluminateS();
            j["on_target"] = analyzer.isOnTarget() ? 1 : 0;
            j["planes"] = static_cast<int>(result.planes.size());
            j["laser_rxs"] = static_cast<int>(result.laser_rxs.size());
            j["tracks"] = static_cast<int>(tracked.size());
            if (has_tracked_target) {
                j["target_u"] = primary.center.x;
                j["target_v"] = primary.center.y;
                j["target_vx"] = primary.velocity.x;
                j["target_vy"] = primary.velocity.y;
                j["target_conf"] = primary.confidence;
                if (has_boresight && !laser_trail.empty()) {
                    j["laser_u"] = laser_trail.back().x;
                    j["laser_v"] = laser_trail.back().y;
                    double offset = cv::norm(primary.center -
                                             cv::Point2f(laser_trail.back().x,
                                                         laser_trail.back().y));
                    j["laser_offset_px"] = offset;
                }
            }
            if (analyzer.hasSmoothedTarget()) {
                auto st = analyzer.smoothedTarget();
                j["smooth_u"] = st.x;
                j["smooth_v"] = st.y;
            }
            plotter.plot(j);
        }
    }

    if (writer.isOpened()) {
        writer.release();
        std::cout << "录制完成: " << record_path << "\n";
    }
    analyzer.finalize();
    tracker.saveTrajectoryLog();
    return 0;
}
