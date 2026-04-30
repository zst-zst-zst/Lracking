// ============================================================================
// 测试8a: 红蓝色颜色过滤算法独立测试
// ============================================================================
// 功能:
//   打开相机, 实时显示 HSV 颜色分割效果:
//   1. 原图 + 颜色掩码叠加
//   2. 轮廓形状分析 (紧凑=模块 vs 细长=灯带)
//   3. Trackbar 实时调参 (H/S/V 阈值, 长宽比阈值)
//   用于调试灯带拒绝参数, 不需要 YOLO 模型
//
// 用法: ./build/color [--config ../config/camera.yaml]
//                           [--color blue] [--video test.mp4]
//       按 q/ESC 退出, r/b 切换红蓝
// ============================================================================

#include "galaxy_camera/galaxy_camera.h"
#include "common/time_utils.h"

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

// Trackbar 参数
int g_min_sat = 80;
int g_min_val = 80;
int g_elongation_x10 = 25;  // 2.5 × 10
int g_min_area = 20;
}  // namespace

int main(int argc, char** argv) {
    std::string camera_config = "config/camera.yaml";
    std::string video_path;
    std::string color = "blue";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) camera_config = argv[++i];
        else if (arg == "--video" && i + 1 < argc) video_path = argv[++i];
        else if (arg == "--color" && i + 1 < argc) color = argv[++i];
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::cout << "=== 红蓝色颜色过滤测试 ===\n";
    std::cout << "当前颜色: " << color << " (按 r=红色, b=蓝色 切换)\n\n";

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

    cv::namedWindow("Color Filter", cv::WINDOW_NORMAL);
    cv::createTrackbar("MinSat", "Color Filter", &g_min_sat, 255);
    cv::createTrackbar("MinVal", "Color Filter", &g_min_val, 255);
    cv::createTrackbar("Elong*10", "Color Filter", &g_elongation_x10, 100);
    cv::createTrackbar("MinArea", "Color Filter", &g_min_area, 500);

    int frame_count = 0;
    auto fps_start = std::chrono::steady_clock::now();

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

        // ── HSV 颜色分割 ──
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask;
        if (color == "blue") {
            cv::inRange(hsv, cv::Scalar(90, g_min_sat, g_min_val),
                        cv::Scalar(130, 255, 255), mask);
        } else {
            cv::Mat m1, m2;
            cv::inRange(hsv, cv::Scalar(0, g_min_sat, g_min_val),
                        cv::Scalar(15, 255, 255), m1);
            cv::inRange(hsv, cv::Scalar(165, g_min_sat, g_min_val),
                        cv::Scalar(180, 255, 255), m2);
            mask = m1 | m2;
        }

        float elong_thresh = g_elongation_x10 / 10.0f;

        // ── 轮廓分析 ──
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int compact_count = 0;
        int elongated_count = 0;
        int total_color_px = cv::countNonZero(mask);

        for (const auto& c : contours) {
            double area = cv::contourArea(c);
            if (area < g_min_area) continue;

            cv::RotatedRect rr = cv::minAreaRect(c);
            float w = std::max(rr.size.width, rr.size.height);
            float h = std::min(rr.size.width, rr.size.height);
            float elongation = (h > 0) ? w / h : 0;

            // 圆形度
            double perimeter = cv::arcLength(c, true);
            double circularity = (perimeter > 0) ? 4.0 * CV_PI * area / (perimeter * perimeter) : 0;

            bool is_elongated = (elongation > elong_thresh);
            cv::Scalar contour_color;
            std::string label;

            if (is_elongated) {
                contour_color = cv::Scalar(0, 0, 255);  // 红色 = 灯带嫌疑
                label = "strip";
                elongated_count++;
            } else {
                contour_color = cv::Scalar(0, 255, 0);  // 绿色 = 紧凑 (模块)
                label = "compact";
                compact_count++;
            }

            // 绘制轮廓
            cv::drawContours(frame, contours,
                             static_cast<int>(&c - &contours[0]),
                             contour_color, 2);

            // 绘制最小外接矩形
            cv::Point2f pts[4];
            rr.points(pts);
            for (int j = 0; j < 4; ++j) {
                cv::line(frame, pts[j], pts[(j + 1) % 4], contour_color, 1, cv::LINE_AA);
            }

            // 标注
            cv::putText(frame,
                        cv::format("%s e=%.1f c=%.2f a=%.0f",
                                   label.c_str(), elongation, circularity, area),
                        cv::Point(rr.center.x - 40, rr.center.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35, contour_color, 1);
        }

        // ── 颜色掩码叠加 (半透明) ──
        cv::Mat overlay = frame.clone();
        cv::Scalar mask_color = (color == "blue") ? cv::Scalar(255, 100, 0) : cv::Scalar(0, 0, 255);
        overlay.setTo(mask_color, mask);
        cv::addWeighted(frame, 0.7, overlay, 0.3, 0, frame);

        // ── HUD ──
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_start).count();
        double fps = elapsed > 0 ? frame_count * 1000.0 / elapsed : 0;
        if (elapsed >= 3000) { frame_count = 0; fps_start = now; }

        float ratio = (frame.rows * frame.cols > 0)
            ? static_cast<float>(total_color_px) / (frame.rows * frame.cols) * 100.0f : 0;

        int y = 20;
        auto hud = [&](const std::string& t, cv::Scalar c) {
            cv::putText(frame, t, cv::Point(10, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv::LINE_AA);
            y += 18;
        };

        hud(cv::format("FPS: %.1f  Color: %s (r/b切换)", fps, color.c_str()),
            cv::Scalar(0, 255, 255));
        hud(cv::format("像素占比: %.2f%%  compact: %d  strip: %d",
                        ratio, compact_count, elongated_count),
            cv::Scalar(200, 200, 0));
        hud(cv::format("MinSat=%d  MinVal=%d  Elong=%.1f  MinArea=%d",
                        g_min_sat, g_min_val, elong_thresh, g_min_area),
            cv::Scalar(180, 180, 180));

        std::string verdict;
        cv::Scalar verdict_color;
        if (compact_count > 0) {
            verdict = "✓ 有紧凑轮廓 → 保留 (可能是激光模块)";
            verdict_color = cv::Scalar(0, 255, 0);
        } else if (elongated_count > 0) {
            verdict = "✗ 只有细长轮廓 → 拒绝 (灯带)";
            verdict_color = cv::Scalar(0, 0, 255);
        } else {
            verdict = "— 未检测到颜色轮廓";
            verdict_color = cv::Scalar(128, 128, 128);
        }
        hud(verdict, verdict_color);

        cv::imshow("Color Filter", frame);
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') break;
        if (key == 'r' || key == 'R') { color = "red"; std::cout << "切换: 红色\n"; }
        if (key == 'b' || key == 'B') { color = "blue"; std::cout << "切换: 蓝色\n"; }
    }

    if (use_galaxy) { galaxy.stopGrabbing(); galaxy.close(); }
    cv::destroyAllWindows();
    std::cout << "=== 测试结束 ===\n";
    return 0;
}
