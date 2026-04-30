// ============================================================================
// 测试2: 只识别激光接收装置 (近距离台面测试)
// ============================================================================
// 功能: 打开相机, 用 Layer 2 YOLO 直接在全帧上检测激光接收装置
//       不走级联流程, 不需要先检测无人机, 适合近距离桌面测试
// 用法: ./build/laser_rx [--config ../config/camera.yaml]
//                        [--model ../src/detect/model/export/layer2_laser_rx_fp16.engine]
//                            [--video test.mp4]
//       按 q/ESC 退出
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

#include "api.h"

namespace {
std::atomic<bool> g_exit{false};
void onSignal(int) { g_exit.store(true); }
}  // namespace

int main(int argc, char** argv) {
    std::string camera_config = "config/camera.yaml";
    std::string model_path = "src/detect/model/export/layer2_laser_rx_fp16.engine";
    std::string video_path;
    int input_size = 256;
    float conf_thresh = 0.40f;
    float iou_thresh = 0.45f;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) camera_config = argv[++i];
        else if (arg == "--model" && i + 1 < argc) model_path = argv[++i];
        else if (arg == "--video" && i + 1 < argc) video_path = argv[++i];
        else if (arg == "--size" && i + 1 < argc) input_size = std::stoi(argv[++i]);
        else if (arg == "--conf" && i + 1 < argc) conf_thresh = std::stof(argv[++i]);
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::cout << "=== 激光接收装置识别测试 (Layer 2 Only) ===\n";
    std::cout << "模型: " << model_path << "  输入: " << input_size << "x" << input_size << "\n";
    std::cout << "置信度阈值: " << conf_thresh << "\n\n";

    // ── 加载 TRT 引擎 ──
    TRTInferX::Api api;
    TRTInferX::EngineConfig ecfg;
    ecfg.engine_path = model_path;
    ecfg.target_w = input_size;
    ecfg.target_h = input_size;
    ecfg.max_batch = 1;
    ecfg.num_classes = 3;  // blue, purple, red
    ecfg.prep = TRTInferX::PreprocessMode::LETTERBOX;
    ecfg.out_mode = TRTInferX::OutputMode::RAW_ONLY;

    if (!api.load(ecfg)) {
        std::cerr << "✗ 模型加载失败: " << model_path << "\n";
        return 1;
    }
    std::cout << "✓ 模型已加载\n";

    TRTInferX::InferOptions opt;
    opt.conf = conf_thresh;
    opt.iou = iou_thresh;

    // ── 打开视频源 ──
    cv::VideoCapture cap;
    galaxy_camera::GalaxyCamera galaxy;
    bool use_galaxy = video_path.empty();

    if (use_galaxy) {
        galaxy_camera::CameraConfig gcfg;
        if (!camera_config.empty()) {
            galaxy_camera::loadCameraConfig(camera_config, &gcfg);
        }
        if (!galaxy.open(gcfg) || !galaxy.startGrabbing()) {
            std::cerr << "✗ Galaxy 相机打开失败, 尝试 OpenCV VideoCapture(0)\n";
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

    cv::namedWindow("Laser RX Only", cv::WINDOW_NORMAL);
    int frame_count = 0;
    auto fps_start = std::chrono::steady_clock::now();

    std::cout << "正在检测... (q/ESC 退出)\n";

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

        // ── 推理 ──
        TRTInferX::ImageInput img;
        img.data = frame.data;
        img.width = frame.cols;
        img.height = frame.rows;
        img.stride_bytes = static_cast<int>(frame.step[0]);
        img.color = TRTInferX::ColorSpace::BGR;
        img.layout = TRTInferX::Layout::HWC;
        img.dtype = TRTInferX::DType::UINT8;
        img.prep = TRTInferX::PreprocessMode::LETTERBOX;
        img.target_w = input_size;
        img.target_h = input_size;
        img.timestamp_ms = common::nowMs();

        auto results = api.infer({img}, opt);
        static const std::vector<TRTInferX::Det> empty_dets;
        const auto& dets = results.empty() ? empty_dets : results[0];

        // DEBUG: 每30帧打印检测数量
        static int dbg_cnt = 0;
        if (++dbg_cnt % 30 == 0 || !dets.empty())
            std::cout << "DBG dets=" << dets.size() << "\n";

        // ── 绘制 ──
        for (const auto& d : dets) {
            // class 0=blue, 1=purple, 2=red
            static const cv::Scalar cls_colors[] = {
                cv::Scalar(255, 0, 0),     // blue
                cv::Scalar(200, 0, 200),   // purple
                cv::Scalar(0, 0, 255),     // red
            };
            cv::Scalar color = cls_colors[std::min(d.cls, 2)];
            int thick = 2;
            cv::Rect bbox(static_cast<int>(d.x1), static_cast<int>(d.y1),
                          static_cast<int>(d.x2 - d.x1), static_cast<int>(d.y2 - d.y1));
            cv::rectangle(frame, bbox, color, thick);

            cv::Point2f center((d.x1 + d.x2) / 2.0f, (d.y1 + d.y2) / 2.0f);
            cv::drawMarker(frame, cv::Point(center.x, center.y), color,
                           cv::MARKER_CROSS, 15, 2);

            static const char* cls_names[] = {"blue", "purple", "red"};
            const char* cls_name = cls_names[std::min(d.cls, 2)];
            cv::putText(frame,
                        cv::format("%s %.2f", cls_name, d.score),
                        cv::Point(bbox.x, std::max(20, bbox.y - 5)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }

        // ── FPS ──
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_start).count();
        if (elapsed > 0) {
            double fps = frame_count * 1000.0 / elapsed;
            cv::putText(frame, cv::format("FPS: %.1f  Dets: %d", fps, static_cast<int>(dets.size())),
                        cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 255), 2);
        }
        if (elapsed >= 3000) {
            frame_count = 0;
            fps_start = now;
        }

        cv::imshow("Laser RX Only", frame);
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') break;
    }

    if (use_galaxy) {
        galaxy.stopGrabbing();
        galaxy.close();
    }
    cv::destroyAllWindows();
    std::cout << "=== 测试结束 ===\n";
    return 0;
}
