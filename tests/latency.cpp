// ============================================================================
// 端到端延迟测试 — 测量完整延迟链并给出 system_delay_ms 建议值
// ============================================================================
//
//   延迟链示意:
//
//     相机取帧 ──→ YOLO推理 ──→ 跟踪器 ──→ 控制器 ──→ 串口发送 ──→ 电控接收
//     [grab]       [detect]     [track]    [ctrl]     [tx]          [serial_rtt/2]
//     ├─────────── PC 软件延迟 (sw_total) ──────────┤  ├── 串口延迟 ──┤
//     ├──────────────── 建议 system_delay_ms ───────────────────────┤
//
//   最终输出建议值 = sw_total + serial_rtt / 2
//   把这个值填到 config/control.yaml 的 system_delay_ms 即可
//
// 用法:
//   ./build/latency                             # 有相机有串口
//   ./build/latency --video test.mp4            # 用视频替代相机
//   ./build/latency --no-serial                 # 不接串口也能测
//   ./build/latency --no-show                   # 无窗口
//   ./build/latency --port /dev/ttyACM0         # 指定串口
//   按 Ctrl+C 或 q 退出，退出后打印最终报告
// ============================================================================

#include "cascade_detector.h"
#include "target_tracker.h"
#include "control/controller.h"
#include "control/config.h"
#include "gimbal_serial/serial_port.h"
#include "gimbal_serial/protocol.h"
#include "galaxy_camera/galaxy_camera.h"
#include "common/types.h"
#include "common/time_utils.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <deque>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {
std::atomic<bool> g_exit{false};
void onSignal(int) { g_exit.store(true); }

using Clock = std::chrono::high_resolution_clock;
using Us = std::chrono::microseconds;

struct LatencyRecord {
    double grab_us = 0;       // 取帧
    double detect_us = 0;     // YOLO 推理
    double track_us = 0;      // 跟踪器
    double ctrl_us = 0;       // 控制器计算
    double tx_us = 0;         // 串口发送
    double sw_total_us = 0;   // PC全链路 (grab→tx)
    double serial_rtt_us = 0; // 串口往返 (电控时间戳→PC收到)
};

struct LatencyStats {
    std::deque<LatencyRecord> history;
    static constexpr int kMax = 300;

    void add(const LatencyRecord& r) {
        history.push_back(r);
        while (static_cast<int>(history.size()) > kMax) history.pop_front();
    }

    struct S { double avg, p95, mx; };
    S stat(double LatencyRecord::* field) const {
        if (history.empty()) return {0, 0, 0};
        std::vector<double> v;
        v.reserve(history.size());
        for (const auto& r : history) v.push_back(r.*field);
        std::sort(v.begin(), v.end());
        double s = std::accumulate(v.begin(), v.end(), 0.0);
        return {s / v.size(), v[std::min<int>(v.size()-1, v.size()*95/100)], v.back()};
    }
};

}  // namespace

int main(int argc, char** argv) {
    std::string cascade_config = "config/cascade.yaml";
    std::string enemy_config = "config/enemy.yaml";
    std::string camera_config = "config/camera.yaml";
    std::string control_config = "config/control.yaml";
    std::string video_path;
    std::string port = "/dev/ttyUSB0";
    int baud = 115200;
    bool show = true;
    bool use_serial = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i+1 < argc) cascade_config = argv[++i];
        else if (arg == "--enemy" && i+1 < argc) enemy_config = argv[++i];
        else if (arg == "--camera" && i+1 < argc) camera_config = argv[++i];
        else if (arg == "--control" && i+1 < argc) control_config = argv[++i];
        else if (arg == "--video" && i+1 < argc) video_path = argv[++i];
        else if (arg == "--port" && i+1 < argc) port = argv[++i];
        else if (arg == "--baud" && i+1 < argc) baud = std::stoi(argv[++i]);
        else if (arg == "--no-show") show = false;
        else if (arg == "--no-serial") use_serial = false;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║          端到端延迟测试 — system_delay_ms 标定          ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n\n";

    // ── 初始化各模块 ──
    detect::CascadeDetector detector;
    if (!detector.loadConfig(cascade_config)) { std::cerr << "✗ cascade 配置\n"; return 1; }
    detector.loadConfig(enemy_config);
    std::cout << "  ✓ 检测器\n";

    solve::TargetTracker tracker;
    tracker.loadConfig(cascade_config);
    std::cout << "  ✓ 跟踪器\n";

    control::ControlConfig ctrl_cfg;
    common::CameraModel cam_model;
    if (!control::loadControlConfig(control_config, &ctrl_cfg, &cam_model, camera_config)) {
        std::cerr << "✗ 控制配置\n"; return 1;
    }
    control::Controller controller(ctrl_cfg);
    std::cout << "  ✓ 控制器\n";

    // ── 串口 ──
    gimbal_serial::SerialPort serial;
    gimbal_serial::FrameParser parser;
    if (use_serial) {
        if (serial.open(port, baud)) {
            std::cout << "  ✓ 串口 " << port << " @ " << baud << "\n";
        } else {
            std::cout << "  ✗ 串口打开失败，跳过串口测量\n";
            use_serial = false;
        }
    } else {
        std::cout << "  - 串口跳过 (--no-serial)\n";
    }

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
        } else {
            std::cout << "  ✓ Galaxy 相机\n";
        }
    } else {
        cap.open(video_path);
        std::cout << "  ✓ 视频: " << video_path << "\n";
    }
    if (!use_galaxy && !cap.isOpened()) { std::cerr << "✗ 视频源\n"; return 1; }

    if (show) cv::namedWindow("Latency", cv::WINDOW_NORMAL);

    LatencyStats stats;
    auto last_print = std::chrono::steady_clock::now();
    int warmup = 10;
    double serial_rtt_sum = 0;
    int serial_rtt_count = 0;

    common::GimbalState ctrl_state{};
    ctrl_state.pitch = 0; ctrl_state.yaw = 0;

    std::cout << "\n  正在测量... Ctrl+C 或 q 退出\n\n";
    std::cout << "  环节:       取帧     推理     跟踪     控制     串口    SW总计    串口RTT\n";
    std::cout << "  单位:        ms       ms       ms       ms       ms       ms       ms\n";
    std::cout << "  ─────────────────────────────────────────────────────────────────────────\n";

    while (!g_exit.load()) {
        LatencyRecord rec;

        // ① 取帧
        auto t0 = Clock::now();
        cv::Mat frame;
        if (use_galaxy) {
            galaxy_camera::Frame gf;
            if (!galaxy.read(&gf, 50)) continue;
            if (gf.bgr.empty()) continue;
            frame = gf.bgr.clone();
        } else {
            if (!cap.read(frame) || frame.empty()) break;
        }
        auto t1 = Clock::now();
        rec.grab_us = std::chrono::duration_cast<Us>(t1 - t0).count();

        // ② YOLO 推理
        int64_t ts = common::nowMs();
        auto result = detector.detectCascade(frame, ts);
        auto t2 = Clock::now();
        rec.detect_us = std::chrono::duration_cast<Us>(t2 - t1).count();

        // ③ 跟踪器
        std::vector<std::pair<cv::Rect2f, float>> trk_in;
        for (const auto& lrx : result.laser_rxs) {
            trk_in.emplace_back(
                cv::Rect2f(lrx.bbox.x, lrx.bbox.y, lrx.bbox.width, lrx.bbox.height),
                lrx.confidence);
        }
        auto tracked = tracker.update(trk_in, ts);
        auto t3 = Clock::now();
        rec.track_us = std::chrono::duration_cast<Us>(t3 - t2).count();

        // ④ 控制器
        common::TargetMeasurement meas;
        meas.timestamp = ts;
        if (!result.laser_rxs.empty()) {
            const auto& d = result.laser_rxs.front();
            meas.valid = true;
            meas.uv = cv::Point2f(d.bbox.x + d.bbox.width/2.0f,
                                   d.bbox.y + d.bbox.height/2.0f);
            meas.confidence = d.confidence;
            meas.bbox_area = static_cast<float>(d.bbox.width) * d.bbox.height;
        }
        auto cmd = controller.update(meas, cam_model, ctrl_state);
        auto t4 = Clock::now();
        rec.ctrl_us = std::chrono::duration_cast<Us>(t4 - t3).count();

        // ⑤ 串口发送
        if (use_serial) {
            // 先读取回传 (非阻塞)
            std::vector<uint8_t> rx(256);
            int n = serial.read(rx.data(), static_cast<int>(rx.size()), 1);
            if (n > 0) {
                gimbal_serial::GimbalState gs;
                if (parser.push(rx.data(), static_cast<size_t>(n), &gs)) {
                    int64_t now_sys = common::nowSystemMs();
                    if (gs.timestamp > 1000000000000LL && now_sys > 1000000000000LL) {
                        double rtt = static_cast<double>(now_sys - static_cast<int64_t>(gs.timestamp));
                        if (rtt > 0 && rtt < 500) {
                            serial_rtt_sum += rtt * 1000.0;  // ms → us
                            serial_rtt_count++;
                        }
                    }
                    ctrl_state.pitch = gs.pitch;
                    ctrl_state.yaw = gs.yaw;
                }
            }
            uint8_t pkt[gimbal_serial::kTxFrameSize]{};
            gimbal_serial::packGimbalCommand(cmd, ctrl_state, pkt);
            serial.write(pkt, static_cast<int>(sizeof(pkt)));
        }
        auto t5 = Clock::now();
        rec.tx_us = std::chrono::duration_cast<Us>(t5 - t4).count();
        rec.sw_total_us = std::chrono::duration_cast<Us>(t5 - t0).count();
        rec.serial_rtt_us = (serial_rtt_count > 0) ? serial_rtt_sum / serial_rtt_count : 0;

        if (warmup > 0) { warmup--; continue; }
        stats.add(rec);

        // ── 每秒打印 ──
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print).count() >= 1000) {
            auto g  = stats.stat(&LatencyRecord::grab_us);
            auto d  = stats.stat(&LatencyRecord::detect_us);
            auto tr = stats.stat(&LatencyRecord::track_us);
            auto c  = stats.stat(&LatencyRecord::ctrl_us);
            auto tx = stats.stat(&LatencyRecord::tx_us);
            auto sw = stats.stat(&LatencyRecord::sw_total_us);
            double rtt_ms = (serial_rtt_count > 0) ? serial_rtt_sum / serial_rtt_count / 1000.0 : 0;

            char line[256];
            snprintf(line, sizeof(line),
                "  avg:   %5.1f    %5.1f    %5.1f    %5.1f    %5.1f    %5.1f    %5.1f",
                g.avg/1000, d.avg/1000, tr.avg/1000, c.avg/1000, tx.avg/1000, sw.avg/1000, rtt_ms);
            std::cout << line << "\n";
            last_print = now;
        }

        // ── 可视化 ──
        if (show) {
            for (const auto& p : result.planes)
                cv::rectangle(frame, p.bbox, cv::Scalar(0, 255, 0), 2);
            for (const auto& lrx : result.laser_rxs)
                cv::rectangle(frame, lrx.bbox, cv::Scalar(0, 0, 255), 2);

            int bx = 10, by = frame.rows - 100;
            cv::rectangle(frame, cv::Rect(bx, by, 220, 95), cv::Scalar(0,0,0), cv::FILLED);
            auto drawBar = [&](int i, double us, const std::string& lb, cv::Scalar c) {
                int y = by + 5 + i * 16;
                int len = std::min(static_cast<int>(us/1000*4), 150);
                cv::rectangle(frame, cv::Point(bx+55,y), cv::Point(bx+55+len,y+12), c, cv::FILLED);
                cv::putText(frame, lb, cv::Point(bx+2,y+10), 0, 0.3, {200,200,200}, 1);
                cv::putText(frame, cv::format("%.1fms",us/1000), cv::Point(bx+58+len,y+10), 0, 0.3, c, 1);
            };
            drawBar(0, rec.grab_us, "grab", {0,200,200});
            drawBar(1, rec.detect_us, "detect", {0,255,0});
            drawBar(2, rec.track_us, "track", {255,100,0});
            drawBar(3, rec.ctrl_us, "ctrl", {200,200,0});
            drawBar(4, rec.sw_total_us, "TOTAL", {0,0,255});

            cv::imshow("Latency", frame);
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') break;
        }
    }

    if (use_galaxy) { galaxy.stopGrabbing(); galaxy.close(); }
    if (show) cv::destroyAllWindows();

    // ════════════════════ 最终报告 ════════════════════
    auto g  = stats.stat(&LatencyRecord::grab_us);
    auto d  = stats.stat(&LatencyRecord::detect_us);
    auto tr = stats.stat(&LatencyRecord::track_us);
    auto c  = stats.stat(&LatencyRecord::ctrl_us);
    auto tx = stats.stat(&LatencyRecord::tx_us);
    auto sw = stats.stat(&LatencyRecord::sw_total_us);
    double rtt_ms = (serial_rtt_count > 0) ? serial_rtt_sum / serial_rtt_count / 1000.0 : 0;

    std::cout << "\n";
    std::cout << "  ╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "  ║                    延迟测试最终报告                          ║\n";
    std::cout << "  ╠══════════════════════════════════════════════════════════════╣\n";
    std::cout << "  ║  环节          平均(ms)   P95(ms)   最大(ms)                ║\n";
    std::cout << "  ╟──────────────────────────────────────────────────────────────╢\n";
    auto pr = [](const char* name, LatencyStats::S s) {
        char buf[80];
        snprintf(buf, sizeof(buf), "  ║  %-10s  %7.1f   %7.1f   %7.1f                  ║",
                 name, s.avg/1000, s.p95/1000, s.mx/1000);
        std::cout << buf << "\n";
    };
    pr("取帧", g);
    pr("推理", d);
    pr("跟踪", tr);
    pr("控制", c);
    pr("串口发送", tx);
    std::cout << "  ╟──────────────────────────────────────────────────────────────╢\n";
    pr("PC总计", sw);
    char rttline[80];
    snprintf(rttline, sizeof(rttline),
        "  ║  %-10s  %7.1f   (电控时间戳→PC接收)              ║", "串口RTT", rtt_ms);
    std::cout << rttline << "\n";
    std::cout << "  ╠══════════════════════════════════════════════════════════════╣\n";

    double recommended = sw.avg / 1000.0 + rtt_ms / 2.0;
    char recline[120];
    snprintf(recline, sizeof(recline),
        "  ║  ★ 建议 system_delay_ms = %.1f                            ║", recommended);
    std::cout << recline << "\n";
    snprintf(recline, sizeof(recline),
        "  ║    = PC软件 %.1f + 串口单程 %.1f                            ║",
        sw.avg/1000.0, rtt_ms/2.0);
    std::cout << recline << "\n";
    std::cout << "  ║                                                              ║\n";
    std::cout << "  ║  用法: 把上面的值填到 config/control.yaml:                   ║\n";
    std::cout << "  ║        system_delay_ms: " << std::fixed << std::setprecision(1) << recommended
              << "                                    ║\n";
    std::cout << "  ╚══════════════════════════════════════════════════════════════╝\n";

    return 0;
}
