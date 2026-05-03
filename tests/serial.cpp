// ══════════════════════════════════════════════════════════════
// 串口延迟测量工具 — 同步往返测试
// 测量: 发送指令 → 等待回复 的往返时间 (RTT)
// 用法: ./serial [--port /dev/ttyUSB0] [--baud 115200] [--count 200]
// ══════════════════════════════════════════════════════════════
#include "common/time_utils.h"
#include "gimbal_serial/protocol.h"
#include "gimbal_serial/serial_port.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;
using us = std::chrono::microseconds;

int main(int argc, char** argv) {
    std::string port = "/dev/ttyUSB0";
    int baud = 115200;
    int count = 200;           // 测量次数
    int warmup = 20;           // 预热次数 (丢弃)
    int read_timeout_ms = 50;  // 单次读超时

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc)          port = argv[++i];
        else if (arg == "--baud" && i + 1 < argc)     baud = std::stoi(argv[++i]);
        else if (arg == "--count" && i + 1 < argc)    count = std::stoi(argv[++i]);
        else if (arg == "--warmup" && i + 1 < argc)   warmup = std::stoi(argv[++i]);
        else if (arg == "--timeout" && i + 1 < argc)  read_timeout_ms = std::stoi(argv[++i]);
        else if (arg == "--help") {
            std::cout << "串口往返延迟测量 (5m USB-TTL 线缆)\n"
                      << "  --port    串口设备 (default: /dev/ttyUSB0)\n"
                      << "  --baud    波特率   (default: 115200)\n"
                      << "  --count   测量次数 (default: 200)\n"
                      << "  --warmup  预热次数 (default: 20)\n"
                      << "  --timeout 读超时ms (default: 50)\n";
            return 0;
        }
    }

    // ── 打开串口 ──
    gimbal_serial::SerialPort serial;
    if (!serial.open(port, baud)) {
        std::cerr << "无法打开串口 " << port << "\n";
        return 1;
    }
    std::cout << "═══════════════════════════════════════\n";
    std::cout << "  串口延迟测量工具\n";
    std::cout << "  端口: " << port << "  波特率: " << baud << "\n";
    std::cout << "  测量: " << count << " 次  预热: " << warmup << " 次\n";
    std::cout << "═══════════════════════════════════════\n\n";

    gimbal_serial::FrameParser parser;
    std::vector<uint8_t> rx_buf(2048);

    // 排空辅助函数
    auto drain = [&](int drain_ms = 200) {
        auto t0 = Clock::now();
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   Clock::now() - t0).count() < drain_ms) {
            serial.read(rx_buf.data(), static_cast<int>(rx_buf.size()), 5);
        }
        parser = gimbal_serial::FrameParser{};
    };

    // ── 测量 1: 通信建立 + 云台上报频率 ──
    // 云台是自主上报模式，需要先发一个指令"唤醒"通信，然后持续收发
    std::cout << "[1/3] 建立通信 + 测量云台上报频率...\n";

    // 发几个指令唤醒云台
    for (int i = 0; i < 10; ++i) {
        gimbal_serial::GimbalCommand cmd{};
        cmd.mode = static_cast<uint8_t>(common::GimbalCommandMode::Control);
        cmd.timestamp = common::nowMs();
        uint8_t frame[gimbal_serial::kTxFrameSize]{};
        gimbal_serial::packGimbalCommand(cmd, frame);
        serial.write(frame, static_cast<int>(sizeof(frame)));
        // 同时读取，不要让缓冲区堆积
        serial.read(rx_buf.data(), static_cast<int>(rx_buf.size()), 10);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // 测量上报间隔：持续收发 (模拟正常工作模式)
    std::vector<double> rx_intervals;
    rx_intervals.reserve(200);
    gimbal_serial::GimbalState last_state{};
    bool has_state = false;
    auto last_rx = Clock::now();
    bool first_rx = true;
    int total_rx_bytes = 0;
    int total_parsed = 0;

    for (int i = 0; i < 1000 && rx_intervals.size() < 200; ++i) {
        // 发送 (保持通信活跃)
        gimbal_serial::GimbalCommand cmd{};
        cmd.mode = static_cast<uint8_t>(common::GimbalCommandMode::Control);
        cmd.timestamp = common::nowMs();
        uint8_t frame[gimbal_serial::kTxFrameSize]{};
        if (has_state) {
            gimbal_serial::packGimbalCommand(cmd, last_state, frame);
        } else {
            gimbal_serial::packGimbalCommand(cmd, frame);
        }
        serial.write(frame, static_cast<int>(sizeof(frame)));

        // 读取
        int n = serial.read(rx_buf.data(), static_cast<int>(rx_buf.size()), 10);
        if (n > 0) {
            total_rx_bytes += n;
            gimbal_serial::GimbalState state;
            if (parser.push(rx_buf.data(), static_cast<size_t>(n), &state)) {
                total_parsed++;
                auto now = Clock::now();
                last_state = state;
                has_state = true;
                if (!first_rx) {
                    double interval_ms = std::chrono::duration_cast<us>(now - last_rx).count() / 1000.0;
                    rx_intervals.push_back(interval_ms);
                }
                first_rx = false;
                last_rx = now;
            }
        }
        // 每 100 次打印进度
        if ((i + 1) % 100 == 0) {
            const auto& pst = parser.stats();
            std::cout << "  [" << (i+1) << "] rx_bytes=" << total_rx_bytes
                      << " parsed=" << total_parsed
                      << " bad=" << pst.bad_frames
                      << " discard=" << pst.discarded_bytes << "\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!has_state) {
        const auto& pst = parser.stats();
        std::cerr << "\n[错误] 无法解析云台数据!\n"
                  << "  收到字节: " << total_rx_bytes << "\n"
                  << "  解析成功: " << total_parsed << "\n"
                  << "  坏帧:     " << pst.bad_frames << "\n"
                  << "  丢弃字节: " << pst.discarded_bytes << "\n";
        if (total_rx_bytes == 0) {
            std::cerr << "  → 串口没有收到任何数据! 检查接线和云台上电\n";
        } else {
            std::cerr << "  → 收到数据但无法解析! 可能协议不匹配或线路噪声\n";
        }
        serial.close();
        return 1;
    }
    std::cout << "  已收到 " << rx_intervals.size() << " 个状态包\n";

    // ── 测量 2: TX 写入耗时 (边发边收，不堆积) ──
    std::cout << "[2/3] 测量发送 (TX) 耗时...\n";
    drain(100);
    std::vector<double> tx_times;
    tx_times.reserve(count);
    for (int i = 0; i < count + warmup; ++i) {
        gimbal_serial::GimbalCommand cmd{};
        cmd.mode = static_cast<uint8_t>(common::GimbalCommandMode::Control);
        cmd.timestamp = common::nowMs();
        uint8_t frame[gimbal_serial::kTxFrameSize]{};
        if (has_state) {
            gimbal_serial::packGimbalCommand(cmd, last_state, frame);
        } else {
            gimbal_serial::packGimbalCommand(cmd, frame);
        }

        auto t0 = Clock::now();
        serial.write(frame, static_cast<int>(sizeof(frame)));
        auto t1 = Clock::now();

        if (i >= warmup) {
            tx_times.push_back(
                std::chrono::duration_cast<us>(t1 - t0).count() / 1000.0);
        }
        // 边发边收，防止缓冲区堆积
        int n = serial.read(rx_buf.data(), static_cast<int>(rx_buf.size()), 5);
        if (n > 0) {
            gimbal_serial::GimbalState state;
            if (parser.push(rx_buf.data(), static_cast<size_t>(n), &state)) {
                last_state = state;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }

    // ── 测量 3: 发送-到-接收延迟 ──
    // 不是严格 RTT，而是: 发送指令 → 下一个状态包到达的时间
    // 这反映了实际控制环路中的串口延迟
    std::cout << "[3/3] 测量发送→接收延迟...\n";
    drain(200);

    std::vector<double> rtt_times;
    rtt_times.reserve(count);
    int timeouts = 0;
    int success = 0;

    for (int i = 0; i < count + warmup; ++i) {
        // 排空残余：读到没有为止
        for (int d = 0; d < 20; ++d) {
            int n = serial.read(rx_buf.data(), static_cast<int>(rx_buf.size()), 3);
            if (n <= 0) break;
            gimbal_serial::GimbalState state;
            if (parser.push(rx_buf.data(), static_cast<size_t>(n), &state)) {
                last_state = state;
            }
        }

        // 发送
        gimbal_serial::GimbalCommand cmd{};
        cmd.mode = static_cast<uint8_t>(common::GimbalCommandMode::Control);
        cmd.timestamp = common::nowMs();
        uint8_t frame[gimbal_serial::kTxFrameSize]{};
        gimbal_serial::packGimbalCommand(cmd, last_state, frame);

        auto t_send = Clock::now();
        serial.write(frame, static_cast<int>(sizeof(frame)));

        // 等待下一个状态包
        bool got_reply = false;
        auto deadline = t_send + std::chrono::milliseconds(read_timeout_ms);
        while (Clock::now() < deadline) {
            int n = serial.read(rx_buf.data(), static_cast<int>(rx_buf.size()), 2);
            if (n > 0) {
                gimbal_serial::GimbalState state;
                if (parser.push(rx_buf.data(), static_cast<size_t>(n), &state)) {
                    auto t_recv = Clock::now();
                    double delay_ms = std::chrono::duration_cast<us>(t_recv - t_send).count() / 1000.0;
                    last_state = state;
                    got_reply = true;
                    if (i >= warmup) {
                        rtt_times.push_back(delay_ms);
                        success++;
                    }
                    break;
                }
            }
        }
        if (!got_reply && i >= warmup) {
            timeouts++;
        }

        if ((i + 1) % 50 == 0) {
            std::cout << "  进度: " << (i + 1) << "/" << (count + warmup) << "\n";
        }
    }

    serial.close();

    // ── 统计输出 ──
    auto stats = [](const std::vector<double>& v, const std::string& name) {
        if (v.empty()) {
            std::cout << "  " << name << ": 无数据\n";
            return;
        }
        std::vector<double> sorted = v;
        std::sort(sorted.begin(), sorted.end());
        double sum = std::accumulate(sorted.begin(), sorted.end(), 0.0);
        double mean = sum / sorted.size();
        double sq_sum = 0;
        for (auto x : sorted) sq_sum += (x - mean) * (x - mean);
        double stddev = std::sqrt(sq_sum / sorted.size());
        double median = sorted[sorted.size() / 2];
        double p5 = sorted[static_cast<size_t>(sorted.size() * 0.05)];
        double p95 = sorted[static_cast<size_t>(sorted.size() * 0.95)];
        double p99 = sorted[static_cast<size_t>(sorted.size() * 0.99)];

        std::cout << "  " << name << ":\n"
                  << "    样本数: " << sorted.size() << "\n"
                  << "    最小:   " << std::fixed << std::setprecision(2) << sorted.front() << " ms\n"
                  << "    最大:   " << sorted.back() << " ms\n"
                  << "    平均:   " << mean << " ms\n"
                  << "    中位:   " << median << " ms\n"
                  << "    标准差: " << stddev << " ms\n"
                  << "    P5:     " << p5 << " ms\n"
                  << "    P95:    " << p95 << " ms\n"
                  << "    P99:    " << p99 << " ms\n";
    };

    std::cout << "\n═══════════════════════════════════════\n";
    std::cout << "  测量结果 (" << port << " @ " << baud << " baud, 5m线)\n";
    std::cout << "═══════════════════════════════════════\n\n";

    stats(tx_times, "TX 发送耗时 (serial.write 29字节)");
    std::cout << "\n";
    stats(rtt_times, "RTT 往返延迟 (发送→收到回复)");
    std::cout << "    超时次数: " << timeouts << " / " << (success + timeouts) << "\n";
    std::cout << "\n";
    stats(rx_intervals, "RX 接收间隔 (云台上报周期)");

    // ── 建议 ──
    if (!rtt_times.empty()) {
        std::vector<double> sorted = rtt_times;
        std::sort(sorted.begin(), sorted.end());
        double median_rtt = sorted[sorted.size() / 2];
        double p95_rtt = sorted[static_cast<size_t>(sorted.size() * 0.95)];
        std::cout << "\n── 建议 ──\n";
        std::cout << "  串口往返延迟中位数: " << std::fixed << std::setprecision(1)
                  << median_rtt << " ms\n";
        std::cout << "  串口单程延迟估计:   " << median_rtt / 2.0 << " ms\n";
        std::cout << "  软件处理延迟:       ~3 ms (从 tracking 日志)\n";
        double total = 3.0 + median_rtt / 2.0;
        std::cout << "  建议 system_delay_ms: " << std::setprecision(0)
                  << std::ceil(total) << " ~ " << std::ceil(p95_rtt / 2.0 + 3.0) << " ms\n";
    }

    return 0;
}
