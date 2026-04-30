#ifndef TOOLS_PLOTTER_H
#define TOOLS_PLOTTER_H

// ============================================================================
// UDP JSON Plotter — 通过 UDP 发送 JSON 数据到 PlotJuggler
// ============================================================================
// 用法:
//   1. 打开 PlotJuggler, 添加 UDP 数据源, 端口 9870
//   2. 代码中:
//        tools::Plotter plotter;
//        nlohmann::json j;
//        j["gimbal_pitch"] = state.pitch;
//        j["cmd_pitch"] = cmd.pitch;
//        plotter.plot(j);
//   3. PlotJuggler 自动识别 JSON key 为曲线名
// ============================================================================

#include <netinet/in.h>

#include <mutex>
#include <string>

#include <nlohmann/json.hpp>

namespace tools {

class Plotter {
public:
    explicit Plotter(const std::string& host = "127.0.0.1", uint16_t port = 9870);
    ~Plotter();

    Plotter(const Plotter&) = delete;
    Plotter& operator=(const Plotter&) = delete;

    // 发送一帧 JSON 数据
    void plot(const nlohmann::json& json);

private:
    int socket_ = -1;
    sockaddr_in dest_{};
    std::mutex mutex_;
};

}  // namespace tools

#endif  // TOOLS_PLOTTER_H
