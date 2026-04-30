#include "plotter.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>

namespace tools {

Plotter::Plotter(const std::string& host, uint16_t port) {
    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) {
        std::cerr << "[Plotter] socket 创建失败\n";
        return;
    }
    dest_.sin_family = AF_INET;
    dest_.sin_port = ::htons(port);
    dest_.sin_addr.s_addr = ::inet_addr(host.c_str());
}

Plotter::~Plotter() {
    if (socket_ >= 0) ::close(socket_);
}

void Plotter::plot(const nlohmann::json& json) {
    if (socket_ < 0) return;
    std::lock_guard<std::mutex> lock(mutex_);
    auto data = json.dump();
    ::sendto(socket_, data.c_str(), data.length(), 0,
             reinterpret_cast<const sockaddr*>(&dest_), sizeof(dest_));
}

}  // namespace tools
