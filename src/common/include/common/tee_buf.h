#ifndef COMMON_TEE_BUF_H
#define COMMON_TEE_BUF_H

#include <streambuf>

namespace common {

// TeeBuf: 同时写入两个 streambuf (用于日志 + 终端双写)
// 用法:
//   std::ofstream log("log.txt");
//   common::TeeBuf tee(std::cout.rdbuf(), log.rdbuf());
//   std::cout.rdbuf(&tee);  // cout 同时输出到终端和日志文件
class TeeBuf : public std::streambuf {
public:
    TeeBuf(std::streambuf* a, std::streambuf* b) : a_(a), b_(b) {}

protected:
    int overflow(int c) override {
        if (c == EOF) {
            return !EOF;
        }
        const int r1 = a_->sputc(static_cast<char>(c));
        const int r2 = b_->sputc(static_cast<char>(c));
        return (r1 == EOF || r2 == EOF) ? EOF : c;
    }
    int sync() override {
        const int r1 = a_->pubsync();
        const int r2 = b_->pubsync();
        return (r1 == 0 && r2 == 0) ? 0 : -1;
    }

private:
    std::streambuf* a_;
    std::streambuf* b_;
};

}  // namespace common

#endif  // COMMON_TEE_BUF_H
