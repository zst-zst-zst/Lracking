#include "gimbal_serial/protocol.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

#include "common/time_utils.h"

namespace gimbal_serial {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;

#pragma pack(push, 1)
struct RawGimbalToVision {
    uint8_t head[2];
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint8_t tail[2];
};

struct RawVisionToGimbal {
    uint8_t head[2];
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint8_t tail[2];
};
#pragma pack(pop)

static_assert(sizeof(RawGimbalToVision) == kRxFrameSize, "Unexpected RX frame size");
static_assert(sizeof(RawVisionToGimbal) == kTxFrameSize, "Unexpected TX frame size");

float radToDeg(float value) {
    return static_cast<float>(value * kRadToDeg);
}

float degToRad(float value) {
    return static_cast<float>(value * kDegToRad);
}

float normalizeDeg(float value) {
    float wrapped = std::fmod(value, 360.0f);
    if (wrapped > 180.0f) {
        wrapped -= 360.0f;
    } else if (wrapped <= -180.0f) {
        wrapped += 360.0f;
    }
    return wrapped;
}

uint8_t clampTxMode(uint8_t mode) {
    if (mode > static_cast<uint8_t>(common::GimbalCommandMode::ControlAndFire)) {
        return static_cast<uint8_t>(common::GimbalCommandMode::Control);
    }
    return mode;
}
}  // namespace

bool parseGimbalState(const uint8_t* buf, size_t len, GimbalState* out) {
    if (!buf || !out || len < kRxFrameSize) {
        return false;
    }
    if (buf[0] != kFrameHead[0] || buf[1] != kFrameHead[1] ||
        buf[kRxFrameSize - 2] != kFrameTail[0] || buf[kRxFrameSize - 1] != kFrameTail[1]) {
        return false;
    }

    RawGimbalToVision frame{};
    std::memcpy(&frame, buf, sizeof(frame));

    out->mode = frame.mode;
    out->yaw = radToDeg(frame.yaw);
    out->yaw_rate = radToDeg(frame.yaw_vel);
    out->pitch = radToDeg(frame.pitch);
    out->pitch_rate = radToDeg(frame.pitch_vel);
    out->bullet_speed = 0.0f;
    out->bullet_count = 0;
    out->quaternion_wxyz = {1.0f, 0.0f, 0.0f, 0.0f};
    out->timestamp = common::nowSystemMs();
    return true;
}

void packGimbalCommand(const GimbalCommand& cmd,
                       const GimbalState& state,
                       uint8_t out[kTxFrameSize]) {
    RawVisionToGimbal frame{};
    frame.head[0] = kFrameHead[0];
    frame.head[1] = kFrameHead[1];
    frame.tail[0] = kFrameTail[0];
    frame.tail[1] = kFrameTail[1];

    frame.mode = clampTxMode(cmd.mode);

    float yaw_delta_deg = normalizeDeg(cmd.yaw - state.yaw);
    float pitch_delta_deg = normalizeDeg(cmd.pitch - state.pitch);
    if (frame.mode == static_cast<uint8_t>(common::GimbalCommandMode::Idle)) {
        yaw_delta_deg = 0.0f;
        pitch_delta_deg = 0.0f;
    }

    frame.yaw = degToRad(yaw_delta_deg);
    frame.yaw_vel = degToRad(cmd.yaw_rate);
    frame.yaw_acc = degToRad(cmd.yaw_acc);
    frame.pitch = degToRad(pitch_delta_deg);
    frame.pitch_vel = degToRad(cmd.pitch_rate);
    frame.pitch_acc = degToRad(cmd.pitch_acc);

    std::memcpy(out, &frame, sizeof(frame));
}

void packGimbalCommand(const GimbalCommand& cmd, uint8_t out[kTxFrameSize]) {
    packGimbalCommand(cmd, GimbalState{}, out);
}

bool FrameParser::push(const uint8_t* data, size_t len, GimbalState* out) {
    if (!data || len == 0) {
        return false;
    }

    stats_.total_bytes += len;
    buffer_.insert(buffer_.end(), data, data + len);

    bool got = false;
    while (buffer_.size() >= kFrameHead.size()) {
        auto head_it = std::search(buffer_.begin(), buffer_.end(),
                                   kFrameHead.begin(), kFrameHead.end());
        if (head_it == buffer_.end()) {
            const size_t keep = kFrameHead.size() - 1;
            if (buffer_.size() > keep) {
                stats_.discarded_bytes += buffer_.size() - keep;
                buffer_.erase(buffer_.begin(),
                              buffer_.end() - static_cast<std::ptrdiff_t>(keep));
            }
            return got;
        }

        const size_t start = static_cast<size_t>(head_it - buffer_.begin());
        if (start > 0) {
            stats_.discarded_bytes += start;
            buffer_.erase(buffer_.begin(),
                          buffer_.begin() + static_cast<std::ptrdiff_t>(start));
        }
        if (buffer_.size() < kRxFrameSize) {
            return got;
        }

        if (buffer_[kRxFrameSize - 2] == kFrameTail[0] &&
            buffer_[kRxFrameSize - 1] == kFrameTail[1]) {
            GimbalState parsed;
            const bool ok = parseGimbalState(buffer_.data(), kRxFrameSize, &parsed);
            buffer_.erase(buffer_.begin(),
                          buffer_.begin() + static_cast<std::ptrdiff_t>(kRxFrameSize));
            if (ok && out) {
                *out = parsed;
                got = true;
            } else {
                stats_.bad_frames++;
            }
            continue;
        }

        stats_.bad_frames++;
        stats_.discarded_bytes++;
        buffer_.erase(buffer_.begin());
    }
    return got;
}

const FrameParser::Stats& FrameParser::stats() const {
    return stats_;
}

void FrameParser::resetStats() {
    stats_ = {};
}

}  // namespace gimbal_serial
