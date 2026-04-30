#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <array>
#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

namespace common {

enum class GimbalFeedbackMode : uint8_t {
    Idle = 0,
    AutoAim = 1,
    SmallBuff = 2,
    BigBuff = 3,
};

enum class GimbalCommandMode : uint8_t {
    Idle = 0,
    Control = 1,
    ControlAndFire = 2,
};

struct TargetMeasurement {
    bool valid = false;
    int64_t timestamp = 0;
    cv::Point2f uv{};
    float confidence = 0.0f;
    int class_id = -1;
    std::string label;
    float bbox_area = 0.0f;      // bbox 面积 (px²), 用于距离估算/自适应增益
    cv::Point2f velocity{};      // 跟踪器估计的速度 (px/s), 用于延迟补偿
};

struct GimbalState {
    float pitch = 0.0f;
    float yaw = 0.0f;
    float pitch_rate = 0.0f;
    float yaw_rate = 0.0f;
    int64_t timestamp = 0;
    uint8_t mode = static_cast<uint8_t>(GimbalFeedbackMode::Idle);
    std::array<float, 4> quaternion_wxyz{1.0f, 0.0f, 0.0f, 0.0f};
    float bullet_speed = 0.0f;
    uint16_t bullet_count = 0;
};

struct GimbalCommand {
    float pitch = 0.0f;
    float yaw = 0.0f;
    float pitch_rate = 0.0f;
    float yaw_rate = 0.0f;
    int64_t timestamp = 0;
    uint8_t mode = static_cast<uint8_t>(GimbalCommandMode::Idle);
    float pitch_acc = 0.0f;
    float yaw_acc = 0.0f;
};

struct CameraModel {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

struct Boresight {
    double u_L = 0.0;
    double v_L = 0.0;
};

}  // namespace common

#endif  // COMMON_TYPES_H
