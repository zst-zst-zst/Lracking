#ifndef TRACKING_COLOR_VOTING_H
#define TRACKING_COLOR_VOTING_H

#include <cstdint>
#include <string>
#include <opencv2/core.hpp>

namespace tracking_app {

// 颜色检测原始结果
struct ColorResult {
    cv::Mat mask;       // 颜色掩码 (单通道)
    float ratio;        // 颜色像素占比 (0~1)
    int pixel_count;    // 颜色像素数
};

// 检测指定颜色 (red/blue/purple), HSV inRange + 紫色双段
ColorResult detectColor(const cv::Mat& bgr_roi,
                        const std::string& color,
                        int min_sat = 80,
                        int min_val = 80);

// 单帧颜色判定 (red/blue/purple/unknown + 是否敌方)
struct ColorConfirmation {
    std::string detected_color;  // red/blue/purple/unknown
    bool is_enemy;               // 是否为敌方颜色
    float confidence;            // 占比形式的置信度
};

// 单帧观测: 三色比例 + 判定
struct ColorObservation {
    ColorConfirmation conf;
    float red_ratio = 0.0f;
    float blue_ratio = 0.0f;
    float purple_ratio = 0.0f;
};

// 综合三色比例做敌我判定. 紫色优先 (说明追踪稳定击中).
ColorObservation observeEnemyColor(const cv::Mat& bgr_roi,
                                   const std::string& enemy_color,
                                   float enemy_threshold = 0.15f,
                                   float purple_threshold = 0.10f);

// 检测结果 + 颜色信息 + 几何, 用于颜色与跟踪匹配
struct DetColorInfo {
    int det_index = -1;
    cv::Point2f center;
    cv::Rect bbox;
    ColorObservation obs;
    std::string status;
};

// 紫色转换记录: 跟踪到击中的瞬间快照, 用于离线分析命中精度
struct PurpleTransitionRecord {
    bool valid = false;
    int64_t ts_ms = 0;
    int track_id = -1;
    std::string prev_color;
    std::string current_color;
    float confidence = 0.0f;
    float red_ratio = 0.0f;
    float blue_ratio = 0.0f;
    float purple_ratio = 0.0f;
    float prediction_horizon_s = 0.0f;
    float predicted_x = 0.0f;
    float predicted_y = 0.0f;
    float prediction_error_x = 0.0f;
    float prediction_error_y = 0.0f;
    cv::Rect bbox;
    cv::Point2f center{};
    cv::Point2f velocity{};
    float cmd_yaw = 0.0f;
    float cmd_pitch = 0.0f;
    float state_yaw = 0.0f;
    float state_pitch = 0.0f;
    int tracked_count = 0;
    double fps = 0.0;
};

}  // namespace tracking_app

#endif  // TRACKING_COLOR_VOTING_H
