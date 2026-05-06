#include "color_voting.h"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace tracking_app {

ColorResult detectColor(const cv::Mat& bgr_roi,
                        const std::string& color,
                        int min_sat,
                        int min_val) {
    ColorResult result;
    result.ratio = 0.0f;
    result.pixel_count = 0;

    if (bgr_roi.empty()) {
        result.mask = cv::Mat();
        return result;
    }

    cv::Mat hsv;
    cv::cvtColor(bgr_roi, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    if (color == "blue") {
        // 蓝色 H∈[90, 130]
        cv::inRange(hsv, cv::Scalar(90, min_sat, min_val),
                    cv::Scalar(130, 255, 255), mask);
    } else if (color == "red") {
        // 红色 H∈[0,15] ∪ [165,180]
        cv::Mat mask1, mask2;
        cv::inRange(hsv, cv::Scalar(0, min_sat, min_val),
                    cv::Scalar(15, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(165, min_sat, min_val),
                    cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;
    } else if (color == "purple") {
        // 紫色 H∈[130, 165]，允许低饱和度（紫白色）
        cv::Mat mask_purple, mask_pale;
        cv::inRange(hsv, cv::Scalar(130, min_sat, min_val),
                    cv::Scalar(165, 255, 255), mask_purple);
        cv::inRange(hsv, cv::Scalar(130, 30, min_val),
                    cv::Scalar(165, min_sat - 1, 255), mask_pale);
        mask = mask_purple | mask_pale;
    } else {
        result.mask = cv::Mat();
        return result;
    }

    result.mask = mask;
    result.pixel_count = cv::countNonZero(mask);
    int total = mask.rows * mask.cols;
    result.ratio = total > 0 ? static_cast<float>(result.pixel_count) / total : 0.0f;
    return result;
}

ColorObservation observeEnemyColor(const cv::Mat& bgr_roi,
                                   const std::string& enemy_color,
                                   float enemy_threshold,
                                   float purple_threshold) {
    ColorObservation obs;
    ColorConfirmation conf;
    conf.is_enemy = false;
    conf.confidence = 0.0f;

    if (bgr_roi.empty()) {
        conf.detected_color = "unknown";
        obs.conf = conf;
        return obs;
    }

    auto red_res = detectColor(bgr_roi, "red");
    auto blue_res = detectColor(bgr_roi, "blue");
    auto purple_res = detectColor(bgr_roi, "purple");

    obs.red_ratio = red_res.ratio;
    obs.blue_ratio = blue_res.ratio;
    obs.purple_ratio = purple_res.ratio;

    float max_ratio = std::max({red_res.ratio, blue_res.ratio, purple_res.ratio});
    conf.confidence = max_ratio;

    if (max_ratio < 0.05f) {
        conf.detected_color = "unknown";
        obs.conf = conf;
        return obs;
    }

    // 紫色优先：紫色说明追踪效果好 (击中)
    if (purple_res.ratio >= purple_threshold && purple_res.ratio >= max_ratio * 0.8f) {
        conf.detected_color = "purple";
        obs.conf = conf;
        return obs;
    }

    // 判断是否为敌方颜色
    if (enemy_color == "red") {
        if (red_res.ratio >= enemy_threshold && red_res.ratio >= blue_res.ratio) {
            conf.detected_color = "red";
            conf.is_enemy = true;
        } else if (blue_res.ratio >= enemy_threshold) {
            conf.detected_color = "blue";
            conf.is_enemy = false;
        } else {
            conf.detected_color = "unknown";
        }
    } else {  // enemy_color == "blue"
        if (blue_res.ratio >= enemy_threshold && blue_res.ratio >= red_res.ratio) {
            conf.detected_color = "blue";
            conf.is_enemy = true;
        } else if (red_res.ratio >= enemy_threshold) {
            conf.detected_color = "red";
            conf.is_enemy = false;
        } else {
            conf.detected_color = "unknown";
        }
    }

    obs.conf = conf;
    return obs;
}

}  // namespace tracking_app
