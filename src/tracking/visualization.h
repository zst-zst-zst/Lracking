#ifndef TRACKING_VISUALIZATION_H
#define TRACKING_VISUALIZATION_H

#include <deque>
#include <opencv2/core.hpp>

namespace tracking_app {

// Catmull-Rom 平滑插值, 给离散轨迹拟合连续曲线
cv::Point2f catmullRom(const cv::Point2f& p0,
                       const cv::Point2f& p1,
                       const cv::Point2f& p2,
                       const cv::Point2f& p3,
                       float t);

// 绘制平滑历史轨迹 (alpha 渐变, 远端淡)
void drawSmoothTrail(cv::Mat& img,
                     const std::deque<cv::Point2f>& trail,
                     cv::Scalar color,
                     int max_len = 120,
                     int thickness = 2);

// 历史轨迹 + 基于速度的未来预测段 (黄色)
void drawPredictedTrail(cv::Mat& img,
                        const std::deque<cv::Point2f>& trail,
                        const cv::Point2f& velocity_px_s,
                        cv::Scalar history_color,
                        int max_len = 120,
                        float prediction_horizon_s = 0.5f);

}  // namespace tracking_app

#endif  // TRACKING_VISUALIZATION_H
