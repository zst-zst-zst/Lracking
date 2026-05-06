#include "visualization.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace tracking_app {

cv::Point2f catmullRom(const cv::Point2f& p0,
                       const cv::Point2f& p1,
                       const cv::Point2f& p2,
                       const cv::Point2f& p3,
                       float t) {
    const float t2 = t * t;
    const float t3 = t2 * t;
    return 0.5f * ((2.0f * p1) + (-p0 + p2) * t +
                   (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                   (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
}

void drawSmoothTrail(cv::Mat& img,
                     const std::deque<cv::Point2f>& trail,
                     cv::Scalar color,
                     int max_len,
                     int thickness) {
    if (trail.size() < 2) return;

    std::vector<cv::Point2f> pts(trail.begin(), trail.end());
    const int n = static_cast<int>(pts.size());
    const int start = std::max(0, n - max_len);
    if (n - start < 2) return;

    constexpr int kSamplesPerSegment = 8;
    for (int i = start; i < n - 1; ++i) {
        const cv::Point2f& p1 = pts[i];
        const cv::Point2f& p2 = pts[i + 1];
        const cv::Point2f& p0 = (i > start) ? pts[i - 1] : p1;
        const cv::Point2f& p3 = (i + 2 < n) ? pts[i + 2] : p2;

        cv::Point2f prev = p1;
        for (int s = 1; s <= kSamplesPerSegment; ++s) {
            const float t = static_cast<float>(s) / kSamplesPerSegment;
            const cv::Point2f cur = catmullRom(p0, p1, p2, p3, t);
            const float alpha = static_cast<float>(i - start + t) /
                                std::max(1, n - start - 1);
            const cv::Scalar blended(color[0] * alpha,
                                     color[1] * alpha,
                                     color[2] * alpha);
            cv::line(img,
                     cv::Point(cvRound(prev.x), cvRound(prev.y)),
                     cv::Point(cvRound(cur.x), cvRound(cur.y)),
                     blended, thickness, cv::LINE_AA);
            prev = cur;
        }
    }
}

void drawPredictedTrail(cv::Mat& img,
                        const std::deque<cv::Point2f>& trail,
                        const cv::Point2f& velocity_px_s,
                        cv::Scalar history_color,
                        int max_len,
                        float prediction_horizon_s) {
    drawSmoothTrail(img, trail, history_color, max_len, 2);

    if (trail.size() < 2) return;

    const float speed = std::sqrt(velocity_px_s.x * velocity_px_s.x +
                                  velocity_px_s.y * velocity_px_s.y);
    if (speed < 1.0f) return;

    constexpr int kPredictionSegments = 16;
    const float dt = prediction_horizon_s / kPredictionSegments;

    std::deque<cv::Point2f> predicted;
    predicted.push_back(trail.back());
    cv::Point2f predicted_point = trail.back();
    for (int i = 0; i < kPredictionSegments; ++i) {
        predicted_point += velocity_px_s * dt;
        predicted.push_back(predicted_point);
    }

    const cv::Scalar prediction_color(0, 255, 255);
    drawSmoothTrail(img, predicted, prediction_color, static_cast<int>(predicted.size()), 4);
    cv::circle(img,
               cv::Point(cvRound(predicted.back().x), cvRound(predicted.back().y)),
               4, prediction_color, cv::FILLED, cv::LINE_AA);
}

}  // namespace tracking_app
