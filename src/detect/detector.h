#ifndef DETECT_DETECTOR_H
#define DETECT_DETECTOR_H

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace detect {

struct Detection {
    bool valid = false;
    int class_id = -1;
    std::string label;
    cv::Point2f center{};
    float radius = 0.0f;
    cv::Rect bbox{};
    float confidence = 0.0f;
};

class Detector {
public:
    virtual ~Detector() = default;
    virtual std::vector<Detection> detect(const cv::Mat& bgr, int64_t timestamp) = 0;
};

}  // namespace detect

#endif  // DETECT_DETECTOR_H
