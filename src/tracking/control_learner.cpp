#include "control_learner.h"

#include <filesystem>

#include <opencv2/core.hpp>

namespace tracking_app {

bool ControlLearner::load(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    if (!fs["ff_gain_yaw"].empty()) fs["ff_gain_yaw"] >> ff_gain_yaw;
    if (!fs["ff_gain_pitch"].empty()) fs["ff_gain_pitch"] >> ff_gain_pitch;
    if (!fs["session_count"].empty()) fs["session_count"] >> session_count;
    if (!fs["lifetime_tracking_frames"].empty()) fs["lifetime_tracking_frames"] >> lifetime_tracking_frames;
    if (!fs["lifetime_hit_frames"].empty()) fs["lifetime_hit_frames"] >> lifetime_hit_frames;
    if (!fs["ema_hit_ratio"].empty()) fs["ema_hit_ratio"] >> ema_hit_ratio;
    fs.release();
    return true;
}

void ControlLearner::save(const std::string& path) const {
    namespace fs = std::filesystem;
    fs::create_directories(fs::path(path).parent_path());
    cv::FileStorage out(path, cv::FileStorage::WRITE);
    if (!out.isOpened()) return;
    out << "ff_gain_yaw" << ff_gain_yaw;
    out << "ff_gain_pitch" << ff_gain_pitch;
    out << "session_count" << session_count;
    out << "lifetime_tracking_frames" << lifetime_tracking_frames;
    out << "lifetime_hit_frames" << lifetime_hit_frames;
    out << "ema_hit_ratio" << ema_hit_ratio;
    out.release();
}

}  // namespace tracking_app
