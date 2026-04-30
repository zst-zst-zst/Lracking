#include "target_tracker.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

#include <opencv2/core.hpp>

namespace solve {

// ─── 轨迹 Track 实现 ─────────────────────────────────────────────

/**
 * 构造函数：初始化轨迹 Track
 * @param id 轨迹 ID
 * @param bbox 初始检测框
 * @param confidence 初始置信度
 * @param timestamp 初始时间戳
 */
Track::Track(int id, const cv::Rect2f& bbox, float confidence, int64_t timestamp)
    : id_(id), last_confidence_(confidence), last_timestamp_(timestamp) {
    // 初始化卡尔曼滤波器
    initKalman(bbox);
    // 添加初始轨迹点
    addTrajectoryPoint(confidence, true, timestamp);
}

/**
 * 初始化卡尔曼滤波器
 * @param bbox 初始检测框
 */
void Track::initKalman(const cv::Rect2f& bbox) {
    // 状态向量：[cx, cy, vx, vy, w, h]  (6维)
    // 测量向量：[cx, cy, w, h]            (4维)
    kf_ = cv::KalmanFilter(6, 4, 0, CV_32F);

    // 状态转移矩阵 (匀速运动模型)
    // cx' = cx + vx, cy' = cy + vy, vx' = vx, vy' = vy, w' = w, h' = h
    kf_.transitionMatrix = cv::Mat::eye(6, 6, CV_32F);
    kf_.transitionMatrix.at<float>(0, 2) = 1.0f;  // cx += vx
    kf_.transitionMatrix.at<float>(1, 3) = 1.0f;  // cy += vy

    // 测量矩阵：从状态 [cx,cy,vx,vy,w,h] 中观测 [cx,cy,w,h]
    kf_.measurementMatrix = cv::Mat::zeros(4, 6, CV_32F);
    kf_.measurementMatrix.at<float>(0, 0) = 1.0f;  // cx
    kf_.measurementMatrix.at<float>(1, 1) = 1.0f;  // cy
    kf_.measurementMatrix.at<float>(2, 4) = 1.0f;  // w
    kf_.measurementMatrix.at<float>(3, 5) = 1.0f;  // h

    // 过程噪声协方差矩阵 (模型预测的不确定性)
    kf_.processNoiseCov = cv::Mat::eye(6, 6, CV_32F);
    float pn_pos = cfg_.process_noise_pos;
    float pn_vel = cfg_.process_noise_vel;
    float pn_sz = cfg_.process_noise_size;
    kf_.processNoiseCov.at<float>(0, 0) = pn_pos * pn_pos;  // 位置 x
    kf_.processNoiseCov.at<float>(1, 1) = pn_pos * pn_pos;  // 位置 y
    kf_.processNoiseCov.at<float>(2, 2) = pn_vel * pn_vel;  // 速度 vx
    kf_.processNoiseCov.at<float>(3, 3) = pn_vel * pn_vel;  // 速度 vy
    kf_.processNoiseCov.at<float>(4, 4) = pn_sz * pn_sz;    // 尺寸 w
    kf_.processNoiseCov.at<float>(5, 5) = pn_sz * pn_sz;    // 尺寸 h

    // 测量噪声协方差矩阵 (检测结果的不确定性)
    float mn = cfg_.measurement_noise;
    kf_.measurementNoiseCov = cv::Mat::eye(4, 4, CV_32F) * (mn * mn);

    // 初始误差协方差 (速度初始不确定性很高)
    kf_.errorCovPost = cv::Mat::eye(6, 6, CV_32F);
    kf_.errorCovPost.at<float>(0, 0) = 10.0f;   // 位置较确定
    kf_.errorCovPost.at<float>(1, 1) = 10.0f;
    kf_.errorCovPost.at<float>(2, 2) = 100.0f;  // 速度完全未知
    kf_.errorCovPost.at<float>(3, 3) = 100.0f;
    kf_.errorCovPost.at<float>(4, 4) = 10.0f;   // 尺寸较确定
    kf_.errorCovPost.at<float>(5, 5) = 10.0f;

    // 初始状态
    float cx = bbox.x + bbox.width / 2.0f;
    float cy = bbox.y + bbox.height / 2.0f;
    kf_.statePost.at<float>(0) = cx;           // 中心 x
    kf_.statePost.at<float>(1) = cy;           // 中心 y
    kf_.statePost.at<float>(2) = 0.0f;         // 初始速度 vx = 0
    kf_.statePost.at<float>(3) = 0.0f;         // 初始速度 vy = 0
    kf_.statePost.at<float>(4) = bbox.width;   // 宽度
    kf_.statePost.at<float>(5) = bbox.height;  // 高度
}

/**
 * 预测下一帧状态
 */
void Track::predict() {
    kf_.predict();
    age_++;
    time_since_update_++;
}

/**
 * 更新轨迹状态
 * @param bbox 检测框
 * @param confidence 置信度
 * @param timestamp 时间戳
 */
void Track::update(const cv::Rect2f& bbox, float confidence, int64_t timestamp) {
    // 计算帧间隔 (ms)
    if (last_timestamp_ > 0 && timestamp > last_timestamp_) {
        last_dt_ms_ = static_cast<float>(timestamp - last_timestamp_);
    }

    float cx = bbox.x + bbox.width / 2.0f;
    float cy = bbox.y + bbox.height / 2.0f;

    cv::Mat measurement = (cv::Mat_<float>(4, 1) << cx, cy, bbox.width, bbox.height);

    // ── NIS 发散检测 (借鉴 sp_vision 的卡方检验) ──
    // 计算新息 (innovation = measurement - H*x_prior)
    cv::Mat innovation = measurement - kf_.measurementMatrix * kf_.statePre;
    updateNIS(innovation);

    kf_.correct(measurement);

    last_confidence_ = confidence;
    last_timestamp_ = timestamp;
    time_since_update_ = 0;
    hit_streak_++;
    frame_count_++;

    // 状态转换
    if (state_ == TrackState::TENTATIVE && hit_streak_ >= cfg_.hits_to_confirm) {
        state_ = TrackState::CONFIRMED;  // 暂定 → 确认
    }
    if (state_ == TrackState::LOST) {
        state_ = TrackState::CONFIRMED;  // 丢失后重新匹配 → 确认 (恢复)
        hit_streak_ = 1;
    }

    addTrajectoryPoint(confidence, true, timestamp);
}

/**
 * 标记轨迹丢失
 */
void Track::markMissed() {
    hit_streak_ = 0;

    if (state_ == TrackState::TENTATIVE && time_since_update_ > cfg_.tentative_max_age) {
        state_ = TrackState::LOST;
    }
    if (state_ == TrackState::CONFIRMED && time_since_update_ > cfg_.max_age) {
        state_ = TrackState::LOST;
    }

    // 将预测点加入轨迹历史
    if (state_ != TrackState::LOST) {
        addTrajectoryPoint(0.0f, false, last_timestamp_);
    }
}

/**
 * 获取预测检测框
 * @return 预测检测框
 */
cv::Rect2f Track::predictedBbox() const {
    float cx = kf_.statePre.at<float>(0);
    float cy = kf_.statePre.at<float>(1);
    float w = std::max(kf_.statePre.at<float>(4), 1.0f);
    float h = std::max(kf_.statePre.at<float>(5), 1.0f);
    return cv::Rect2f(cx - w / 2.0f, cy - h / 2.0f, w, h);
}

/**
 * 获取预测中心点
 * @return 预测中心点
 */
cv::Point2f Track::predictedCenter() const {
    return cv::Point2f(kf_.statePre.at<float>(0), kf_.statePre.at<float>(1));
}

/**
 * 获取速度
 * @return 速度
 */
cv::Point2f Track::velocity() const {
    return cv::Point2f(kf_.statePost.at<float>(2), kf_.statePost.at<float>(3));
}

cv::Point2f Track::velocity_px_s() const {
    // Kalman 状态中的速度是 px/frame, 转换为 px/s
    float dt_s = last_dt_ms_ / 1000.0f;
    if (dt_s < 1e-6f) dt_s = 0.01f;  // fallback 10ms
    return cv::Point2f(kf_.statePost.at<float>(2) / dt_s,
                       kf_.statePost.at<float>(3) / dt_s);
}

bool Track::diverged() const {
    if (!cfg_.nis_check) return false;
    if (static_cast<int>(nis_failures_.size()) < cfg_.nis_window) return false;
    int fails = std::accumulate(nis_failures_.begin(), nis_failures_.end(), 0);
    float ratio = static_cast<float>(fails) / static_cast<float>(nis_failures_.size());
    return ratio >= cfg_.nis_fail_ratio;
}

void Track::updateNIS(const cv::Mat& innovation) {
    if (!cfg_.nis_check) return;
    // NIS = innovation^T * S^{-1} * innovation, where S = H*P*H^T + R
    cv::Mat S = kf_.measurementMatrix * kf_.errorCovPre * kf_.measurementMatrix.t()
                + kf_.measurementNoiseCov;
    cv::Mat S_inv;
    cv::invert(S, S_inv, cv::DECOMP_LU);
    cv::Mat nis_mat = innovation.t() * S_inv * innovation;
    float nis = nis_mat.at<float>(0, 0);
    nis_failures_.push_back(nis > cfg_.nis_threshold ? 1 : 0);
    while (static_cast<int>(nis_failures_.size()) > cfg_.nis_window) {
        nis_failures_.pop_front();
    }
}

/**
 * 添加轨迹点
 * @param conf 置信度
 * @param detected 是否检测到
 * @param timestamp 时间戳
 */
void Track::addTrajectoryPoint(float conf, bool detected, int64_t timestamp) {
    TrajectoryPoint pt;
    pt.timestamp = timestamp;
    pt.frame_id = frame_count_;
    pt.x = kf_.statePost.at<float>(0);
    pt.y = kf_.statePost.at<float>(1);
    pt.vx = kf_.statePost.at<float>(2);
    pt.vy = kf_.statePost.at<float>(3);
    pt.w = kf_.statePost.at<float>(4);
    pt.h = kf_.statePost.at<float>(5);
    pt.confidence = conf;
    pt.detected = detected;
    trajectory_.push_back(pt);

    // 限制轨迹长度，防止内存溢出
    while (static_cast<int>(trajectory_.size()) > cfg_.trajectory_max_length) {
        trajectory_.pop_front();
    }
}

// ─── 多目标跟踪器实现 ───────────────────────────────────────────

/**
 * 构造函数：初始化多目标跟踪器
 */
TargetTracker::TargetTracker() = default;

/**
 * 析构函数：释放资源
 */
TargetTracker::~TargetTracker() = default;

/**
 * 加载配置文件
 * @param config_path 配置文件路径
 * @return 是否加载成功
 */
bool TargetTracker::loadConfig(const std::string& config_path) {
    cv::FileStorage fs(config_path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    // 从 "tracker" 节点读取参数, 没有则用默认值
    cv::FileNode tn = fs["tracker"];
    if (tn.empty()) return true;  // 无跟踪器配置, 使用默认值

    if (!tn["max_distance"].empty()) tn["max_distance"] >> cfg_.max_distance;
    if (!tn["min_iou"].empty()) tn["min_iou"] >> cfg_.min_iou;
    if (!tn["hits_to_confirm"].empty()) tn["hits_to_confirm"] >> cfg_.hits_to_confirm;
    if (!tn["max_age"].empty()) tn["max_age"] >> cfg_.max_age;
    if (!tn["tentative_max_age"].empty()) tn["tentative_max_age"] >> cfg_.tentative_max_age;
    if (!tn["process_noise_pos"].empty()) tn["process_noise_pos"] >> cfg_.process_noise_pos;
    if (!tn["process_noise_vel"].empty()) tn["process_noise_vel"] >> cfg_.process_noise_vel;
    if (!tn["process_noise_size"].empty()) tn["process_noise_size"] >> cfg_.process_noise_size;
    if (!tn["measurement_noise"].empty()) tn["measurement_noise"] >> cfg_.measurement_noise;
    // NIS 发散检测
    if (!tn["nis_check"].empty()) {
        int v; tn["nis_check"] >> v;
        cfg_.nis_check = (v != 0);
    }
    if (!tn["nis_threshold"].empty()) tn["nis_threshold"] >> cfg_.nis_threshold;
    if (!tn["nis_window"].empty()) tn["nis_window"] >> cfg_.nis_window;
    if (!tn["nis_fail_ratio"].empty()) tn["nis_fail_ratio"] >> cfg_.nis_fail_ratio;

    if (!tn["log_trajectory"].empty()) {
        int v; tn["log_trajectory"] >> v;
        cfg_.log_trajectory = (v != 0);
    }
    if (!tn["trajectory_log_path"].empty()) tn["trajectory_log_path"] >> cfg_.trajectory_log_path;

    return true;
}

void TargetTracker::setConfig(const TrackerConfig& cfg) {
    cfg_ = cfg;
}

float TargetTracker::computeIoU(const cv::Rect2f& a, const cv::Rect2f& b) {
    float x1 = std::max(a.x, b.x);
    float y1 = std::max(a.y, b.y);
    float x2 = std::min(a.x + a.width, b.x + b.width);
    float y2 = std::min(a.y + a.height, b.y + b.height);

    float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    float area_a = a.width * a.height;
    float area_b = b.width * b.height;
    float union_area = area_a + area_b - inter;

    return (union_area > 0) ? (inter / union_area) : 0.0f;
}

void TargetTracker::associateDetections(
    const std::vector<std::pair<cv::Rect2f, float>>& detections,
    std::vector<int>& track_to_det,
    std::vector<int>& det_to_track) {

    int num_tracks = static_cast<int>(tracks_.size());
    int num_dets = static_cast<int>(detections.size());

    track_to_det.assign(num_tracks, -1);
    det_to_track.assign(num_dets, -1);

    if (num_tracks == 0 || num_dets == 0) return;

    // 构建代价矩阵: 中心距离 + (1 - IoU), 越小越好
    std::vector<std::vector<float>> cost(num_tracks, std::vector<float>(num_dets, 1e9f));

    for (int t = 0; t < num_tracks; ++t) {
        cv::Point2f tc = tracks_[t]->predictedCenter();
        cv::Rect2f tb = tracks_[t]->predictedBbox();

        for (int d = 0; d < num_dets; ++d) {
            const auto& det_bbox = detections[d].first;
            float det_cx = det_bbox.x + det_bbox.width / 2.0f;
            float det_cy = det_bbox.y + det_bbox.height / 2.0f;

            float dist = std::sqrt((tc.x - det_cx) * (tc.x - det_cx) +
                                   (tc.y - det_cy) * (tc.y - det_cy));
            float iou = computeIoU(tb, det_bbox);

            // 门限: 距离太远且无重叠 → 不匹配
            if (dist > cfg_.max_distance && iou < cfg_.min_iou) {
                continue;
            }

            // 代价: 距离主导, IoU 作为奖励
            cost[t][d] = dist - iou * 50.0f;
        }
    }

    // 贪心匹配 (目标数量少, 通常 1-3 个, 贪心就够了)
    // 迭代: 找全局最小代价, 分配, 重复
    std::vector<bool> track_used(num_tracks, false);
    std::vector<bool> det_used(num_dets, false);

    for (int iter = 0; iter < std::min(num_tracks, num_dets); ++iter) {
        float best_cost = 1e9f;
        int best_t = -1, best_d = -1;

        for (int t = 0; t < num_tracks; ++t) {
            if (track_used[t]) continue;
            for (int d = 0; d < num_dets; ++d) {
                if (det_used[d]) continue;
                if (cost[t][d] < best_cost) {
                    best_cost = cost[t][d];
                    best_t = t;
                    best_d = d;
                }
            }
        }

        if (best_t < 0 || best_cost >= 1e9f) break;

        track_to_det[best_t] = best_d;
        det_to_track[best_d] = best_t;
        track_used[best_t] = true;
        det_used[best_d] = true;
    }
}

std::vector<TargetTracker::TrackedTarget> TargetTracker::update(
    const std::vector<std::pair<cv::Rect2f, float>>& detections,
    int64_t timestamp) {

    // 1. 预测所有现有轨迹的下一帧状态
    for (auto& track : tracks_) {
        track->predict();
    }

    // 2. 关联匹配: 检测结果 ↔ 现有轨迹
    std::vector<int> track_to_det, det_to_track;
    associateDetections(detections, track_to_det, det_to_track);

    // 3. 更新已匹配轨迹, 标记未匹配轨迹
    for (int t = 0; t < static_cast<int>(tracks_.size()); ++t) {
        if (track_to_det[t] >= 0) {
            int d = track_to_det[t];
            tracks_[t]->update(detections[d].first, detections[d].second, timestamp);
        } else {
            tracks_[t]->markMissed();
        }
    }

    // 4. 为未匹配的检测创建新轨迹
    for (int d = 0; d < static_cast<int>(detections.size()); ++d) {
        if (det_to_track[d] < 0) {
            auto track = std::make_unique<Track>(
                next_id_++, detections[d].first, detections[d].second, timestamp);
            track->setConfig(cfg_);
            tracks_.push_back(std::move(track));
        }
    }

    // 5. 删除已丢失/发散的轨迹
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                        [](const std::unique_ptr<Track>& t) {
                            if (t->state() == TrackState::LOST) return true;
                            // NIS 发散检测 (借鉴 sp_vision)
                            if (t->diverged()) {
                                std::cout << "[Tracker] Track " << t->id()
                                          << " diverged (NIS), removing\n";
                                return true;
                            }
                            return false;
                        }),
        tracks_.end());

    // 6. 构建输出并找到主目标
    std::vector<TrackedTarget> result;
    float best_score = -1.0f;
    primary_track_id_ = -1;

    for (const auto& track : tracks_) {
        TrackedTarget tt;
        tt.track_id = track->id();
        tt.bbox = track->predictedBbox();
        tt.center = track->predictedCenter();
        tt.velocity = track->velocity();
        tt.velocity_px_s = track->velocity_px_s();
        tt.confidence = track->lastConfidence();
        tt.detected = (track->timeSinceUpdate() == 0);
        tt.state = track->state();
        result.push_back(tt);

        // 主目标: 已确认轨迹中置信度最高的
        if (track->state() == TrackState::CONFIRMED) {
            float score = track->lastConfidence() + track->hitStreak() * 0.01f;
            if (score > best_score) {
                best_score = score;
                primary_track_id_ = track->id();
            }
        }
    }

    return result;
}

bool TargetTracker::hasPrimaryTarget() const {
    return primary_track_id_ >= 0;
}

TargetTracker::TrackedTarget TargetTracker::primaryTarget() const {
    TrackedTarget tt;
    for (const auto& track : tracks_) {
        if (track->id() == primary_track_id_) {
            tt.track_id = track->id();
            tt.bbox = track->predictedBbox();
            tt.center = track->predictedCenter();
            tt.velocity = track->velocity();
            tt.velocity_px_s = track->velocity_px_s();
            tt.confidence = track->lastConfidence();
            tt.detected = (track->timeSinceUpdate() == 0);
            tt.state = track->state();
            return tt;
        }
    }
    return tt;  // empty
}

void TargetTracker::saveTrajectoryLog() const {
    if (!cfg_.log_trajectory || cfg_.trajectory_log_path.empty()) return;

    cv::FileStorage fs(cfg_.trajectory_log_path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "轨迹日志打开失败: " << cfg_.trajectory_log_path << "\n";
        return;
    }

    fs << "num_tracks" << static_cast<int>(tracks_.size());
    fs << "tracks" << "[";

    for (const auto& track : tracks_) {
        fs << "{";
        fs << "id" << track->id();
        fs << "state" << static_cast<int>(track->state());
        fs << "trajectory" << "[";
        for (const auto& pt : track->trajectory()) {
            fs << "{";
            fs << "ts" << static_cast<double>(pt.timestamp);
            fs << "frame" << pt.frame_id;
            fs << "x" << pt.x;
            fs << "y" << pt.y;
            fs << "vx" << pt.vx;
            fs << "vy" << pt.vy;
            fs << "w" << pt.w;
            fs << "h" << pt.h;
            fs << "conf" << pt.confidence;
            fs << "det" << (pt.detected ? 1 : 0);
            fs << "}";
        }
        fs << "]";
        fs << "}";
    }
    fs << "]";

    std::cout << "轨迹日志已保存: " << cfg_.trajectory_log_path
              << " (" << tracks_.size() << " 条轨迹)\n";
}

void TargetTracker::reset() {
    tracks_.clear();
    next_id_ = 1;
    primary_track_id_ = -1;
}

}  // namespace solve
