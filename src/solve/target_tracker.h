#ifndef SOLVE_TARGET_TRACKER_H
#define SOLVE_TARGET_TRACKER_H

#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

namespace solve {

// ─── 轨迹状态机 ───────────────────────────────────────────────────
// TENTATIVE(暂定): 新建轨迹, 尚未连续命中足够帧数
// CONFIRMED(确认): 连续命中 ≥ hits_to_confirm 帧, 可信轨迹
// LOST(丢失): 超过 max_age 帧未更新, 即将删除
enum class TrackState { TENTATIVE, CONFIRMED, LOST };

// ─── 跟踪器配置 ───────────────────────────────────────────────────
struct TrackerConfig {
    // ── 关联匹配参数 ──
    float max_distance = 100.0f;   // 关联最大中心距离 (像素), 超过不匹配
    float min_iou = 0.1f;          // 关联最小 IoU, 低于此值且距离也超限→不匹配

    // ── 轨迹生命周期 ──
    int hits_to_confirm = 3;       // 连续命中此帧数 → TENTATIVE升级为CONFIRMED
    int max_age = 15;              // 已确认轨迹连续丢失此帧数 → 变为LOST并删除
    int tentative_max_age = 3;     // 暂定轨迹连续丢失此帧数 → 变为LOST并删除

    // ── Kalman 滤波器噪声参数 ──
    // 过程噪声: 模型预测的不确定性 (越大越信任测量, 越小越信任预测)
    // 测量噪声: 检测结果的不确定性 (越大越信任预测, 越小越信任测量)
    float process_noise_pos = 4.0f;     // 位置过程噪声标准差 (像素)
    float process_noise_vel = 10.0f;    // 速度过程噪声标准差 (像素/帧)
    float process_noise_size = 2.0f;    // 尺寸过程噪声标准差 (像素)
    float measurement_noise = 4.0f;     // 测量噪声标准差 (像素)

    // ── NIS 发散检测 (借鉴 sp_vision 的卡方检验) ──
    bool nis_check = true;              // 启用 NIS 发散检测
    float nis_threshold = 9.488f;       // 卡方检验阈值 (自由度4, 95%置信度)
    int nis_window = 10;                // 滑窗大小
    float nis_fail_ratio = 0.4f;        // 失败率超过此值 → 发散

    // ── 轨迹日志 ──
    bool log_trajectory = true;                          // 程序退出时保存轨迹日志
    std::string trajectory_log_path = "trajectory_log.yaml"; // 日志文件路径
    int trajectory_max_length = 10000;  // 内存中保留的最大轨迹点数
};

// ─── 轨迹点 ───────────────────────────────────────────────────────
// 每帧记录一个轨迹点, 用于赛后复盘分析
struct TrajectoryPoint {
    int64_t timestamp = 0;          // 时间戳
    int frame_id = 0;               // 帧序号
    float x = 0, y = 0;            // 中心位置 (全帧坐标)
    float w = 0, h = 0;            // bbox 尺寸
    float vx = 0, vy = 0;          // Kalman 速度估计 (像素/帧)
    float confidence = 0;           // 检测置信度 (预测时为0)
    bool detected = false;          // true=来自检测, false=Kalman预测
};

// ─── 单个跟踪目标 ─────────────────────────────────────────────────
// 每个 Track 维护一个 Kalman 滤波器和完整轨迹历史
class Track {
public:
    Track(int id, const cv::Rect2f& bbox, float confidence, int64_t timestamp);

    // 预测下一帧状态 (在关联匹配之前调用)
    void predict();

    // 用匹配到的检测结果更新 Kalman 滤波器
    void update(const cv::Rect2f& bbox, float confidence, int64_t timestamp);

    // 标记本帧未匹配到检测 (丢失一帧)
    void markMissed();

    // 获取器
    int id() const { return id_; }                    // 轨迹唯一ID
    TrackState state() const { return state_; }       // 当前状态
    cv::Rect2f predictedBbox() const;                 // Kalman 预测的 bbox
    cv::Point2f predictedCenter() const;              // Kalman 预测的中心
    cv::Point2f velocity() const;                     // Kalman 估计的速度 (px/frame)
    cv::Point2f velocity_px_s() const;                 // Kalman 估计的速度 (px/s)
    bool diverged() const;                             // NIS 发散检测
    float lastConfidence() const { return last_confidence_; } // 上次检测置信度
    int age() const { return age_; }                  // 轨迹存活帧数
    int hitStreak() const { return hit_streak_; }     // 当前连续命中帧数
    int timeSinceUpdate() const { return time_since_update_; } // 距上次更新帧数
    const std::deque<TrajectoryPoint>& trajectory() const { return trajectory_; }

    void setConfig(const TrackerConfig& cfg) { cfg_ = cfg; }
    float lastDtMs() const { return last_dt_ms_; }

private:
    int id_;                                          // 轨迹唯一ID
    TrackState state_ = TrackState::TENTATIVE;        // 当前状态
    cv::KalmanFilter kf_;                             // Kalman 滤波器
    float last_confidence_ = 0;                       // 上次检测置信度
    int age_ = 0;                                     // 轨迹存活总帧数
    int hit_streak_ = 0;                              // 当前连续命中帧数
    int time_since_update_ = 0;                       // 距上次成功更新的帧数
    int64_t last_timestamp_ = 0;                      // 上次更新时间戳
    int frame_count_ = 0;                             // 已处理帧计数
    TrackerConfig cfg_;                               // 配置
    std::deque<TrajectoryPoint> trajectory_;           // 完整轨迹历史
    std::deque<int> nis_failures_;                    // NIS 检验滑窗 (1=fail, 0=pass)
    float last_dt_ms_ = 10.0f;                        // 上次帧间隔 (ms)

    void initKalman(const cv::Rect2f& bbox);          // 初始化 Kalman 滤波器
    void addTrajectoryPoint(float conf, bool detected, int64_t timestamp);
    void updateNIS(const cv::Mat& innovation);        // NIS 卡方检验
};

// ─── 多目标跟踪器 ─────────────────────────────────────────────────
// 类似 DeepSORT 的帧间关联跟踪器。
// 激光模块数量少 (通常1-3个), 用贪心匹配就够了。
// 核心价值:
//   1. 检测短暂丢失时用 Kalman 预测维持跟踪
//   2. 区分稳定目标 vs 干扰闪烁 (需连续 3 帧命中才确认)
//   3. 速度估计辅助运动补偿
//   4. 每局结束保存完整轨迹用于赛后复盘
class TargetTracker {
public:
    TargetTracker();
    ~TargetTracker();

    bool loadConfig(const std::string& config_path);  // 从 YAML 加载配置
    void setConfig(const TrackerConfig& cfg);          // 直接设置配置

    // 主更新接口: 输入检测结果, 输出跟踪结果
    // detections: (bbox, 置信度) 列表, 全帧坐标
    // 返回所有活跃轨迹的当前状态
    struct TrackedTarget {
        int track_id = -1;           // 轨迹唯一ID
        cv::Rect2f bbox;             // 当前 bbox (Kalman 预测/更新后)
        cv::Point2f center;          // 当前中心
        cv::Point2f velocity;        // Kalman 估计速度 (像素/帧)
        cv::Point2f velocity_px_s;   // Kalman 估计速度 (像素/秒), 用于延迟补偿
        float confidence = 0;        // 上次检测置信度
        bool detected = false;       // true=本帧有匹配检测, false=Kalman预测
        TrackState state = TrackState::LOST;
    };

    std::vector<TrackedTarget> update(
        const std::vector<std::pair<cv::Rect2f, float>>& detections,
        int64_t timestamp);

    // 获取最佳已确认目标 (如果有)
    bool hasPrimaryTarget() const;
    TrackedTarget primaryTarget() const;

    // 获取所有活跃轨迹
    const std::vector<std::unique_ptr<Track>>& tracks() const { return tracks_; }

    // 保存轨迹日志 (程序退出时调用)
    void saveTrajectoryLog() const;

    // 重置所有轨迹 (新一局比赛时调用)
    void reset();

private:
    TrackerConfig cfg_;                                // 配置
    std::vector<std::unique_ptr<Track>> tracks_;       // 所有活跃轨迹
    int next_id_ = 1;                                  // 下一个轨迹ID
    int primary_track_id_ = -1;                        // 当前主目标轨迹ID

    // 关联匹配: 计算代价矩阵并贪心匹配
    void associateDetections(
        const std::vector<std::pair<cv::Rect2f, float>>& detections,
        std::vector<int>& track_to_det,   // 轨迹索引 → 检测索引 (-1=未匹配)
        std::vector<int>& det_to_track);  // 检测索引 → 轨迹索引 (-1=未匹配)

    static float computeIoU(const cv::Rect2f& a, const cv::Rect2f& b);
};

}  // namespace solve

#endif  // SOLVE_TARGET_TRACKER_H
