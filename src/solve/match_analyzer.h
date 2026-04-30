#ifndef SOLVE_MATCH_ANALYZER_H
#define SOLVE_MATCH_ANALYZER_H

#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace solve {

// ─── 枚举定义 ────────────────────────────────────────────────────────

// 飞行阶段: 根据无人机的位置和运动状态判断
enum class FlightPhase {
    IDLE,       // 停机坡上, 未起飞或已关机
    TAKEOFF,    // 从停机坡起飞中 (bbox增大)
    CRUISING,   // 巡航中 (bbox稳定, 速度较快)
    ATTACKING,  // 接近前哨, 悬停/慢速 — 最佳照射窗口
    RETURNING,  // 返航中
    LANDING     // 降落中 (bbox缩小)
};

// 瞄准策略: 根据稳定性分级自动选择
// 注意: 激光上电即常亮, 不存在"开不开火"的概念
// 策略影响的是瞄准质量评估 (isOnTarget), 而非是否发射
enum class AimStrategy {
    PRECISE,    // 第1级: 精确跟踪, 激光直接对准目标中心
    TRACK,      // 第2级: 正常跟踪, 轻度预测补偿
    PREDICT,    // 第3级: 重度平滑 + 运动预测
    OPPORTUNE   // 第4级: 目标不稳定时认为照射无效
};

const char* flightPhaseStr(FlightPhase p);
const char* aimStrategyStr(AimStrategy s);

// ─── 稳定性分级配置 ──────────────────────────────────────────────────
// 每级对应不同的抖动容忍度和平滑参数
struct StabilityTierParams {
    float shake_threshold_px = 6.0f;    // 抖动阈值 (像素), 超过则升级
    float smooth_alpha = 0.5f;          // EMA 平滑系数 (越小越平滑)
    int hold_frames = 5;                // 丢失检测后保持目标的帧数
    float min_continuous_s = 0.08f;     // 最小连续照射时间 (秒)
    AimStrategy strategy = AimStrategy::TRACK; // 对应的瞄准策略
};

// ─── 比赛分析器配置 ──────────────────────────────────────────────────

struct MatchAnalyzerConfig {
    // ── 场地几何 ──
    float field_length = 28.0f;             // 场地长度 (米)
    float field_width = 15.0f;              // 场地宽度 (米)
    float camera_height = 3.35f;            // 相机安装高度 (米)
    float drone_flight_height_min = 2.0f;   // 无人机最低飞行高度 (米)
    float drone_flight_height_max = 2.4f;   // 无人机最高飞行高度 (米)

    // ── P值规则 ──
    // 增长: 连续照射每0.1s, P += n (n为连续判定次数)
    // 累计: P = n(n+1)/2
    // 第1次锁定 P0=50: 需 n≥10 → ≥1.0s
    // 第2次锁定 P0=100: 需 n≥14 → ≥1.4s
    // 第3次锁定 P0=100: 需 n≥14 → ≥1.4s (第3次后停止发光)
    int p0_first = 50;                      // 第1次锁定阈值
    int p0_second = 100;                    // 第2次锁定阈值
    int p0_third = 100;                     // 第3次锁定阈值
    float p_decay_rate = 0.5f;              // 未照射时P值衰减速率 (每秒)
    float p_judge_interval = 0.1f;          // 判定间隔 (秒)
    float lock_duration = 45.0f;            // 锁定持续时间 (秒)
    int max_locks = 3;                      // 每局最大锁定次数

    // ── 稳定性分级 (1-4级) ──
    int preset_tier = 2;                    // 预设分级 (启动时使用)
    StabilityTierParams tiers[4];           // 4级稳定性参数

    // ── 自适应校准 ──
    float calibration_window_s = 3.0f;      // 校准窗口 (秒)
    float recalibrate_interval_s = 10.0f;   // 重新校准间隔 (秒)
    float tier_change_cooldown_s = 5.0f;    // 分级变更冷却时间 (秒)

    // ── 飞行阶段判断阈值 ──
    float phase_idle_max_speed_px = 2.0f;           // 停机最大速度 (像素/帧)
    float phase_cruise_min_speed_px = 5.0f;         // 巡航最小速度
    float phase_attack_max_speed_px = 8.0f;         // 攻击最大速度
    float phase_attack_x_threshold = 0.6f;          // 攻击区域 x 坐标阈值 (归一化)
    float phase_takeoff_bbox_grow_rate = 0.05f;     // 起飞判定 bbox 增长率

    // ── 日志 ──
    bool log_enabled = true;                // 是否启用日志
    std::string log_file = "match_log.yaml"; // 日志文件路径
    int log_interval_ms = 100;              // 日志记录间隔 (毫秒)
};

bool loadMatchAnalyzerConfig(const std::string& path, MatchAnalyzerConfig* out);

// ─── 跟踪样本 ────────────────────────────────────────────────────────
// 每帧记录一个样本, 用于稳定性分析和P值计算
struct TrackSample {
    int64_t timestamp_ms = 0;           // 时间戳 (毫秒)
    cv::Point2f center;                 // 激光模块中心 (全帧坐标)
    cv::Point2f drone_center;           // 无人机中心 (全帧坐标)
    float drone_bbox_h = 0.0f;          // 无人机 bbox 高度 (像素)
    float confidence = 0.0f;            // 检测置信度
    bool detected = false;              // 本帧是否检测到激光模块
};

// ─── 日志条目 ───────────────────────────────────────────────────────

struct LogEntry {
    int64_t timestamp_ms = 0;                               // 时间戳
    FlightPhase phase = FlightPhase::IDLE;                  // 飞行阶段
    int stability_tier = 2;                                 // 当前稳定性分级
    float measured_shake_px = 0.0f;                         // 测量抖动 (像素)
    float estimated_p = 0.0f;                               // 估计P值
    int lock_count = 0;                                     // 已锁定次数
    float target_x = 0.0f;                                  // 目标x坐标
    float target_y = 0.0f;                                  // 目标y坐标
    float confidence = 0.0f;                                // 置信度
    bool target_detected = false;                           // 是否检测到
    bool is_on_target = false;                              // 是否正在有效照射
    float continuous_illuminate_s = 0.0f;                   // 连续照射时间 (秒)
    AimStrategy current_strategy = AimStrategy::TRACK;      // 当前策略
};

// ─── 比赛分析器 ──────────────────────────────────────────────────────
// 核心功能:
//   1. 飞行阶段判断 (停机/起飞/巡航/攻击/返航/降落)
//   2. 稳定性分级自适应 (1-4级)
//   3. P值跟踪与锁定判定
//   4. 瞄准策略选择 (激光上电即常亮, 不存在"开不开火")
//   5. 比赛日志记录

class MatchAnalyzer {
public:
    MatchAnalyzer();
    ~MatchAnalyzer();

    bool loadConfig(const std::string& path);     // 从 YAML 加载配置
    void setConfig(const MatchAnalyzerConfig& cfg); // 直接设置配置

    // 每帧调用, 输入最新检测结果
    // drone_center / drone_bbox_h: 第1层无人机检测结果
    // laser_center / laser_conf: 第2层激光模块检测结果 (未检测到时无效)
    void update(int64_t timestamp_ms,
                bool drone_detected,
                cv::Point2f drone_center, float drone_bbox_h,
                bool laser_detected,
                cv::Point2f laser_center, float laser_conf,
                int img_w, int img_h);

    // ── 获取器 ──
    FlightPhase currentPhase() const;                    // 当前飞行阶段
    int currentTier() const;                             // 当前稳定性分级
    const StabilityTierParams& currentTierParams() const; // 当前分级参数
    AimStrategy currentStrategy() const;                 // 当前瞄准策略
    float measuredShakePx() const;                       // 测量抖动度 (像素)
    float estimatedP() const;                            // 估计P值
    int lockCount() const;                               // 已锁定次数
    float continuousIlluminateS() const;                 // 连续照射时间 (秒)

    // 平滑后的目标位置 (根据分级的EMA平滑)
    cv::Point2f smoothedTarget() const;
    bool hasSmoothedTarget() const;

    // 瞄准质量评估: 激光是否正在有效照射目标
    // 注意: 激光上电即常亮, 此函数评估的是瞄准质量,
    // 而非控制是否发射。返回true表示当前可能正在累P值。
    bool isOnTarget() const;

    // 结束并关闭日志
    void finalize();

private:
    void updatePhase(int img_w, int img_h);              // 更新飞行阶段
    void updateStability();                               // 更新稳定性测量
    void updatePValue(int64_t timestamp_ms, bool illuminating); // 更新P值
    void recalibrateTier();                               // 自适应重新校准分级
    void writeLog(int64_t timestamp_ms);                  // 写日志

    MatchAnalyzerConfig cfg_;                             // 配置
    int current_tier_ = 2;                                // 当前稳定性分级
    FlightPhase phase_ = FlightPhase::IDLE;               // 当前飞行阶段

    // 跟踪历史
    std::deque<TrackSample> history_;                     // 最近 N 帧样本
    static constexpr int kMaxHistory = 300;               // ~10秒 @ 30fps

    // 平滑目标
    cv::Point2f smoothed_target_{0, 0};                   // EMA 平滑后的目标位置
    bool has_smoothed_ = false;                           // 是否有有效的平滑目标
    int frames_since_detection_ = 0;                      // 距上次检测到的帧数

    // 稳定性测量
    float measured_shake_px_ = 0.0f;                      // 测量抖动度 (90分位)
    std::deque<float> shake_samples_;                     // 抖动样本队列
    int64_t last_recalibrate_ms_ = 0;                     // 上次校准时间
    int64_t last_tier_change_ms_ = 0;                     // 上次分级变更时间

    // P值跟踪
    float estimated_p_ = 0.0f;                            // 当前估计P值
    int lock_count_ = 0;                                  // 已锁定次数
    float continuous_illuminate_s_ = 0.0f;                // 当前连续照射时间
    int64_t illuminate_start_ms_ = 0;                     // 本次照射开始时间
    bool was_illuminating_ = false;                       // 上一帧是否在照射

    // 日志
    std::ofstream log_stream_;                            // 日志文件流
    int64_t last_log_ms_ = 0;                             // 上次写日志时间
    int log_entry_count_ = 0;                             // 日志条目计数
};

}  // namespace solve

#endif  // SOLVE_MATCH_ANALYZER_H
