#include "match_analyzer.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <numeric>

#include <opencv2/core.hpp>

namespace solve {

// ─── 字符串辅助函数 ─────────────────────────────────────────────────

const char* flightPhaseStr(FlightPhase p) {
    switch (p) {
        case FlightPhase::IDLE:      return "IDLE";
        case FlightPhase::TAKEOFF:   return "TAKEOFF";
        case FlightPhase::CRUISING:  return "CRUISING";
        case FlightPhase::ATTACKING: return "ATTACKING";
        case FlightPhase::RETURNING: return "RETURNING";
        case FlightPhase::LANDING:   return "LANDING";
    }
    return "UNKNOWN";
}

const char* aimStrategyStr(AimStrategy s) {
    switch (s) {
        case AimStrategy::PRECISE:   return "precise";
        case AimStrategy::TRACK:     return "track";
        case AimStrategy::PREDICT:   return "predict";
        case AimStrategy::OPPORTUNE: return "opportune";
    }
    return "unknown";
}

static AimStrategy parseStrategy(const std::string& s) {
    if (s == "precise")   return AimStrategy::PRECISE;
    if (s == "track")     return AimStrategy::TRACK;
    if (s == "predict")   return AimStrategy::PREDICT;
    if (s == "opportune") return AimStrategy::OPPORTUNE;
    return AimStrategy::TRACK;
}

// ─── 配置加载 ───────────────────────────────────────────────────────

static void loadTier(cv::FileStorage& fs, const std::string& prefix,
                     StabilityTierParams* out) {
    auto key = [&](const char* k) { return prefix + k; };
    if (!fs[key("shake_threshold_px")].empty())
        fs[key("shake_threshold_px")] >> out->shake_threshold_px;
    if (!fs[key("smooth_alpha")].empty())
        fs[key("smooth_alpha")] >> out->smooth_alpha;
    if (!fs[key("hold_frames")].empty())
        fs[key("hold_frames")] >> out->hold_frames;
    if (!fs[key("min_continuous_s")].empty())
        fs[key("min_continuous_s")] >> out->min_continuous_s;
    std::string strat;
    if (!fs[key("aim_strategy")].empty()) {
        fs[key("aim_strategy")] >> strat;
        out->strategy = parseStrategy(strat);
    }
}

bool loadMatchAnalyzerConfig(const std::string& path, MatchAnalyzerConfig* out) {
    if (!out) return false;
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "MatchAnalyzer: 配置文件打开失败: " << path << "\n";
        return false;
    }

    // 场地几何
    if (!fs["field_length"].empty()) fs["field_length"] >> out->field_length;
    if (!fs["field_width"].empty()) fs["field_width"] >> out->field_width;
    if (!fs["camera_height"].empty()) fs["camera_height"] >> out->camera_height;
    if (!fs["drone_flight_height_min"].empty()) fs["drone_flight_height_min"] >> out->drone_flight_height_min;
    if (!fs["drone_flight_height_max"].empty()) fs["drone_flight_height_max"] >> out->drone_flight_height_max;

    // P值规则
    if (!fs["p0_first"].empty()) fs["p0_first"] >> out->p0_first;
    if (!fs["p0_second"].empty()) fs["p0_second"] >> out->p0_second;
    if (!fs["p0_third"].empty()) fs["p0_third"] >> out->p0_third;
    if (!fs["p_decay_rate"].empty()) fs["p_decay_rate"] >> out->p_decay_rate;
    if (!fs["p_judge_interval"].empty()) fs["p_judge_interval"] >> out->p_judge_interval;
    if (!fs["lock_duration"].empty()) fs["lock_duration"] >> out->lock_duration;
    if (!fs["max_locks_per_match"].empty()) fs["max_locks_per_match"] >> out->max_locks;

    // 稳定性分级
    if (!fs["preset_stability_tier"].empty()) fs["preset_stability_tier"] >> out->preset_tier;
    out->preset_tier = std::clamp(out->preset_tier, 1, 4);

    // 加载前设置默认分级参数
    out->tiers[0] = {3.0f, 0.7f, 3, 0.08f, AimStrategy::PRECISE};
    out->tiers[1] = {6.0f, 0.5f, 5, 0.08f, AimStrategy::TRACK};
    out->tiers[2] = {12.0f, 0.3f, 8, 0.06f, AimStrategy::PREDICT};
    out->tiers[3] = {25.0f, 0.2f, 12, 0.05f, AimStrategy::OPPORTUNE};

    loadTier(fs, "tier1_", &out->tiers[0]);
    loadTier(fs, "tier2_", &out->tiers[1]);
    loadTier(fs, "tier3_", &out->tiers[2]);
    loadTier(fs, "tier4_", &out->tiers[3]);

    // 自适应校准
    if (!fs["calibration_window_s"].empty()) fs["calibration_window_s"] >> out->calibration_window_s;
    if (!fs["recalibrate_interval_s"].empty()) fs["recalibrate_interval_s"] >> out->recalibrate_interval_s;
    if (!fs["tier_change_cooldown_s"].empty()) fs["tier_change_cooldown_s"] >> out->tier_change_cooldown_s;

    // 飞行阶段判断阈值
    if (!fs["phase_idle_max_speed_px"].empty()) fs["phase_idle_max_speed_px"] >> out->phase_idle_max_speed_px;
    if (!fs["phase_cruise_min_speed_px"].empty()) fs["phase_cruise_min_speed_px"] >> out->phase_cruise_min_speed_px;
    if (!fs["phase_attack_max_speed_px"].empty()) fs["phase_attack_max_speed_px"] >> out->phase_attack_max_speed_px;
    if (!fs["phase_attack_x_threshold"].empty()) fs["phase_attack_x_threshold"] >> out->phase_attack_x_threshold;
    if (!fs["phase_takeoff_bbox_grow_rate"].empty()) fs["phase_takeoff_bbox_grow_rate"] >> out->phase_takeoff_bbox_grow_rate;

    // 日志
    if (!fs["log_enabled"].empty()) {
        int v = 0;
        fs["log_enabled"] >> v;
        out->log_enabled = (v != 0);
    }
    if (!fs["log_file"].empty()) fs["log_file"] >> out->log_file;
    if (!fs["log_interval_ms"].empty()) fs["log_interval_ms"] >> out->log_interval_ms;

    // 将相对路径转为绝对路径 (相对于配置文件目录)
    if (!out->log_file.empty()) {
        std::filesystem::path p(out->log_file);
        if (p.is_relative()) {
            out->log_file = (std::filesystem::path(path).parent_path() / p).string();
        }
    }

    return true;
}

// ─── 比赛分析器实现 ─────────────────────────────────────────────────

MatchAnalyzer::MatchAnalyzer() = default;
MatchAnalyzer::~MatchAnalyzer() { finalize(); }

bool MatchAnalyzer::loadConfig(const std::string& path) {
    if (!loadMatchAnalyzerConfig(path, &cfg_)) return false;
    current_tier_ = cfg_.preset_tier;
    if (cfg_.log_enabled && !cfg_.log_file.empty()) {
        log_stream_.open(cfg_.log_file, std::ios::out | std::ios::trunc);
        if (log_stream_.is_open()) {
            log_stream_ << "%YAML:1.0\n";
            log_stream_ << "# 比赛分析日志\n";
            log_stream_ << "preset_tier: " << cfg_.preset_tier << "\n";
            log_stream_ << "entries:\n";
            std::cout << "MatchAnalyzer: 日志输出到 " << cfg_.log_file << "\n";
        }
    }
    std::cout << "MatchAnalyzer: 预设分级=" << current_tier_
              << " 策略=" << aimStrategyStr(currentTierParams().strategy) << "\n";
    return true;
}

void MatchAnalyzer::setConfig(const MatchAnalyzerConfig& cfg) {
    cfg_ = cfg;
    current_tier_ = cfg_.preset_tier;
}

void MatchAnalyzer::update(int64_t timestamp_ms,
                           bool drone_detected,
                           cv::Point2f drone_center, float drone_bbox_h,
                           bool laser_detected,
                           cv::Point2f laser_center, float laser_conf,
                           int img_w, int img_h) {
    // 记录样本
    TrackSample sample;
    sample.timestamp_ms = timestamp_ms;
    sample.drone_center = drone_detected ? drone_center : cv::Point2f(-1, -1);
    sample.drone_bbox_h = drone_bbox_h;
    sample.center = laser_detected ? laser_center : cv::Point2f(-1, -1);
    sample.confidence = laser_conf;
    sample.detected = laser_detected;
    history_.push_back(sample);
    while (static_cast<int>(history_.size()) > kMaxHistory) {
        history_.pop_front();
    }

    // 更新EMA平滑目标
    if (laser_detected) {
        float alpha = currentTierParams().smooth_alpha;
        if (!has_smoothed_) {
            smoothed_target_ = laser_center;
            has_smoothed_ = true;
        } else {
            smoothed_target_.x = alpha * laser_center.x + (1.0f - alpha) * smoothed_target_.x;
            smoothed_target_.y = alpha * laser_center.y + (1.0f - alpha) * smoothed_target_.y;
        }
        frames_since_detection_ = 0;
    } else {
        frames_since_detection_++;
        if (frames_since_detection_ > currentTierParams().hold_frames) {
            has_smoothed_ = false;
        }
    }

    // 更新子系统
    updatePhase(img_w, img_h);
    updateStability();
    updatePValue(timestamp_ms, laser_detected && has_smoothed_);

    // 周期性重新校准分级
    if (timestamp_ms - last_recalibrate_ms_ >
        static_cast<int64_t>(cfg_.recalibrate_interval_s * 1000)) {
        recalibrateTier();
        last_recalibrate_ms_ = timestamp_ms;
    }

    // 写日志
    if (cfg_.log_enabled && log_stream_.is_open() &&
        timestamp_ms - last_log_ms_ >= cfg_.log_interval_ms) {
        writeLog(timestamp_ms);
        last_log_ms_ = timestamp_ms;
    }
}

// ─── 飞行阶段检测 ───────────────────────────────────────────────────

void MatchAnalyzer::updatePhase(int img_w, int /*img_h*/) {
    if (history_.size() < 3) {
        phase_ = FlightPhase::IDLE;
        return;
    }

    const auto& cur = history_.back();
    if (!cur.detected && cur.drone_center.x < 0) {
        // 未检测到无人机 — 可能停机或超出视野
        if (phase_ == FlightPhase::ATTACKING || phase_ == FlightPhase::CRUISING) {
            phase_ = FlightPhase::RETURNING;
        } else {
            phase_ = FlightPhase::IDLE;
        }
        return;
    }

    // 从最近几帧计算速度 (像素/帧)
    float speed_px = 0.0f;
    int count = 0;
    for (int i = static_cast<int>(history_.size()) - 1;
         i >= 1 && count < 5; --i, ++count) {
        const auto& a = history_[i - 1];
        const auto& b = history_[i];
        if (a.drone_center.x >= 0 && b.drone_center.x >= 0) {
            float dx = b.drone_center.x - a.drone_center.x;
            float dy = b.drone_center.y - a.drone_center.y;
            speed_px += std::sqrt(dx * dx + dy * dy);
        }
    }
    if (count > 0) speed_px /= count;

    // 计算 bbox 高度变化率 (用于起飞/降落判断)
    float bbox_grow = 0.0f;
    if (history_.size() >= 5) {
        float h_old = history_[history_.size() - 5].drone_bbox_h;
        float h_new = cur.drone_bbox_h;
        if (h_old > 1.0f) {
            bbox_grow = (h_new - h_old) / h_old;
        }
    }

    float norm_x = (cur.drone_center.x >= 0)
                       ? cur.drone_center.x / static_cast<float>(img_w)
                       : 0.0f;

    // 飞行阶段状态机
    if (speed_px < cfg_.phase_idle_max_speed_px && bbox_grow < 0.01f) {
        // 极慢 + 尺寸稳定 → 停机或攻击
        if (norm_x > cfg_.phase_attack_x_threshold) {
            phase_ = FlightPhase::ATTACKING;  // 靠近前方 → 攻击中
        } else {
            phase_ = FlightPhase::IDLE;       // 靠近己方 → 停机坡
        }
    } else if (bbox_grow > cfg_.phase_takeoff_bbox_grow_rate) {
        phase_ = FlightPhase::TAKEOFF;
    } else if (bbox_grow < -cfg_.phase_takeoff_bbox_grow_rate) {
        phase_ = FlightPhase::LANDING;
    } else if (speed_px >= cfg_.phase_cruise_min_speed_px) {
        // 快速移动
        if (phase_ == FlightPhase::ATTACKING || phase_ == FlightPhase::RETURNING) {
            phase_ = FlightPhase::RETURNING;
        } else {
            phase_ = FlightPhase::CRUISING;
        }
    } else if (speed_px < cfg_.phase_attack_max_speed_px &&
               norm_x > cfg_.phase_attack_x_threshold) {
        phase_ = FlightPhase::ATTACKING;
    }
}

// ─── 稳定性测量 ─────────────────────────────────────────────────────

void MatchAnalyzer::updateStability() {
    if (history_.size() < 2) return;

    const auto& prev = history_[history_.size() - 2];
    const auto& cur = history_.back();

    if (prev.detected && cur.detected) {
        float dx = cur.center.x - prev.center.x;
        float dy = cur.center.y - prev.center.y;
        float disp = std::sqrt(dx * dx + dy * dy);
        shake_samples_.push_back(disp);
    }

    // 保留校准窗口内的样本数 (~30fps * window_s)
    int max_samples = static_cast<int>(30.0f * cfg_.calibration_window_s);
    while (static_cast<int>(shake_samples_.size()) > max_samples) {
        shake_samples_.pop_front();
    }

    // 计算90分位抖动度 (排除极端值)
    if (shake_samples_.size() >= 10) {
        std::vector<float> sorted(shake_samples_.begin(), shake_samples_.end());
        std::sort(sorted.begin(), sorted.end());
        int idx_90 = static_cast<int>(sorted.size() * 0.9f);
        measured_shake_px_ = sorted[std::min(idx_90,
                                             static_cast<int>(sorted.size()) - 1)];
    }
}

// ─── 自适应分级重新校准 ─────────────────────────────────────────────

void MatchAnalyzer::recalibrateTier() {
    if (shake_samples_.size() < 10) return;

    int64_t now = history_.empty() ? 0 : history_.back().timestamp_ms;
    if (now - last_tier_change_ms_ <
        static_cast<int64_t>(cfg_.tier_change_cooldown_s * 1000)) {
        return;
    }

    // 找到与实测抖动最匹配的分级
    int best_tier = cfg_.preset_tier;  // 回退到预设值
    for (int t = 0; t < 4; ++t) {
        if (measured_shake_px_ <= cfg_.tiers[t].shake_threshold_px) {
            best_tier = t + 1;
            break;
        }
        if (t == 3) best_tier = 4;  // 超过所有阈值
    }

    if (best_tier != current_tier_) {
        std::cout << "MatchAnalyzer: 分级 " << current_tier_ << " → " << best_tier
                  << " (实测抖动=" << measured_shake_px_ << "px"
                  << ", 策略=" << aimStrategyStr(cfg_.tiers[best_tier - 1].strategy)
                  << ")\n";
        current_tier_ = best_tier;
        last_tier_change_ms_ = now;
    }
}

// ─── P值跟踪 ───────────────────────────────────────────────────────
// P值增长公式: 连续照射 t 秒, n = floor(t / 0.1), P = n(n+1)/2
// 第1次锁定: P0=50, 需 n≥10 → 连续照射 ≥1.0s
// 第2次锁定: P0=100, 需 n≥14 → 连续照射 ≥1.4s
// 第3次锁定: P0=100, 需 n≥14 → 连续照射 ≥1.4s (第3次后停止发光)
// 未照射时 P 每秒衰减 p_decay_rate

void MatchAnalyzer::updatePValue(int64_t timestamp_ms, bool illuminating) {
    if (lock_count_ >= cfg_.max_locks) return;

    float dt_s = 0.0f;
    if (history_.size() >= 2) {
        dt_s = (timestamp_ms - history_[history_.size() - 2].timestamp_ms) / 1000.0f;
    }
    if (dt_s <= 0.0f || dt_s > 1.0f) dt_s = 1.0f / 30.0f;

    if (illuminating) {
        if (!was_illuminating_) {
            // 新一轮照射开始
            illuminate_start_ms_ = timestamp_ms;
            continuous_illuminate_s_ = 0.0f;
        }
        continuous_illuminate_s_ = (timestamp_ms - illuminate_start_ms_) / 1000.0f;

        // P值增长: 每0.1s, n计数+1
        // n = floor(连续照射时间 / 判定间隔)
        // P = 1+2+...+n = n*(n+1)/2
        int n = static_cast<int>(continuous_illuminate_s_ / cfg_.p_judge_interval);
        estimated_p_ = static_cast<float>(n * (n + 1)) / 2.0f;
    } else {
        // 未照射 → P值衰减, 连续计数器重置
        continuous_illuminate_s_ = 0.0f;
        estimated_p_ = std::max(0.0f, estimated_p_ - cfg_.p_decay_rate * dt_s);
    }
    was_illuminating_ = illuminating;

    // 检查P值是否达到阈值 → 锁定
    int current_p0 = cfg_.p0_first;
    if (lock_count_ == 1) current_p0 = cfg_.p0_second;
    if (lock_count_ >= 2) current_p0 = cfg_.p0_third;

    if (estimated_p_ >= static_cast<float>(current_p0)) {
        lock_count_++;
        estimated_p_ = 0.0f;
        continuous_illuminate_s_ = 0.0f;
        std::cout << "MatchAnalyzer: ★ 第" << lock_count_ << "次锁定达成! ★\n";
    }
}

// ─── 日志记录 ────────────────────────────────────────────────────────

void MatchAnalyzer::writeLog(int64_t timestamp_ms) {
    if (!log_stream_.is_open()) return;

    const auto& sample = history_.back();
    log_stream_ << "  - { t: " << timestamp_ms
                << ", phase: \"" << flightPhaseStr(phase_) << "\""
                << ", tier: " << current_tier_
                << ", strategy: \"" << aimStrategyStr(currentTierParams().strategy) << "\""
                << ", shake: " << measured_shake_px_
                << ", P: " << estimated_p_
                << ", locks: " << lock_count_
                << ", det: " << (sample.detected ? 1 : 0)
                << ", conf: " << sample.confidence
                << ", tx: " << smoothed_target_.x
                << ", ty: " << smoothed_target_.y
                << ", illum_s: " << continuous_illuminate_s_
                << ", on_target: " << (isOnTarget() ? 1 : 0)
                << " }\n";
    log_entry_count_++;

    // 定期刷新
    if (log_entry_count_ % 50 == 0) {
        log_stream_.flush();
    }
}

void MatchAnalyzer::finalize() {
    if (log_stream_.is_open()) {
        log_stream_ << "# 摘要:\n";
        log_stream_ << "total_entries: " << log_entry_count_ << "\n";
        log_stream_ << "final_tier: " << current_tier_ << "\n";
        log_stream_ << "final_locks: " << lock_count_ << "\n";
        log_stream_ << "final_shake_px: " << measured_shake_px_ << "\n";
        log_stream_.flush();
        log_stream_.close();
        std::cout << "MatchAnalyzer: 日志已完成 (" << log_entry_count_
                  << " 条记录, " << lock_count_ << " 次锁定)\n";
    }
}

// ─── 获取器 ──────────────────────────────────────────────────────────

FlightPhase MatchAnalyzer::currentPhase() const { return phase_; }
int MatchAnalyzer::currentTier() const { return current_tier_; }

const StabilityTierParams& MatchAnalyzer::currentTierParams() const {
    return cfg_.tiers[std::clamp(current_tier_, 1, 4) - 1];
}

AimStrategy MatchAnalyzer::currentStrategy() const {
    return currentTierParams().strategy;
}

float MatchAnalyzer::measuredShakePx() const { return measured_shake_px_; }
float MatchAnalyzer::estimatedP() const { return estimated_p_; }
int MatchAnalyzer::lockCount() const { return lock_count_; }
float MatchAnalyzer::continuousIlluminateS() const { return continuous_illuminate_s_; }

cv::Point2f MatchAnalyzer::smoothedTarget() const { return smoothed_target_; }
bool MatchAnalyzer::hasSmoothedTarget() const { return has_smoothed_; }

bool MatchAnalyzer::isOnTarget() const {
    // 瞄准质量评估: 激光上电即常亮, 此函数评估的是激光是否正在有效照射目标
    // 返回 true 表示当前瞄准质量足够, P值可能正在累积
    // 返回 false 表示瞄准质量差, P值不太可能在增长
    if (!has_smoothed_) return false;
    if (lock_count_ >= cfg_.max_locks) return false;

    // 按策略评估瞄准质量
    switch (currentStrategy()) {
        case AimStrategy::PRECISE:
            // 精确跟踪: 有平滑目标即认为在瞄准
            return true;

        case AimStrategy::TRACK:
            // 正常跟踪: 近期有检测才算有效瞄准
            return frames_since_detection_ <= 2;

        case AimStrategy::PREDICT:
            // 预测跟踪: 预测位置也算, 但不能太久
            return frames_since_detection_ <= currentTierParams().hold_frames;

        case AimStrategy::OPPORTUNE: {
            // 瞄准质量评估: 只在目标相对稳定时认为照射有效
            if (frames_since_detection_ > 2) return false;
            if (shake_samples_.size() < 3) return false;
            float recent_avg = 0.0f;
            int n = std::min(5, static_cast<int>(shake_samples_.size()));
            auto it = shake_samples_.end();
            for (int i = 0; i < n; ++i) {
                --it;
                recent_avg += *it;
            }
            recent_avg /= n;
            return recent_avg < currentTierParams().shake_threshold_px * 0.5f;
        }
    }
    return false;
}

}  // namespace solve
