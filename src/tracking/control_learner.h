#ifndef TRACKING_CONTROL_LEARNER_H
#define TRACKING_CONTROL_LEARNER_H

#include <string>

namespace tracking_app {

// 控制参数在线学习: 训练阶段采集每帧跟踪误差自动调优 ff_gain.
// 比赛阶段加载训练成果, 立即使用最优参数.
//
// 原理:
//   FF 增益学习: 误差与速度的相关性
//     err * vel > 0  → 误差方向与运动方向一致 → FF 不足, 增大 ff_gain
//     err * vel < 0  → FF 过大, 减小
//     gradient = sum(err*vel) / sum(vel^2)  (归一化最小二乘)
//   学习结果持久化到 logs/learned_control.yaml, 重启自动加载
struct ControlLearner {
    // ── 学习结果 (跨会话持久化) ──
    double ff_gain_yaw = 1.0;
    double ff_gain_pitch = 1.0;
    int session_count = 0;
    int lifetime_tracking_frames = 0;
    int lifetime_hit_frames = 0;
    double ema_hit_ratio = 0.0;

    // ── 在线 EMA 状态 (每帧更新, 不攒批次) ──
    double ema_err_yaw = 0.0;          // EMA 跟踪的 yaw 绝对误差 (deg)
    double ema_err_pitch = 0.0;        // EMA 跟踪的 pitch 绝对误差 (deg)
    double ema_osc_yaw = 0.0;          // EMA 跟踪的 yaw 振荡率 (0~1)
    double ema_osc_pitch = 0.0;        // EMA 跟踪的 pitch 振荡率 (0~1)
    double ema_ff_grad_yaw = 0.0;      // EMA 跟踪的 yaw FF 梯度
    double ema_ff_grad_pitch = 0.0;    // EMA 跟踪的 pitch FF 梯度
    double last_err_yaw = 0.0;
    double last_err_pitch = 0.0;
    int session_tracking_frames = 0;
    int session_hit_frames = 0;

    // EMA 常数
    static constexpr double kEmaFast = 0.08;   // 快速 EMA (误差/振荡)
    static constexpr double kEmaSlow = 0.02;   // 慢速 EMA (梯度)

    // 加载持久化文件 (yaml). 缺字段保留默认值.
    bool load(const std::string& path);

    // 写出持久化文件. 失败静默 (parent dir 自动创建).
    void save(const std::string& path) const;
};

}  // namespace tracking_app

#endif  // TRACKING_CONTROL_LEARNER_H
