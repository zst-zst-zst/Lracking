# 调参指南

> 合并自: PlotJuggler 调参指南、25-30m 远距离联调指南、control 模块说明

---

## PlotJuggler 连接

```bash
plotjuggler
# 或: QT_XCB_GL_INTEGRATION=none /path/to/PlotJuggler.AppImage
```

数据源: UDP 端口 9870, tracking 程序运行后自动推送。

---

## 控制逻辑概述

- 像素误差 → `atan(du/fx)` → 角度误差 (deg)
- 激光补偿: `boresight_deg` (常数) + `atan(laser_offset/Z)` (视差)
- 支持: 死区、滞回、输出低通、延迟补偿、速度前馈、自适应增益
- 目标丢失: yaw 微扫, pitch 保持最后有目标时的位置

---

## 参数调节

### 1. kp — 主增益

```yaml
kp: 3.0
```

| 现象 | 操作 |
|------|------|
| cmd 和 state 差距大, 收敛慢 | ↑ kp (试 4.0→5.0) |
| error 来回振荡 | ↓ kp (试 2.0→1.5) |
| 跟随快且稳 | ✓ 当前合适 |

**方法**: 从 3.0 开始, 每次 +1.0 观察曲线, 出现轻微振荡后回退一档。

### 2. lowpass_alpha — 输出低通

```yaml
lowpass_alpha: 0.7   # 0~1
```

| 值 | 效果 |
|----|------|
| → 1.0 | 几乎不滤波, 响应最快, 可能抖 |
| → 0.0 | 重度平滑, 响应慢, 非常稳 |

**建议**: 0.5~0.8。cmd 抖→降低; 延迟大→提高。

### 3. ff_alpha — 速度前馈

```yaml
use_velocity_ff: 1
ff_alpha: 0.5        # 0~1
ff_rate_max: 150.0   # deg/s 上限
```

| 值 | 效果 |
|----|------|
| 0.0 | 无前馈, 纯追踪 (总慢半拍) |
| 0.5 | 中等预判 |
| 0.8 | 强预判, 适合快速运动 |

### 4. damping_kd — 阻尼

```yaml
use_damping: 1
damping_kd: 0.008
```

| 现象 | 操作 |
|------|------|
| 到达目标后来回晃 | ↑ (试 0.02→0.05) |
| 跟随太慢太黏 | ↓ (试 0.005→0.002) |

### 5. max_angle_rate — 最大角速率

```yaml
max_angle_rate: 180.0   # deg/s
```

限制每帧最大角度变化。云台物理能力有限时设小。

---

## 自适应增益 (远距离)

远距离目标 bbox 小, 像素误差小, 但角度误差不变 → 需要更大 kp。

```yaml
adaptive_kp: 1
kp_near: 3.0         # bbox > 8000px² 时
kp_far: 6.0          # bbox < 800px² 时
kp_near_area: 8000.0
kp_far_area: 800.0
```

---

## 延迟补偿

```yaml
delay_compensate: 1
system_delay_ms: 10.0   # 先用10ms, 逐步调
```

测量: PlotJuggler 中 cmd_yaw 和 state_yaw 的时间差。
典型值: 检测(5ms) + 通信(3ms) + 电控响应(10-20ms) ≈ 20-30ms。

---

## 激光视差补偿 (自动)

激光和相机有物理偏移, 不同距离下补偿不同:

```
近距离 → bbox 大 → 补偿大
远距离 → bbox 小 → 补偿小 (20m+ 时 < 0.1°, 可忽略)
```

`control.yaml` 中 4 个标定值 (由 `laser_boresight` 写入):

| 参数 | 含义 | 当前值 |
|------|------|--------|
| `boresight_yaw_deg` | 激光-相机 yaw 角度偏差 (常数) | 0.4176° |
| `boresight_pitch_deg` | 激光-相机 pitch 角度偏差 (常数) | 1.3838° |
| `laser_offset_x` | 物理水平偏移 (m) | 0.000318 |
| `laser_offset_y` | 物理垂直偏移 (m) | 0.029600 |

| 距离 | 视差 pitch 补偿 (offset_y=29.6mm) |
|------|-----|
| 2m | 0.85° |
| 5m | 0.34° |
| 10m | 0.17° |
| 20m | 0.085° |
| 30m | 0.057° |

---

## 自学习系统

每次命中 (紫色):
1. 记录参数到 `logs/learning_samples.csv`
2. EMA 微调 `boresight_yaw_deg` / `boresight_pitch_deg`
3. 保存到 `logs/bs_learned.yaml`, 下次启动自动加载

```yaml
# control.yaml
bs_learn_alpha: 0.03       # EMA 学习率
bs_learn_min_hits: 3       # 最少命中次数才开始学习
```

---

## 远距离 (25-30m) 精度提升优先级

| 优先级 | 措施 | 预期效果 |
|--------|------|----------|
| 1 | 换高分辨率相机 | 30m: 2.16px→4.3px (500万) 或 6.3px (800万) |
| 2 | 亚像素定位 (已实现) | 精度 ±1px→±0.3px |
| 3 | 延迟补偿 (已实现) | 减少跟踪滞后 |
| 4 | 速度前馈 (已实现) | 电机提前预判 |
| 5 | 自学习 (已实现) | 越用越准 |
| 6 | 增大 YOLO 输入分辨率 | 小目标检测更准 |

### 远距离参数建议

```yaml
lowpass_alpha: 0.4    # 加重平滑
kp: 6.0               # 或用 adaptive_kp
deadband_px: 3        # 忽略微小抖动
ff_alpha: 0.2         # 前馈保守
```

---

## 调参 Checklist

1. 启动 PlotJuggler, 连接 UDP 9870
2. 运行 tracking, 观察 `cmd_pitch` vs `state_pitch`
3. 调 `kp`: 逐步增大直到轻微振荡, 回退一档
4. 调 `lowpass_alpha`: 响应速度与稳定性平衡
5. 调 `ff_alpha`: 运动目标是否提前预判
6. 调 `damping_kd`: 消除到达后振荡
7. 验证 cmd/state 收敛
