# 25-30m 远距离跟踪联调指南

## 硬件参数
```
相机: 大恒 MER-139-210U3C (139万像素, 1280×1024)
镜头: MVL-KF3528M-8MP (35mm, 镜头分辨率800万)
fx = 1293 px, fy = 1292 px
标定重投影误差: 0.07 px
```

## 系统精度预算 (30m)
```
目标: S190 激光检测模块 (正面视角)
  - 整体: 50mm(高) × 50mm(宽) × 50mm(深)
  - 检测区域: φ42mm (正面看就是整个目标大小)
  - 必须打在 φ42mm 正中心
  - 容差 = φ42/2 = 21mm (半径, 从中心到边缘)

30m处像素尺寸:
  1px = 30000mm / 1293 = 23.2mm
  目标: 50mm → 2.16px × 2.16px (极小!)
  容差: 21mm / 23.2mm/px = 0.91px

精度预算: ±0.91px ≈ ±21mm ≈ ±0.04° ≈ ±0.7mrad
→ 仍然非常紧张! (YOLO检测框误差通常 ±1-3px)

25m处:
  1px = 25000/1293 = 19.3mm
  目标: 2.59px, 容差: 21/19.3 = 1.09px → 稍微好一点
```

## 第一步: 静态标定 (视觉单独)
1. 确认 `config/camera.yaml` 相机内参准确 (fx, fy, cx, cy)
   - 远距离精度完全依赖焦距标定
   - 建议用 10+ 张棋盘格重新标定
2. 确认 `control.yaml` 中 boresight (u_L, v_L) 已校准
   - 用 `laser_calib` 工具在 5m/10m/20m 三个距离标定
   - 结果保存在 control.yaml 的 u_L, v_L
3. 确认激光偏移 (laser_offset_x, laser_offset_y) 精确
   - 用尺子量相机光心到激光发射点的物理距离 (mm)

## 第二步: NUC→STM32 通信验证
```bash
# 1. 编译运行 tracking
cd /home/zst/Tracking/src/build && ./tracking/tracking

# 2. 观察终端输出:
#    DBG gimbal cmd: mode=1 yaw=xxx pitch=xxx
#    → 确认 mode≠0, yaw/pitch 在合理范围

# 3. 电控侧用串口调试工具看收到的帧:
#    yaw, pitch: 应该和 DBG 输出一致
#    yaw_vel, pitch_vel: 目标静止时≈0, 运动时有值
```

## 第三步: 电控 PID 调参 (和电控一起做)

### 3.1 先关掉速度前馈, 只调角度跟踪
```yaml
# control.yaml
use_velocity_ff: 0    # 先关掉前馈
```
- 让视觉锁定静止目标, 看云台是否能稳定瞄准
- 观察 PlotJuggler: cmd_yaw vs state_yaw, 误差应该 < 0.1°
- 如果振荡: 电控降低角度环 Kp (当前=8), 或加大速度环阻尼

### 3.2 调视觉端 kp
- 目标在画面中心附近(误差小)时, kp=3.0 应该够
- 如果跟踪滞后(总是落后目标), 加大 kp
- 如果震荡/抖动, 减小 kp 或加大 damping_kd
```yaml
# control.yaml
kp: 3.0          # 主增益
damping_kd: 0.008  # 阻尼 (抑制超调)
lowpass_alpha: 0.7  # 低通 (0.5更平滑, 1.0更快)
```

### 3.3 开启速度前馈, 调前馈参数
```yaml
# control.yaml
use_velocity_ff: 1
ff_alpha: 0.5       # 前馈EMA平滑 (0.3更稳, 0.7更快)
ff_rate_max: 150.0   # 前馈速度上限 deg/s (防止噪声)
```
- 让目标匀速运动, 观察跟踪滞后是否减小
- 电控侧: `pid_ref(速度环) = 角度环输出 + yaw_vel`
  - 如果前馈太大导致超调: 减小 ff_alpha 或 ff_rate_max
  - 如果前馈太小没效果: 加大 ff_alpha

### 3.4 开启自适应增益 (远距离)
```yaml
# control.yaml
adaptive_kp: 1
kp_near: 3.0        # 近距离(bbox>8000px²) kp
kp_far: 6.0         # 远距离(bbox<800px²) kp ← 远距离需要更大
kp_near_area: 8000.0
kp_far_area: 800.0
```
远距离目标 bbox 很小, 像素误差也很小, 但角度误差不变 → 需要更大 kp

## 第四步: 延迟补偿
```yaml
# control.yaml
delay_compensate: 1
system_delay_ms: 10.0   # 先用10ms, 逐步调整
```
- 测量方法: 看 PlotJuggler 中 cmd_yaw 和 state_yaw 的时间差
- 匀速运动目标: 如果总是落后, 加大 system_delay_ms
- 典型值: 检测(5ms) + 通信(3ms) + 电控响应(10-20ms) ≈ 20-30ms

## 第五步: 自学习系统
系统会自动:
1. 每次激光命中(紫色), 记录全部参数到 `logs/learning_samples.csv`
2. 用 EMA 自动微调 boresight 偏移
3. 保存到 `logs/bs_learned.yaml`, 下次启动自动加载

命中次数越多, boresight 越准 → **越用越准**

数据格式 (可用 Python/PyTorch 训练更复杂的模型):
```
ts_ms, hit_count, target_u, target_v, bs_u, bs_v, err_u, err_v,
corr_u, corr_v, bbox_area, vel_x, vel_y,
cmd_yaw, cmd_pitch, state_yaw, state_pitch,
ff_yaw_rate, ff_pitch_rate, kp_used
```

## 远距离精度提升建议 (优先级排序)

### 当前瓶颈: 目标仅 2.16px, 容差 ±0.47px

1. **换高分辨率相机** ← **最有效!**
   - 当前: 139万 (1280×1024), fx=1293 → 30m处 2.16px
   - 换500万 (2592×1944), 同35mm镜头 → fx≈2600 → 30m处 4.3px, 容差±0.95px
   - 换800万 (3840×2160) → fx≈3800 → 30m处 6.3px, 容差±1.4px ✅ 舒适
   - 镜头800万分辨率支持升级! 相机是瓶颈
2. **sub-pixel refinement** ← 不换硬件的最优方案 **✅ 已实现**
   - 用图像矩(intensity-weighted centroid)对 YOLO 检测框做亚像素中心修正
   - 实现位置: track.cpp `subpixelRefineCenter()`, 在 YOLO 检测后、Kalman 之前
   - 定位精度从 ±1px → ±0.3px
   - 调试日志: `DBG detections=N [WxH shift=dx,dy]` 显示每个检测的亚像素偏移
3. **延迟补偿** ← 运动目标必须 (已实现)
4. **速度前馈** ← 让电控提前预判 (已实现)
5. **自学习校准** ← 越用越准 (已实现)
6. **增大 YOLO 输入分辨率** ← 当前可能下采样到 640×640
   - 小目标检测建议用 1280×1280 输入 (P2 小目标检测头)
   - 或用 SAHI (切片辅助推理)

## 电控侧关键参数 (gimbal.c)
```c
// YAW GM6020
angle_PID: Kp=8, MaxOut=200 (deg/s)
speed_PID: Kp=25, Ki=100, MaxOut=15000

// PITCH GM6020
angle_PID: Kp=8, MaxOut=200 (deg/s)
speed_PID: Kp=15, Ki=50, MaxOut=15000
```
- MaxOut=200 意味着最大追踪速度 200°/s
- 如果远距离跟踪不上: 确认不是 MaxOut 限制
- 速度前馈叠加在速度环 ref 上, 不受 MaxOut 限制

## 协议单位汇总
| 字段 | 单位 | 方向 |
|------|------|------|
| yaw | deg | IMU YawTotalAngle |
| pitch | deg | -IMU.Pitch |
| yaw_vel | deg/s | 目标运动方向 |
| pitch_vel | deg/s | 目标运动方向 |
| yaw_acc | deg/s² | **不使用, 恒0** |
| pitch_acc | deg/s² | **不使用, 恒0** |
