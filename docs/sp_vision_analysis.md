# sp_vision_25 架构分析 & 同轴补偿方案对比

## 一、sp_vision_25 识别逻辑

### 检测 (detector.cpp)
- **传统 CV 流水线**: 灰度化 → 二值化 → 轮廓提取 → 灯条筛选 → 灯条配对成装甲板
- **分类器**: ResNet 小网络对装甲板 ROI 分类（数字识别）
- **可选 YOLO**: 代码中有 yolo.hpp 但 uav.cpp 注释掉了
- 不依赖 bbox 大小推断距离

### 测距 — **solvePnP** (solver.cpp, 核心区别)
```cpp
// 已知装甲板物理尺寸
constexpr double LIGHTBAR_LENGTH = 56e-3;     // 56mm
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // 135mm

// 用4个角点的2D-3D对应关系，解出目标在相机坐标系的3D位置
cv::solvePnP(object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec);

// 相机坐标 → 云台坐标（这里包含了激光/相机的物理偏移！）
armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
//                                                        ^^^^^^^^^^^^^^^^
//                   这个向量就是相机到云台旋转中心的物理偏移(米)
//                   uav.yaml: t_camera2gimbal: [-0.035, -0.00007, 0.001]
//                   即相机在云台中心左侧3.5cm

// 云台坐标 → 世界坐标
armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;
```

### 跟踪 — **EKF** (target.cpp)
- 11 维状态: `[x, vx, y, vy, z, vz, angle, angular_vel, radius, delta_r, delta_z]`
- 建模整车旋转中心 + 装甲板绕中心旋转
- 可处理小陀螺、装甲板跳变等复杂场景

### 瞄准 — **弹道迭代** (aimer.cpp + trajectory.cpp)
```cpp
// 1. 预测目标未来位置（考虑延迟 + 飞行时间）
target.predict(future + fly_time);

// 2. 弹道解算（抛物线，考虑重力）
Trajectory(bullet_speed, horizontal_distance, height);
// 解二次方程得 pitch 抬枪角

// 3. 迭代（最多10次）直到收敛
// 因为 fly_time 依赖距离，距离依赖预测位置，预测位置依赖 fly_time

// 4. 最终输出
yaw = atan2(y, x) + yaw_offset;
pitch = -(trajectory.pitch + pitch_offset);
```

### 控制输出 (cboard.cpp)
- 通过 CAN 总线发送
- **发送的是绝对角度（弧度）**，不是角度增量
- 下位机负责角度闭环

---

## 二、同轴补偿：为什么会随距离变化？

**物理本质**: 激光发射器和相机有物理间距 d，这导致两者的光轴不平行或不重合。

```
目标近(2m)时:
  相机 ──→ ×  (目标)
  激光 ↗     ← 偏差大(角度大)

目标远(20m)时:
  相机 ──────────→ ×  (目标)
  激光 ──────────↗    ← 偏差小(角度小)
```

像素偏移: `dv = fy × d / Z`

| 距离 Z | 像素偏移 (d=3.3cm, fy=1293) |
|--------|---------------------------|
| 2m | 21.3 px |
| 5m | 8.5 px |
| 10m | 4.3 px |
| 20m | 2.1 px |
| 30m | 1.4 px |

**结论: 距离越远，像素补偿越小。20m以上补偿已不到2px，可以忽略。**

---

## 三、三种补偿方案对比

### 方案A: sp_vision 的方法 (solvePnP + 坐标变换)
```
已知目标物理尺寸 → solvePnP得3D坐标 → t_camera2gimbal补偿 → 输出绝对角度
```
- ✅ 数学严格正确，自动处理所有距离
- ✅ 只需标定一次 `t_camera2gimbal`（尺子量）
- ❌ **需要检测目标的4个角点**
- ❌ 需要目标物理尺寸精确已知
- ❌ 远距离(20m+)目标太小，角点不稳定，solvePnP 容易发散

### 方案B: 我们当前的方法 (bbox高度反推距离)
```
bbox_h → Z = fy * H / bbox_h → dv = fy * d / Z = d * bbox_h / H
```
- ✅ 简单，不需要角点
- ✅ YOLO bbox 在远距离也能工作
- ❌ bbox_h 精度有限(±5px 的抖动在远距离会放大)
- ❌ 需要知道目标物理高度

### 方案C: 更优方案 — **角度域补偿** (推荐)
```
不在像素域补偿，而是在角度域补偿
```
- 先用 bbox_h 估算距离 Z
- 计算角度补偿: `dpitch_offset = atan(d / Z)`
- 在输出的 pitch 指令上加这个角度偏移
- ✅ 物理意义清晰
- ✅ 远距离时补偿自动趋近于0，不会过补偿
- ✅ 不修改 boresight，不干扰跟踪控制
- ✅ 20-30m 时 dpitch < 0.1°，自然消失

---

## 四、对于 20-30m 远距离场景的建议

1. **20m+ 时同轴补偿几乎可以忽略** (< 2px, < 0.1°)
   - 你的 3.3cm 偏移在 20m 处只产生 0.095° 的角度误差
   - 大部分误差来自检测抖动，而非视差

2. **远距离的真正挑战是**:
   - 检测抖动: 目标只有几个像素大，bbox 中心抖动 ±2px = ±0.09°
   - 延迟: 检测+通信延迟导致的滞后
   - 大气抖动(30m+): 热空气折射

3. **远距离最佳实践**:
   - 加重低通滤波(lowpass_alpha 降到 0.3-0.4)
   - 减小 kp 避免放大噪声
   - 增大 deadband_px(如 3-5px)忽略微小抖动
   - 前馈预测要保守(ff_alpha 降到 0.2)
