# PlotJuggler 调参指南

## 快速启动

```bash
# 终端1: 启动 PlotJuggler
plotjuggler
# 或: QT_XCB_GL_INTEGRATION=none /home/zst/Downloads/PlotJuggler-3.17.1-x86_64.AppImage


```yaml
kp: 3.0   # 当前值
```

| 现象 | 操作 |
|------|------|
| `cmd` 和 `state` 差距大，收敛慢 | ↑ 增大 kp (试 4.0 → 5.0) |
| `error` 曲线来回振荡不收敛 | ↓ 减小 kp (试 2.0 → 1.5) |
| 跟随快且稳，误差快速归零 | ✓ 当前值合适 |

**调参步骤**: 从 3.0 开始，每次 +1.0 观察 PlotJuggler 曲线，直到出现轻微振荡后回退一档。

### 2. lowpass_alpha — 低通滤波（平滑度）

```yaml
lowpass_alpha: 0.7   # 当前值, 范围 0~1
```

| 值 | 效果 |
|----|------|
| → 1.0 | 几乎不滤波，响应最快，但可能抖动 |
| → 0.0 | 重度平滑，响应慢，但非常稳 |

**建议**: 0.5~0.8 之间。如果 `cmd` 曲线抖得厉害就降低，如果延迟大就提高。

### 3. ff_alpha — 速度前馈（预判运动）

```yaml
use_velocity_ff: 1
ff_alpha: 0.5   # 当前值, 范围 0~1
```

| 值 | 效果 |
|----|------|
| 0.0 | 无前馈，纯追踪（总慢半拍） |
| 0.5 | 中等预判 |
| 0.8 | 强预判，适合目标快速移动 |

**建议**: 目标运动快时增大到 0.6~0.8；目标基本静止时可降到 0.2。

### 4. damping_kd — 阻尼系数

```yaml
use_damping: 1
damping_kd: 0.008   # 当前值
```

| 现象 | 操作 |
|------|------|
| 到达目标后来回晃（超调振荡） | ↑ 增大 (试 0.02 → 0.05) |
| 跟随太慢太黏 | ↓ 减小 (试 0.005 → 0.002) |

### 5. max_angle_rate — 最大角速率

```yaml
max_angle_rate: 180.0   # 度/秒
```

限制每帧最大角度变化量。如果云台物理能力有限，设小一些防止指令跳变。

---

## 动态视差补偿（自动，无需赛场校准）

**原理**: 激光和相机有物理偏移，导致不同距离下激光落点在图像中的位置不同。
系统通过目标检测框大小反推距离，实时计算补偿值。

```
近距离 → bbox 大 → 补偿大
远距离 → bbox 小 → 补偿小
```

在 `tracking` 入口中（修改后需重新编译）:

```cpp
// 这两个值来自 config/control.yaml，而不是写死在代码里
double laser_offset_y = 0.033905;   // 激光在相机下方的距离(米)
double target_height_m = 0.072;     // 激光接收模块的物理高度(米)
```

实际使用位置：`tracking/track.cpp` 里会先用 bbox 高度估距离，再算补偿角：

```cpp
Z_est = cam_model.fy * target_height_m / bbox_h;
dpitch_deg = std::atan(laser_offset_y / Z_est) * kRadToDeg;
```

> `control.yaml` 中的 `v_L` 只是旧的静态参考值；当前 tracking 入口不再直接依赖这组像素常量。

### 赛前只需量两个物理尺寸（一次性）

| 参数 | 测量方法 | 当前值 |
|------|----------|--------|
| `laser_offset_y` | 尺子量：激光发射孔中心 → 相机镜头中心的**垂直距离** | 0.033905m |
| `target_height_m` | 尺子量：激光接收模块（检测目标）的**实际高度** | 0.072m |

量好后写入 `config/control.yaml`，重新启动 `tracking` 即可生效，不需要改源码。

---

## 调参 Checklist

1. [ ] 启动 PlotJuggler，连接 UDP 9870
2. [ ] 运行 track，观察 `cmd_pitch` vs `state_pitch` 曲线
3. [ ] 调 `kp`: 逐步增大直到轻微振荡，然后回退一档
4. [ ] 调 `lowpass_alpha`: 在响应速度和稳定性之间取平衡
5. [ ] 调 `ff_alpha`: 目标运动时观察是否提前预判
6. [ ] 调 `damping_kd`: 消除到达目标后的振荡
7. [ ] 验证 `cmd_pitch` / `state_pitch` 收敛情况
