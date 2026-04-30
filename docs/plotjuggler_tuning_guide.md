# PlotJuggler 调参指南

## 快速启动

```bash
# 终端1: 启动 PlotJuggler
plotjuggler
# 或: QT_XCB_GL_INTEGRATION=none /home/zst/Downloads/PlotJuggler-3.17.1-x86_64.AppImage

# 终端2: 启动跟踪程序
./tests/build/track --color red
```

PlotJuggler 配置: **Streaming → UDP Server → 端口 9870 → JSON → OK**

---

## 信号说明

| 信号名 | 含义 | 单位 | 理想状态 |
|--------|------|------|----------|
| `cmd_yaw` | 上位机发给电控的 yaw **指令** | 度(°) | — |
| `cmd_pitch` | 上位机发给电控的 pitch **指令** | 度(°) | — |
| `state_yaw` | 电控反馈的 yaw **实际位置** | 度(°) | 紧跟 `cmd_yaw` |
| `state_pitch` | 电控反馈的 pitch **实际位置** | 度(°) | 紧跟 `cmd_pitch` |
| `error_yaw` | yaw 误差 = `cmd_yaw − state_yaw` | 度(°) | → 0 |
| `error_pitch` | pitch 误差 = `cmd_pitch − state_pitch` | 度(°) | → 0 |
| `target_u` | 目标在图像中的 **水平像素坐标** | px | 趋近 `u_L`(641) |
| `target_v` | 目标在图像中的 **垂直像素坐标** | px | 趋近 `boresight_v` |
| `Z_est` | 估算目标距离 = fy × H_物理 / bbox_h | 米(m) | 近小远大 |
| `dpitch_deg` | 角度域视差补偿量 = atan(0.033/Z) | 度(°) | 近大远小→0 |
| `bbox_h` | 目标检测框高度（反映目标距离） | px | 近大远小 |

### 推荐观察组合

1. **跟随性能**: 拖 `cmd_pitch` + `state_pitch` 到同一图表
   - 两条线贴合 = 跟随好; 间距大 = 云台响应慢
2. **误差收敛**: 拖 `error_yaw` + `error_pitch` 到同一图表
   - 快速归零 = 调得好; 振荡 = kp 太大; 收不回来 = kp 太小
3. **视差补偿**: 拖 `dpitch_deg` + `Z_est` + `bbox_h` 到同一图表
   - 目标靠近时 `dpitch_deg` 增大, `Z_est` 减小

---

## 调参方法

所有控制参数在 `src/control/config/control.yaml`，**改完直接重启 track 即可生效，无需重新编译**。

### 1. kp — 比例增益（最重要）

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

在 `tests/track.cpp` 中（修改后需重新编译）:

```cpp
constexpr double kLaserOffsetY = 0.020;   // 激光在相机下方的距离(米)
constexpr double kModuleHeight = 0.050;    // 激光接收模块的物理高度(米)
```

公式: `dyn_vL = cy + (kLaserOffsetY / kModuleHeight) × bbox_h`

> `control.yaml` 中的 `v_L: 520.0` 仅为静态备用值，运行时被动态值覆盖。

### 赛前只需量两个物理尺寸（一次性）

| 参数 | 测量方法 | 当前值 |
|------|----------|--------|
| `kLaserOffsetY` | 尺子量：激光发射孔中心 → 相机镜头中心的**垂直距离** | 0.020m (2cm) |
| `kModuleHeight` | 尺子量：激光接收模块（检测目标）的**实际高度** | 0.050m (5cm) |

量好后写入 track.cpp，编译一次，之后距离远近变化全自动适应。

---

## 调参 Checklist

1. [ ] 启动 PlotJuggler，连接 UDP 9870
2. [ ] 运行 track，观察 `cmd_pitch` vs `state_pitch` 曲线
3. [ ] 调 `kp`: 逐步增大直到轻微振荡，然后回退一档
4. [ ] 调 `lowpass_alpha`: 在响应速度和稳定性之间取平衡
5. [ ] 调 `ff_alpha`: 目标运动时观察是否提前预判
6. [ ] 调 `damping_kd`: 消除到达目标后的振荡
7. [ ] 验证视差: 拖 `boresight_v` 曲线，靠近/远离时应自动变化
8. [ ] 用尺子校准 `kLaserOffsetY` 和 `kModuleHeight`
