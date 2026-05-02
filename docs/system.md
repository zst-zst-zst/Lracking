# 系统总览

> 合并自: 总工况、雷达反制无人机系统整理、STM32联调文档、gimbal协议说明
> 更新: 2026-05-02

---

## 一、系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│                     雷达基座 (高度约2.5m)                         │
│  ┌─────────────────────┐          ┌──────────────────────────┐  │
│  │   传感器端 (云台)     │  ←5m线→  │   运算平台端 (NUC)        │  │
│  │  - 相机              │          │  - 目标检测/跟踪          │  │
│  │  - 激光发射装置       │          │  - 坐标解算              │  │
│  │  - GM6020二轴云台     │          │  - 前馈计算              │  │
│  │  - STM32 (电控)      │          │  - 自学习校准            │  │
│  │  - BMI088 IMU        │          │  - HDMI调试画面          │  │
│  └──────────┬───────────┘          └──────────┬───────────────┘  │
│             │ USART6 (29B/43B帧)              │ 航空线→战场网络    │
│             └──────────────────────────────────┘                  │
└──────────────────────────────────────────────────────────────────┘
```

| 模块 | 位置 | 核心硬件 | 供电 |
|------|------|----------|------|
| 运算平台端(NUC) | 雷达基座旁台面 | NUC Mini PC + 主控模块 + 电源管理模块 | 220V AC, ≤750W |
| 传感器端(云台) | 雷达基座平台 | 相机 + 激光 + GM6020×2 + STM32F4 + BMI088 | ≤60V DC, Gimbal口(XT30) |

### 硬件参数

```
相机: MER-139-210U3C (139万, 1280×1024)
镜头: MVL-KF3528M-8MP (35mm, 800万)
fx=1293, fy=1292, 标定误差 0.07px
```

### 尺寸约束

| 约束项 | 限制 |
|--------|------|
| 运算平台端尺寸 | ≤ 600×350×600 mm |
| 传感器端长宽 | 不超出平台 (3.4m×1.16m) |
| 传感器端高度 | ≤ 1500mm (建议支架 ≥ 1.2m) |
| 运算端↔传感器端电缆 | ≥ 5m |
| 激光发射装置 | 仅 1 个, < 1mW, ≤ 730nm |

---

## 二、目标与精度预算

### 激光检测模块 (S190)

整体: 72mm(高) × 50mm(宽) × 50mm(深), 检测区域 φ42mm, 必须打在正中心。

| 距离 | 目标像素 | 1px 对应 | 容差 |
|------|---------|----------|------|
| 10m | 6.5px | 7.7mm | ±2.7px |
| 20m | 3.2px | 15.5mm | ±1.4px |
| 25m | 2.6px | 19.3mm | ±1.1px |
| **30m** | **2.16px** | **23.2mm** | **±0.91px** |

> 30m 容差 ±0.91px ≈ ±0.04°, 极其紧张。

---

## 三、通信协议

### 坐标系

```
YAW:   IMU.YawTotalAngle (度, 累积, 无限制)
PITCH: -IMU.Pitch (度, 电控发送时取反)
       pitch 电机配置 FEEDBACK_DIRECTION_REVERSE → PID内部自洽
```

### NUC → STM32 (29字节)

```c
struct __packed RawVisionToGimbal {
    uint8_t head[2];     // 0xA5, 0x5A
    uint8_t mode;        // 0=不控制, 1=控制不开火, 2=控制且开火
    float   yaw;         // 绝对目标角度 (deg)
    float   yaw_vel;     // 速度前馈 (deg/s)
    float   yaw_acc;     // 恒=0
    float   pitch;       // 绝对目标角度 (deg)
    float   pitch_vel;   // 速度前馈 (deg/s)
    float   pitch_acc;   // 恒=0
    uint8_t tail[2];     // 0x0D, 0x0A
};
```

### STM32 → NUC (43字节)

```c
struct __packed RawGimbalToVision {
    uint8_t head[2];     // 0xA5, 0x5A
    uint8_t mode;
    float   q[4];        // 四元数 (q1,q2取反)
    float   yaw;         // YawTotalAngle (deg)
    float   yaw_vel;     // Gyro Z (deg/s)
    float   pitch;       // -IMU.Pitch (deg, 已取反)
    float   pitch_vel;   // -Gyro Y (deg/s, 已取反)
    uint8_t tail[2];     // 0x0D, 0x0A
};
```

### 模式值

- `mode=0`: 空闲
- `mode=1`: 控制云台
- `mode=2`: 控制云台并开火

---

## 四、STM32 控制架构

### 级联PID + 速度前馈

```
目标角度(deg) ─▶ [角度环PID] ─▶ 速度参考 + ff_vel ─▶ [速度环PID] ─▶ 电压
                  ↑ IMU角度反馈              ↑ 陀螺仪角速度反馈
```

前馈来源: NUC 在协议帧中填 yaw_vel/pitch_vel (非零时直接用; 全零时 STM32 差分估算)。

### 数据流

```
NUC帧 → serial_gimbal_cmd.c (解析+差分估算) → robot_cmd.c (转发) →
gimbal.c GimbalTask() (订阅) → DJIMotorControl() (叠加到速度环ref)
```

### PID 参数

**YAW GM6020:**

| 环 | Kp | Ki | Kd | MaxOut | 特殊 |
|----|----|----|----|---------|----|
| 角度环 | 8 | 0 | 0.3 | 350 deg/s | 微分先行+微分滤波(RC=0.005), 死区0.1° |
| 速度环 | 25 | 100 | 0 | 15000 | 积分限幅1500 |

**PITCH GM6020:**

| 环 | Kp | Ki | Kd | MaxOut | 特殊 |
|----|----|----|----|---------|----|
| 角度环 | 8 | 0 | 0.3 | 350 deg/s | 同YAW |
| 速度环 | 15 | 50 | 0 | 15000 | 积分限幅800 |

### 前馈参数

| 参数 | 值 | 说明 |
|------|----|------|
| 低通 alpha | 0.4 | 越大越快但越抖 (0.2~0.6) |
| 限幅 | ±300 deg/s | 防帧间跳变 |
| dt 有效范围 | 0.001s~0.1s | 超出置零 |

### 软件限位

YAW ±50°, PITCH ±35°

---

## 五、STM32 需改动的 3 个文件

### robot_def.h — Gimbal_Ctrl_Cmd_s 加前馈字段

```c
typedef struct {
    float yaw;
    float pitch;
    float yaw_vel;          // [新增] yaw 角速度前馈(deg/s)
    float yaw_acc;          // [新增] 恒=0
    float pitch_vel;        // [新增] pitch 角速度前馈(deg/s)
    float pitch_acc;        // [新增] 恒=0
    float chassis_rotate_wz;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;
```

> ⚠️ 改了结构体大小, 必须全部重编译!

### robot_cmd.c — SerialControlSet() 透传前馈

```c
static void SerialControlSet() {
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    if (serial_gimbal_data != NULL && serial_gimbal_data->online
        && serial_gimbal_data->mode != 0) {
        gimbal_cmd_send.yaw       = serial_gimbal_data->yaw;
        gimbal_cmd_send.pitch     = serial_gimbal_data->pitch;
        gimbal_cmd_send.yaw_vel   = serial_gimbal_data->yaw_vel;   // [新增]
        gimbal_cmd_send.pitch_vel = serial_gimbal_data->pitch_vel; // [新增]
    } else {
        gimbal_cmd_send.yaw_vel = gimbal_cmd_send.pitch_vel = 0;   // 离线清零
    }
}
```

### gimbal.c — 接入速度前馈

```c
// 文件顶部
static float nuc_yaw_vel_ff = 0.0f;
static float nuc_pitch_vel_ff = 0.0f;

// YAW/PITCH 电机初始化中:
.speed_feedforward_ptr = &nuc_yaw_vel_ff,   // 或 nuc_pitch_vel_ff
.feedforward_flag = SPEED_FEEDFORWARD,

// GimbalTask() 中:
nuc_yaw_vel_ff   = gimbal_cmd_recv.yaw_vel;
nuc_pitch_vel_ff = gimbal_cmd_recv.pitch_vel;
```

---

## 六、NUC 视觉端控制

### 激光补偿模型

```
激光总偏差 = boresight_deg (常数, 标定角度偏差)
           + atan(laser_offset / Z) (距离相关视差)

controller:
  目标角度 = atan((target_u - cx) / fx)      ← 相对光轴
  激光偏差 = boresight_deg + atan(offset/Z)  ← 常数项 + 视差项
  误差 = 目标角度 - 激光偏差
```

标定输出 4 个值到 `control.yaml`:
- `boresight_yaw_deg` / `boresight_pitch_deg` — 角度偏差 (距离无关)
- `laser_offset_x` / `laser_offset_y` — 物理偏移 (距离相关)

### 自学习

紫色=命中 → 记录角度误差 → EMA 修正 boresight_deg → 越用越准。
持久化到 `logs/bs_learned.yaml`, 重启自动加载。

### 控制参数 (control.yaml)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| kp | 3.0 | 主增益 |
| damping_kd | 0.008 | 阻尼, 抑制超调 |
| lowpass_alpha | 0.7 | 输出低通 |
| use_velocity_ff | 1 | 速度前馈开关 |
| ff_alpha | 0.5 | 前馈 EMA 平滑 |
| adaptive_kp | 1 | 远距离自动加大 kp |
| delay_compensate | 1 | 延迟补偿开关 |
| system_delay_ms | 10.0 | 系统延迟估计 |

---

## 七、反制机制

### 被瞄准进度 P

- **未被照射**: P 以 0.5/s 衰减, t 和 n 立即归零
- **被照射**: P 停止衰减, 每 0.1s 触发: P += n (n=1,2,3...)
- 照射不足 0.1s 中断: t、n 归零
- **结论**: 连续照射效率远高于间断照射

### 锁定效果

| 锁定次数 | P0 阈值 | 效果 | 难度变化 |
|---------|--------|------|---------|
| 第1次 | 50 | 发射机构锁定 45s | - |
| 第2次 | 100 | 锁定 45s | 检测区域缩至 3/5 |
| 第3次 | 100 | 锁定 45s | 检测区域缩至 1/5, 不再发光 |

每局最多反制 **3次**, 权重 **×50**。

### 激光发射装置

功率 < 1mW, 波段 ≤ 730nm, 仅 1 个, Class I/II。

---

## 八、联调流程

### Phase 1: 静态标定

1. 确认相机内参 (`camera.yaml`: fx, fy, cx, cy)
2. 运行 `laser_boresight` 多距离标定 → 写回 `control.yaml`
3. 确认 `boresight_yaw_deg`, `boresight_pitch_deg`, `laser_offset_x/y`

### Phase 2: 通信验证

1. NUC 运行 tracking, 终端输出 `DBG gimbal cmd: mode=1 yaw=xxx pitch=xxx`
2. 电控侧 J-Link Watch 确认值一致, mode≠0

### Phase 3: 关前馈调基础PID

1. `control.yaml` 设 `use_velocity_ff: 0`
2. NUC 锁静止目标, PlotJuggler 确认稳态误差 < 0.5°
3. 振荡→降 Kd; 慢→升 Kp

### Phase 4: 开前馈调运动跟踪

1. `use_velocity_ff: 1`, 确认 `feedforward_flag = SPEED_FEEDFORWARD`
2. 匀速运动目标, 对比有/无前馈的滞后
3. 超调→降 ff_alpha; 不够→升 ff_alpha

### Phase 5: 远距离验证

1. 20m/25m/30m 逐步测试
2. 开 `adaptive_kp`, 调 `system_delay_ms`
3. 启用自学习

### Phase 6: 安全验证

1. 急停 → 电机停止, 前馈清零
2. 拔 NUC 串口 → 不跳变
3. 重新插上 → 平滑恢复

---

## 九、比赛流程

- 三分钟准备: 运算平台端→台面, 传感器端→雷达基座, 遥控器放指定区域
- 云台仅在对方空中支援时上电
- 被判罚下: 激光+云台断电, 多机通信断开
- 运算平台端禁止无线设备, 操作系统必须开源 (Linux)

### 评分

| 项目 | 权重 |
|------|------|
| 累计准确标注时间 | ×1 |
| **反制无人机次数** | **×50** |
| 解析信息波难度等级 | ×50 |

---

## 十、常见问题

| 问题 | 排查 |
|------|------|
| 云台上电不动 | 检查遥控器模式; NUC mode 是否=0 |
| 云台剧烈震荡 | 降角度环 Kp/Kd; 检查前馈是否过大 |
| 跟踪有明显滞后 | 开前馈; 增大 kp; 开延迟补偿 |
| 远距离打不准 | 检查相机标定; 考虑换高分辨率相机 |
| NUC断开后云台跳变 | 检查 robot_cmd.c 离线角度同步 |
| 前馈始终为0 | Watch serial_cmd_data.yaw, 检查 NUC 角度是否在变 |
| 编译后数据错乱 | robot_def.h 改了结构体, 全部重编译 |
