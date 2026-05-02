# NUC↔STM32 联调文档 (给电控)

## 一、坐标系说明

### 1.1 双方使用的坐标系

```
NUC 和 STM32 之间用的是同一个坐标系:

YAW:   IMU 的 YawTotalAngle (度, 累积量, 无限制)
PITCH: -IMU.Pitch (度, 电控在发送时取反了)
```

### 1.2 为什么 pitch 要取反?

```
IMU 原始 Pitch: 枪口抬起 → 正值
相机坐标系:     目标在画面下方 → v增大 → 枪口应该抬起

电控发送时: serial_gimbal_send.pitch = -IMU.Pitch
NUC 收到后在这个取反坐标系里计算
NUC 发回的值也是取反坐标系的

电控收到后: SetRef(pitch_motor, cmd.pitch)
pitch 电机配置了 FEEDBACK_DIRECTION_REVERSE
→ PID 内部: error = ref - (-feedback) → 自洽 ✅
```

### 1.3 值的范围

| 字段 | 坐标系 | 范围 | 单位 |
|------|--------|------|------|
| yaw | IMU.YawTotalAngle | ±50° (软件限位) | deg |
| pitch | -IMU.Pitch | ±35° (软件限位) | deg |
| yaw_vel | 目标运动角速度 | ±150 | deg/s |
| pitch_vel | 目标运动角速度 | ±150 | deg/s |
| yaw_acc | **不使用, 恒=0** | 0 | deg/s² |
| pitch_acc | **不使用, 恒=0** | 0 | deg/s² |

---

## 二、串口协议帧格式

### 2.1 NUC → STM32 (29字节)

```c
// NUC 发给电控的帧 (视觉→云台)
struct __packed RawVisionToGimbal {
    uint8_t head[2];     // 0xA5, 0x5A
    uint8_t mode;        // 0=不控制, 1=控制不开火, 2=控制且开火
    float   yaw;         // 绝对目标角度 (deg), 坐标系=YawTotalAngle
    float   yaw_vel;     // 速度前馈 (deg/s), 电控叠加到速度环
    float   yaw_acc;     // 恒=0 (不使用)
    float   pitch;       // 绝对目标角度 (deg), 坐标系=-IMU.Pitch
    float   pitch_vel;   // 速度前馈 (deg/s), 电控叠加到速度环
    float   pitch_acc;   // 恒=0 (不使用)
    uint8_t tail[2];     // 0x0D, 0x0A
};
// sizeof = 2+1+4+4+4+4+4+4+2 = 29 字节
```

### 2.2 STM32 → NUC (43字节)

```c
// 电控发给 NUC 的帧 (云台→视觉)
struct __packed RawGimbalToVision {
    uint8_t head[2];     // 0xA5, 0x5A
    uint8_t mode;        // 当前模式
    float   q[4];        // 四元数 (注意 q1,q2 取反了)
    float   yaw;         // YawTotalAngle (deg)
    float   yaw_vel;     // Gyro Z (deg/s)
    float   pitch;       // -IMU.Pitch (deg, 已取反!)
    float   pitch_vel;   // -Gyro Y (deg/s, 已取反!)
    uint8_t tail[2];     // 0x0D, 0x0A
};
// sizeof = 2+1+16+4+4+4+4+4+4+2 = 43 字节
// (实际还有 bullet_speed, bullet_count 等, 暂为0)
```

---

## 三、STM32 需要改的文件 (共3个)

### 3.1 robot_def.h — 加前馈字段

```c
// 文件: application/robot_def.h
// 修改: Gimbal_Ctrl_Cmd_s 结构体, 加4个前馈字段

typedef struct
{
    float yaw;
    float pitch;
    float yaw_vel;          // [新增] yaw 角速度前馈(deg/s), 来自NUC
    float yaw_acc;          // [新增] yaw 角加速度前馈(deg/s²), 恒=0
    float pitch_vel;        // [新增] pitch 角速度前馈(deg/s), 来自NUC
    float pitch_acc;        // [新增] pitch 角加速度前馈(deg/s²), 恒=0
    float chassis_rotate_wz;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;
```

### 3.2 robot_cmd.c — 透传前馈值

```c
// 文件: application/cmd/robot_cmd.c
// 修改: SerialControlSet() 函数

static void SerialControlSet()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;

    if (serial_gimbal_data != NULL && serial_gimbal_data->online
        && serial_gimbal_data->mode != 0)
    {
        // NUC 直接发度制绝对目标角度
        gimbal_cmd_send.yaw       = serial_gimbal_data->yaw;
        gimbal_cmd_send.pitch     = serial_gimbal_data->pitch;
        // [新增] 前馈速度/加速度, 帮助PID提前预判
        gimbal_cmd_send.yaw_vel   = serial_gimbal_data->yaw_vel;
        gimbal_cmd_send.yaw_acc   = serial_gimbal_data->yaw_acc;
        gimbal_cmd_send.pitch_vel = serial_gimbal_data->pitch_vel;
        gimbal_cmd_send.pitch_acc = serial_gimbal_data->pitch_acc;
    }
    else
    {
        // NUC 离线或 mode==0: 清零前馈, 防残留
        gimbal_cmd_send.yaw_vel   = 0;
        gimbal_cmd_send.yaw_acc   = 0;
        gimbal_cmd_send.pitch_vel = 0;
        gimbal_cmd_send.pitch_acc = 0;
    }

    // 软件限位 (保持不变)
    if (gimbal_cmd_send.yaw > YAW_MAX_ANGLE)   gimbal_cmd_send.yaw = YAW_MAX_ANGLE;
    if (gimbal_cmd_send.yaw < YAW_MIN_ANGLE)   gimbal_cmd_send.yaw = YAW_MIN_ANGLE;
    if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE) gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
    if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE) gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
}
```

### 3.3 gimbal.c — 接入速度前馈

```c
// 文件: application/gimbal/gimbal.c

// ======== 第1处: 文件顶部加两个静态变量 ========
static float nuc_yaw_vel_ff = 0.0f;    // NUC yaw 速度前馈
static float nuc_pitch_vel_ff = 0.0f;  // NUC pitch 速度前馈

// ======== 第2处: YAW 电机初始化, 加前馈指针 ========
// yaw_config.controller_param_init_config 中:
.speed_feedforward_ptr = &nuc_yaw_vel_ff,  // [新增] NUC角速度前馈

// yaw_config.controller_setting_init_config 中:
.feedforward_flag = SPEED_FEEDFORWARD,     // [新增] 启用速度前馈

// ======== 第3处: PITCH 电机初始化, 加前馈指针 ========
// pitch_config.controller_param_init_config 中:
.speed_feedforward_ptr = &nuc_pitch_vel_ff,  // [新增] NUC角速度前馈

// pitch_config.controller_setting_init_config 中:
.feedforward_flag = SPEED_FEEDFORWARD,       // [新增] 启用速度前馈

// ======== 第4处: GimbalTask() 中更新前馈值 ========
void GimbalTask()
{
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    gimbal_gyro_dps_yaw = (float)GYRO2GIMBAL_DIR_YAW * gimba_IMU_data->Gyro[2] * RAD2DEG;
    gimbal_gyro_dps_pitch = (float)GYRO2GIMBAL_DIR_PITCH * gimba_IMU_data->Gyro[1] * RAD2DEG;

    // [新增] 更新NUC前馈速度
    nuc_yaw_vel_ff = gimbal_cmd_recv.yaw_vel;
    nuc_pitch_vel_ff = gimbal_cmd_recv.pitch_vel;

    // ... 后面不变 ...
}
```

---

## 四、速度前馈工作原理

```
正常 PID (无前馈):
  角度环: speed_ref = Kp_angle × (target_angle - current_angle)
  速度环: voltage  = Kp_speed × (speed_ref - gyro_speed)
  → 必须等误差产生才反应, 有滞后

加速度前馈后:
  角度环: speed_ref = Kp_angle × (target_angle - current_angle)
  速度环: voltage  = Kp_speed × (speed_ref + nuc_vel_ff - gyro_speed)
                                            ^^^^^^^^^^^^^^^^
                                  NUC告诉电机"目标正在以这个速度运动"
                                  电机提前加速, 减少追踪滞后

dji_motor.c 中的实现 (已有, 不需要改):
  if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
      pid_ref += *motor_controller->speed_feedforward_ptr;
```

### 前馈值的含义

```
nuc_yaw_vel_ff = +10 deg/s
→ 目标正在向右运动 (yaw增大方向)
→ 电机提前往右加速 10 deg/s
→ 减少跟踪滞后

nuc_pitch_vel_ff = -5 deg/s
→ 目标正在向上运动 (pitch减小方向, 取反坐标系)
→ 电机提前调整

目标静止时: yaw_vel = pitch_vel ≈ 0
NUC 离线时: yaw_vel = pitch_vel = 0 (已在 robot_cmd.c 清零)
```

---

## 五、联调步骤

### 第1步: 不开前馈, 验证角度跟踪

```
NUC端: control.yaml 设置 use_velocity_ff: 0
电控端: gimbal.c 中 feedforward_flag = FEEDFORWARD_NONE (或不改, 前馈=0也没影响)

测试:
1. NUC 锁定静止目标
2. 看 PlotJuggler: cmd_yaw vs state_yaw
3. 误差应该 < 0.5° 且稳定不振荡
4. 如果振荡: 降低角度环 Kp (当前=8)
```

### 第2步: 开启前馈, 验证运动跟踪

```
NUC端: control.yaml 设置 use_velocity_ff: 1
电控端: 确认 feedforward_flag = SPEED_FEEDFORWARD

测试:
1. 目标匀速运动
2. 看终端输出 yaw_vel/pitch_vel 值是否合理 (匀速运动应为常数)
3. 对比开/关前馈的跟踪滞后
4. 如果前馈太大导致超调: NUC端调小 ff_alpha (0.5→0.3)
```

### 第3步: 验证急停和离线安全

```
1. 拨轮急停 → 电机应立即停止, 前馈清零
2. 拔掉NUC串口线 → 电机应保持当前位置不跳变, 前馈清零
3. 重新插上 → 应平滑恢复控制
```

---

## 六、当前 PID 参数参考

```
YAW GM6020:
  角度环: Kp=8, Ki=0, Kd=0, MaxOut=200 (deg/s)
  速度环: Kp=25, Ki=100, MaxOut=15000
  反馈: IMU.YawTotalAngle + Gyro[2]*RAD2DEG

PITCH GM6020:
  角度环: Kp=8, Ki=0, Kd=0, MaxOut=200 (deg/s)
  速度环: Kp=15, Ki=50, MaxOut=15000
  反馈: IMU.Pitch + Gyro[1]*RAD2DEG (FEEDBACK_REVERSE)
```

### 远距离(25-30m)调参建议

```
目标: 50mm×50mm, 30m处仅2.16像素, 容差±0.91px ≈ ±0.04°

角度环 Kp=8 → 对 0.04° 的误差产生 0.32 deg/s 的速度指令
→ 需要确认电机在低速时的死区和分辨率
→ 如果低速跟不上, 可能需要加大 Kp 或减小速度环死区

速度前馈的作用:
  目标 1m/s 运动, 30m 处角速度 ≈ 1.9 deg/s
  前馈直接给速度环 +1.9 deg/s, 不需要等角度环慢慢积累
  → 大幅减少运动目标的跟踪滞后
```

---

## 七、改动文件清单

| 文件 | 改动内容 | 风险 |
|------|----------|------|
| `robot_def.h` | Gimbal_Ctrl_Cmd_s 加4个float字段 | ⚠️ 改了结构体大小, 相关模块需重编译 |
| `robot_cmd.c` | SerialControlSet() 透传前馈 | 低风险, NUC离线时清零 |
| `gimbal.c` | 加2个static变量, 初始化配前馈指针, GimbalTask更新值 | 低风险, dji_motor.c 已支持 |

**注意**: `robot_def.h` 改了结构体布局, 必须**全部重新编译**, 否则字段偏移不对会导致数据错乱!
