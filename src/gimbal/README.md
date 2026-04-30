# gimbal_serial

云台串口收发已经切到你提供的 `gimbal (1).cpp/.hpp` 协议：

- 回传帧：43B，帧头 `0x5A 0xA5`，帧尾 `0x7F 0xFE`
- 下发帧：29B，帧头 `0x5A 0xA5`，帧尾 `0x7F 0xFE`
- 线缆协议单位：弧度 / 弧度每秒 / 弧度每秒平方
- 系统内部控制单位仍保留为 `deg / deg/s / deg/s^2`，在串口层自动换算

## 上下游接口

- 上游：`common::GimbalCommand`
- 输出：`common::GimbalState`
- 串口层会把上游的绝对角度指令转换成“相对当前云台角度的增量”，与给电控的代码保持一致

## 当前字段

- `common::GimbalState`
  - `pitch / yaw / pitch_rate / yaw_rate`
  - `mode`
  - `quaternion_wxyz`
  - `timestamp`：主机接收时间戳
- `common::GimbalCommand`
  - `pitch / yaw / pitch_rate / yaw_rate`
  - `pitch_acc / yaw_acc`
  - `mode`

## 模式约定

- 下发 `mode=0`：不控制
- 下发 `mode=1`：控制云台，不开火
- 下发 `mode=2`：控制云台，开火
- 回传 `mode=0/1/2/3`：对应 `IDLE / AUTO_AIM / SMALL_BUFF / BIG_BUFF`
- 当前 `control_system_demo` 主链路实际发送 `mode=1`
- 当电控未上电或长时间无回传时，系统仍会继续下发；只是下发增量的参考姿态会冻结在最后一次有效绝对姿态，或者启动 home 姿态

## 运行

```bash
cd /home/zst/Tracking
./build/serial --port /dev/ttyUSB0 --baud 115200 --pitch 1.0 --yaw -1.0 --mode 1 --send-hz 300
```

## 参数说明

- `--port` 串口设备名，例如 `/dev/ttyACM0`
- `--baud` 波特率，默认 `115200`
- `--pitch` 目标绝对 pitch，单位 `deg`
- `--yaw` 目标绝对 yaw，单位 `deg`
- `--mode` 下发模式，`0/1/2`
- `--send-hz` 发送频率
- `--stats` 打印收发统计
- `--dump-hex` 打印原始回传字节
- `--reconnect` 断线重连开关
- `--reconnect-delay-ms` 重连间隔
- `--read-timeout-ms` 串口读超时

## 帧字段

回传 43B：

- `0-1`：帧头 `5A A5`
- `2`：mode
- `3-18`：四元数 `q[4]`，顺序 `wxyz`
- `19-22`：yaw
- `23-26`：yaw_vel
- `27-30`：pitch
- `31-34`：pitch_vel
- `35-38`：bullet_speed
- `39-40`：bullet_count
- `41-42`：帧尾 `7F FE`

说明：`bullet_speed / bullet_count` 字节仍保留在回传协议里，但当前视觉侧固定忽略，按 `0` 处理。
说明：`q[4]` 四元数字节仍保留在回传协议里，但当前视觉侧固定按单位四元数 `{1,0,0,0}` 处理。

下发 29B：

- `0-1`：帧头 `5A A5`
- `2`：mode
- `3-6`：yaw 增量
- `7-10`：yaw_vel
- `11-14`：yaw_acc
- `15-18`：pitch 增量
- `19-22`：pitch_vel
- `23-26`：pitch_acc
- `27-28`：帧尾 `7F FE`
