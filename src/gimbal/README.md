# gimbal_serial

`src/gimbal` 提供当前主链路使用的串口协议、收发封装与帧编解码。

## 当前结构

- `include/gimbal_serial/protocol.h`
- `include/gimbal_serial/serial_port.h`
- `src/protocol.cpp`
- `src/serial_port.cpp`

编译后生成静态库：

- `gimbal_serial`

## 测试入口

串口联调程序已经移到：

- `tests/serial.cpp`

构建并运行：

```bash
cmake -S tests -B tests/build
cmake --build tests/build -j"$(nproc)"

./tests/build/serial \
  --port /dev/ttyUSB0 \
  --baud 115200 \
  --send-hz 100 \
  --stats
```

## 当前协议约定

### 回传帧

- 长度：43 字节
- 帧头：`0x5A 0xA5`
- 帧尾：`0x7F 0xFE`

字段包括：

- `mode`
- `quaternion_wxyz`
- `yaw` / `yaw_rate`
- `pitch` / `pitch_rate`
- `bullet_speed`
- `bullet_count`

### 下发帧

- 长度：29 字节
- 帧头：`0x5A 0xA5`
- 帧尾：`0x7F 0xFE`

字段包括：

- `mode`
- `yaw`
- `yaw_rate`
- `yaw_acc`
- `pitch`
- `pitch_rate`
- `pitch_acc`

## 单位

- 视觉与控制内部：`deg` / `deg/s` / `deg/s^2`
- 线缆协议：度制 / 度每秒 / 度每秒平方
- 协议层统一使用度制，不做弧度换算

## 当前行为

- `packGimbalCommand()` 直接下发绝对角指令（度）
- 如果没有回传状态，`Idle` 模式下会用当前状态保持发包角度
- 回传帧里的 `yaw` / `pitch` / `yaw_rate` / `pitch_rate` 也按度制读取
- `tests/build/serial` 支持 `--dump-hex`、`--reconnect`、`--read-timeout-ms` 等调试参数

## 模式值

- 下发 `mode=0`：空闲
- 下发 `mode=1`：控制云台
- 下发 `mode=2`：控制云台并开火

当前 `Controller` 默认输出 `mode=1`。
