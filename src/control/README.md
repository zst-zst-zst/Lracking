# control

`src/control` 负责把视觉测量转成云台角指令，并提供当前主系统入口 `tracking_system`。

## 当前组成

- `src/config.cpp`
- `src/controller.cpp`
- `include/control/config.h`
- `include/control/controller.h`
- `examples/system_demo.cpp`

编译结果：

- 静态库 `control`
- 可执行程序 `tracking_system`

构建、主程序路径和完整启动命令请见顶层 [README.md](../../README.md)。本页只保留控制模块自己的状态机和参数说明。

## 单点控制验证

单点验证入口在 [tests/README.md](../../tests/README.md) 中统一说明。

## 当前控制逻辑

- 以相机中心为 boresight 参考点，结合激光偏移做补偿
- 通过 `atan(du / fx)`、`atan(dv / fy)` 把像素误差映射到角度误差
- 支持死区、滞回、输出低通
- 支持延迟补偿：利用跟踪器给出的像素速度做前视预测
- 支持阻尼反馈与角速度前馈
- 目标丢失时只做 yaw 方向的小幅微扫，pitch 保持最后一次有目标时的位置
- 启动阶段支持 `startup_*` 预备流程
- 自适应增益 `adaptive_kp` 可根据 bbox 面积调节 `kp`

## `control.yaml` 里当前最重要的参数

- `kp`
- `deadband_px`
- `max_angle_rate`
- `lowpass_alpha`
- `yaw_sign`
- `pitch_sign`
- `scan_*`
- `startup_*`
- `delay_compensate`
- `system_delay_ms`
- `adaptive_kp`

## 关于 `laser_offset_x/y`

当前主控制链路已经切到相机中心 + 激光偏移补偿，下面这些旧量只保留给测试程序和历史兼容：

- `laser_offset_x` / `laser_offset_y`
  - `laser_boresight` 当前会写回这两个字段
    - 当前主要被 `tracking`、`tests/hit.cpp`、`tests/detect_demo.cpp` 等程序使用

也就是说：

- `laser_boresight` 的当前输出已经不是旧文档里的“直接改 boresight 像素点”
- `tracking_system` 现在不再依赖 `u_L` / `v_L`，而是走相机中心 + 激光偏移补偿

## 日志

`tracking_system` 通过 `TeeBuf` 同时写终端和日志文件。推荐把日志显式写到：

```text
logs/tracking_system.log
```
