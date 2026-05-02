# tracking

`tracking` 是当前平时调试最直接的入口，和 `src/` 里的比赛主系统并列，而不是放在 `tests/` 里一起混用。

它保留了跟踪照射入口的核心链路：

- Layer 2 检测
- Kalman + IoU 跟踪
- 云台补偿
- 实时轨迹叠加
- 可选视频录制

当前串口协议里，`tracking` 发给电控的 `yaw` / `pitch` 是绝对角度，单位为度；电控回传的 `yaw` / `pitch` 也按度读取。

常用命令请看顶层 [README.md](../README.md) 和 [tests/README.md](../tests/README.md)。
