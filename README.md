# Tracking

面向 RoboMaster 反无人机激光追踪场景的视觉与控制工程。

当前仓库的两个有效 CMake 入口是：

- `src/`：核心库、主系统、校准工具
- `tests/`：模块联调程序、可视化 demo、延迟与命中测试
- `tracking/`：平时调试的独立跟踪入口，和 `src/` 并列

仓库里可能同时存在 `build/`、`src/build/`、`tests/build/` 等历史构建产物。实际怎么构建，请看各模块自己的 README；总 README 不再放分开的构建命令。

## 当前模块

- `src/camera`：大恒 Galaxy 相机驱动，支持 CPU / CUDA / NPP 去马赛克，配置来自 `config/camera.yaml`
- `src/detect`：两层级联检测，底层使用 `TRTInferX`
- `src/solve`：Kalman + IoU 跟踪，外加比赛分析器
- `src/control`：像素误差到云台角指令的控制器，以及完整系统入口 `tracking_system`
- `src/gimbal`：串口协议编解码与收发
- `src/common`：共享类型、时间工具、日志 tee、少量配置读取
- `src/plotter`：UDP JSON 到 PlotJuggler
- `src/calibrate`：激光-相机标定工具 `laser_boresight`
- `tracking`：Layer 2 跟踪、云台补偿、轨迹渲染、可选录制
- `src/thirdparty`：可选 ONNX Runtime 预构建包
- `tools/laser_calib_gui.py`：`laser_boresight` 的 PyQt5 前端

## 目录概览

```text
Tracking/
├── config/
│   ├── camera.yaml
│   ├── cascade.yaml
│   ├── control.yaml
│   ├── enemy.yaml
│   └── README.md
├── docs/
├── src/
│   ├── CMakeLists.txt
│   ├── calibrate/
│   ├── camera/
│   ├── common/
│   ├── control/
│   ├── detect/
│   ├── gimbal/
│   ├── plotter/
│   ├── solve/
│   ├── thirdparty/
│   └── log/
├── tracking/
│   ├── CMakeLists.txt
│   └── README.md
├── tests/
│   ├── CMakeLists.txt
│   ├── camera_demo.cpp
│   ├── control_demo.cpp
│   ├── detect_demo.cpp
│   ├── latency.cpp
│   ├── serial.cpp
│   └── ...
└── tools/
    └── laser_calib_gui.py
```

## 依赖

默认源码状态下需要：

- CMake >= 3.10
- C++17 编译器
- OpenCV
- CUDA Toolkit
- TensorRT
- `nlohmann-json3-dev`
- `opencv_freetype`

相机部分默认直接使用仓库内置 SDK：`src/camera/galaxy_sdk`。


## 在主工程里构建

通常不需要单独进这个目录，直接在仓库根目录执行。

## 完整系统

```bash
./src/build/control/tracking_system \
  --camera-config config/camera.yaml \
  --cascade-config config/cascade.yaml \
  --enemy config/enemy.yaml \
  --control-config config/control.yaml \
  --port /dev/ttyUSB0 \
  --send-hz 100 \
  --log-file logs/tracking_system.log
```

说明：

- 传入 `--enemy config/enemy.yaml` 时，`tracking_system` 会进入“比赛参数覆盖”路径
- 如果从仓库根目录启动，建议总是显式传 `--camera-config` / `--cascade-config` / `--control-config` / `--log-file`

## 校准

CLI：

```bash
./src/build/calibrate/laser_boresight \
  --config config/camera.yaml \
  --control config/control.yaml \
  --port /dev/ttyUSB0
```

GUI：

```bash
python3 tools/laser_calib_gui.py
```

当前校准工具的实际行为：

- 主要写回 `config/control.yaml` 中的 `laser_offset_x` / `laser_offset_y`
- 会把会话进度写到 `logs/laser_calib_session.yaml`
- 会把曝光缓存写到 `config/.boresight_exposure`

注意：

- `control.yaml` 里的 `u_L` / `v_L` 仍然被 `Controller` 与部分可视化程序使用
- 但 `laser_boresight` 现在保存的主结果不是 `u_L` / `v_L`，而是物理偏移量

## 赛前最常改的配置

- `config/enemy.yaml`：每场前必改，包含 `enemy_color`、`enemy_side`、`preset_stability_tier`
- `config/camera.yaml`：曝光、ROI、GPU 管线、标定文件路径
- `config/cascade.yaml`：模型路径、级联裁剪、灯带拒绝、跟踪器、比赛分析参数
- `config/control.yaml`：控制参数、搜索策略、`system_delay_ms`、`u_L/v_L`、`laser_offset_x/y`

## 相关 README

- [config/README.md](config/README.md)
- [tests/README.md](tests/README.md)
- [src/camera/README.md](src/camera/README.md)
- [src/detect/README.md](src/detect/README.md)
- [src/control/README.md](src/control/README.md)
- [src/calibrate/README.md](src/calibrate/README.md)
