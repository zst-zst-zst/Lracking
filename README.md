# Tracking

RoboMaster 2026 雷达反制无人机 — 激光追踪视觉与控制工程。

## 构建

```bash
# 核心库 + tracking 入口
cmake -S src -B src/build
cmake --build src/build --target tracking -j$(nproc)

# 测试程序
cmake -S tests -B tests/build
cmake --build tests/build -j$(nproc)
```

## 运行

```bash
# 跟踪 (日常调试主入口)
./src/build/tracking/tracking \
  --config config/camera.yaml \
  --model src/detect/model/export/layer2_laser_rx_fp16.engine \
  --control config/control.yaml \
  --port /dev/ttyUSB0

# 完整系统 (比赛入口)
./src/build/control/tracking_system \
  --camera-config config/camera.yaml \
  --cascade-config config/cascade.yaml \
  --enemy config/enemy.yaml \
  --control-config config/control.yaml \
  --port /dev/ttyUSB0 --send-hz 100

# 激光-相机标定
./src/build/calibrate/laser_boresight \
  --config config/camera.yaml --control config/control.yaml --port /dev/ttyUSB0

# 标定 GUI
python3 tools/laser_calib_gui.py
```

### 测试程序

| 程序 | 作用 |
|------|------|
| `serial` | 串口收发与协议联通性 |
| `camera_demo` | 相机预览与 FPS |
| `detect_demo` | 级联检测 + 跟踪 + PlotJuggler + 录制 |
| `control_demo` | 给定像素点查看控制器输出 |
| `latency` | 端到端延迟测量 |
| `hit` | 全流程命中质量评估 |
| `plane` / `laser_rx` | Layer 1 / Layer 2 单层检测 |
| `color` | HSV 颜色过滤调试 |
| `offset` | 偏移预测模拟 |

```bash
# 示例
./tests/build/serial --port /dev/ttyUSB0 --baud 115200 --send-hz 100 --stats
./tests/build/camera_demo --config config/camera.yaml --show
./tests/build/detect_demo --config config/cascade.yaml --enemy config/enemy.yaml --control config/control.yaml
./tests/build/latency --config config/cascade.yaml --enemy config/enemy.yaml --camera config/camera.yaml --control config/control.yaml --port /dev/ttyUSB0
./tests/build/control_demo --control-config config/control.yaml --camera-config config/camera.yaml --u 640 --v 512
```

> 建议从仓库根目录运行，并显式传所有配置路径。

## 模块

| 模块 | 职责 |
|------|------|
| `src/camera` | 大恒 Galaxy 相机驱动 (CPU/CUDA/NPP 去马赛克) |
| `src/detect` | 两层级联检测 (TRTInferX) |
| `src/solve` | Kalman + IoU 跟踪, 比赛分析器 |
| `src/control` | 像素误差→云台角指令, 完整系统入口 `tracking_system` |
| `src/gimbal` | 串口协议编解码与收发 |
| `src/common` | 共享类型、时间工具、日志 tee |
| `src/plotter` | UDP JSON → PlotJuggler |
| `src/calibrate` | 激光-相机标定 `laser_boresight` |
| `tracking` | Layer 2 跟踪 + 云台补偿 + 轨迹渲染 (日常调试入口) |
| `tools` | `laser_calib_gui.py` PyQt5 标定前端 |

## 配置文件

| 文件 | 作用 | 何时改 |
|------|------|--------|
| `config/camera.yaml` | 相机内参、曝光、ROI、GPU 管线 | 换相机/镜头/光照 |
| `config/cascade.yaml` | 级联检测、跟踪、比赛分析 | 调检测/跟踪 |
| `config/control.yaml` | 控制器、激光补偿、搜索策略 | 调控制或标定后 |
| `config/enemy.yaml` | 敌方颜色、侧面、稳定性预设 | **每场比赛前** |

## 依赖

CMake ≥ 3.10, C++17, OpenCV, CUDA Toolkit, TensorRT, `nlohmann-json3-dev`, `opencv_freetype`

相机 SDK 已内置: `src/camera/galaxy_sdk`

## 文档

- [docs/system.md](docs/system.md) — 系统架构、通信协议、STM32、PID、比赛规则
- [docs/tuning.md](docs/tuning.md) — 调参指南 (PlotJuggler + 远距离 + 自适应)
- [docs/calibration.md](docs/calibration.md) — 标定流程与配置说明
- [docs/detection.md](docs/detection.md) — 检测流水线、训练脚本、数据采集
- [docs/sp_vision_analysis.md](docs/sp_vision_analysis.md) — sp_vision 架构分析与补偿方案对比
