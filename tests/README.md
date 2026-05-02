# 测试程序说明

`tests/` 是当前仓库里最常用的联调入口集合。核心库在 `src/`，但大多数单项验证程序都从这里构建。

构建命令请见顶层 [README.md](../README.md)。本页只保留测试程序的清单和各自用途。

## 当前可执行程序

| 程序 | 作用 | 依赖 |
|---|---|---|
| `serial` | 串口收发与协议联通性检查 | 串口 |
| `laser_rx` | 只跑 Layer 2，直接检测激光接收装置 | 相机或视频 + layer2 engine |
| `plane` | 只跑 Layer 1，无人机检测 | 相机或视频 + layer1 engine |
| `offset` | 模拟模块不发光时的偏移预测 | 相机或视频 + 级联模型 |
| `hit` | 全流程命中质量评估 | 相机/视频 + 级联模型 + 串口可选 |
| `color` | HSV 灯带 / 目标颜色过滤调试 | 相机或视频 |
| `latency` | 端到端延迟测量，给出 `system_delay_ms` 建议 | 相机/视频 + 串口可选 |
| `camera_demo` | 相机预览与 FPS 观测 | 相机 |
| `detect_demo` | 级联检测、跟踪、PlotJuggler、录制 | 相机或视频 + 级联模型 |
| `control_demo` | 给定单个像素点，查看控制器输出角度 | 配置文件 |

## 常用命令

### 相机

```bash
./tests/build/camera_demo --config config/camera.yaml --show
```

### 串口

```bash
./tests/build/serial \
  --port /dev/ttyUSB0 \
  --baud 115200 \
  --send-hz 100 \
  --stats
```

### Layer 1

```bash
./tests/build/plane \
  --config config/camera.yaml \
  --model src/detect/model/export/layer1_plane_fp16.engine
```

### Layer 2

```bash
./tests/build/laser_rx \
  --config config/camera.yaml \
  --model src/detect/model/export/layer2_laser_rx_fp16.engine
```

### 级联检测可视化

```bash
./tests/build/detect_demo \
  --config config/cascade.yaml \
  --enemy config/enemy.yaml \
  --control config/control.yaml \
  --record-dir records/test
```

### 跟踪照射

```bash
./src/build/tracking/tracking \
  --config config/camera.yaml \
  --model src/detect/model/export/layer2_laser_rx_fp16.engine \
  --control config/control.yaml \
  --port /dev/ttyUSB0 \
  --record-dir records/tracking
```

比赛录制模式：

```bash
./tests/build/detect_demo \
  --config config/cascade.yaml \
  --enemy config/enemy.yaml \
  --control config/control.yaml \
  --match \
  --record-dir records/match
```

### 延迟标定

```bash
./tests/build/latency \
  --config config/cascade.yaml \
  --enemy config/enemy.yaml \
  --camera config/camera.yaml \
  --control config/control.yaml \
  --port /dev/ttyUSB0
```

### 控制器单点验证

```bash
./tests/build/control_demo \
  --control-config config/control.yaml \
  --camera-config config/camera.yaml \
  --u 640 \
  --v 512 \
  --pitch 0 \
  --yaw 0
```

## 运行约定

- 文档默认你在仓库根目录运行这些程序
- 因为很多源码默认路径是按“在构建目录内启动”写的，所以从仓库根目录运行时最好总是显式传配置路径
- `detect_demo` 从仓库根目录运行时建议显式传 `--record-dir`，否则默认录制目录会按旧相对路径解析

## 模型与配置

默认示例都假设：

- `config/camera.yaml`
- `config/cascade.yaml`
- `config/control.yaml`
- `config/enemy.yaml`
- `src/detect/model/export/layer1_plane_fp16.engine`
- `src/detect/model/export/layer2_laser_rx_fp16.engine`

## 与主系统的区别

- `tests/` 侧重验证单模块或单链路
- `tracking/` 是平时调试用的独立入口，已经从 `tests/` 拆出来
- 完整系统入口在 `src/build/control/tracking_system`
- 校准入口在 `src/build/calibrate/laser_boresight`
