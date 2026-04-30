# Tracking

- **电控部分 [二轴云台控制开源](https://github.com/NCST-Horizon-RM/Horizon_frame_f1)**


- `detect`：**感知层** — 两层级联 YOLO 检测 + TRTInferX 推理引擎 + ONNX 备用推理。
- `solve`：**估算层** — Kalman + IoU 目标跟踪 + 比赛分析器（P 值 / 稳定性分级 / 飞行阶段）。
- `control`：**执行层** — 像素误差 → 云台角度指令，包含丢失目标搜索策略。
- `common`：统一数据结构与配置读取（`TargetMeasurement` / `GimbalState` / `TeeBuf`）。
- `camera`：Galaxy 相机驱动，支持 CPU/GPU 去马赛克与零拷贝。
- `gimbal`：串口收发云台状态与指令。
- `plotter`：UDP JSON → PlotJuggler 实时数据可视化。
- `boresight`：激光-相机同轴校准工具（生成 `u_L/v_L`）。
- `parallax`：视差距离估算工具。

---

## 工程结构

```
Tracking/
├── config/                              # 顶层配置
│   ├── enemy.yaml                       # ★赛前必改★ 敌方颜色/预设档位/曝光
│   ├── camera.yaml                      # 相机参数 + compute_mode (gpu|cpu)
│   └── control.yaml                     # PID/延迟补偿/搜索/启动策略
├── tests/                               # 顶层测试程序
│   ├── serial.cpp                       # 串口联通性
│   ├── laser_rx.cpp                     # 激光接收装置识别
│   ├── plane.cpp                        # 无人机识别
│   ├── cascade.cpp                      # 级联检测
│   ├── track.cpp                        # 跟踪照射
│   ├── offset.cpp                       # 偏移推断
│   ├── hit.cpp                          # 打中测试
│   ├── color.cpp                        # 颜色过滤
│   ├── latency.cpp                      # 延迟测量
│   └── CMakeLists.txt
├── docs/                                # 设计文档
│   ├── sp_vision_analysis.md            # sp_vision 架构分析 & 控制逻辑对比
│   └── plotjuggler_tuning_guide.md      # PlotJuggler 调参指南
├── records/                             # 自动录制视频 (gitignored)
│   ├── match/                           # 比赛录制 (全分辨率)
│   └── test/                            # 测试录制 (半分辨率)
└── src/
    ├── CMakeLists.txt                   # 顶层构建入口
    ├── detect/                          # ★感知层★ 级联 YOLO 检测
    │   ├── cascade_detector.h/cpp       #   两层级联: YOLO(plane)→YOLO(laser_rx)→灯带拒绝
    │   ├── detector.h                   #   Detection/Detector 基础接口
    │   ├── measurement.h/cpp            #   Detection → TargetMeasurement 转换
    │   ├── onnx_detector.h/cpp          #   ONNX Runtime 备用推理 (开发机)
    │   ├── demo.cpp                     #   级联检测 demo (可视化 + 录制)
    │   ├── TRTInferX/                   #   TensorRT YOLO 推理引擎
    │   ├── config/                      #   cascade.yaml
    │   ├── scripts/                     #   训练 / 标注 / 导出 / 验证
    │   ├── model/                       #   TRT/ONNX 模型
    │   ├── plane/ laser_rx/ dataset/    #   数据集
    │   └── CMakeLists.txt
    ├── solve/                           # ★估算层★ 跟踪 + 比赛分析
    │   ├── target_tracker.h/cpp         #   Kalman + IoU 多目标跟踪
    │   ├── match_analyzer.h/cpp         #   P值/稳定性/飞行阶段/瞄准策略
    │   └── CMakeLists.txt
    ├── control/                         # ★执行层★ PID 控制 + 系统集成
    │   ├── src/                         #   controller + config
    │   └── examples/system_demo.cpp     #   完整系统 (相机+检测+控制+云台)
    ├── common/                          # 共享类型 + TeeBuf + 时间工具
    ├── camera/                          # Galaxy 相机驱动 (CPU/GPU)
    ├── gimbal/                          # 云台串口收发
    ├── plotter/                         # UDP JSON → PlotJuggler
    ├── boresight/                       # 激光-相机同轴校准工具
    ├── parallax/                        # 视差估算工具
    └── thirdparty/                      # ONNX Runtime 预构建 (可选)
```


### 开发环境

| 项目 | 规格 |
|------|------|
| 主机 | HP OMEN 16 Gaming Laptop |
| CPU | Intel Core i9-14900HX |
| GPU | NVIDIA GeForce RTX 4060 Laptop (8 GB) |
| OS | Ubuntu 24.04 LTS (kernel 6.17) |
| CUDA | 13.1 (Driver 590.48) |
| TensorRT | 10.14.1 |
| OpenCV | 4.13.0 |


## 构建

以下命令均在**项目根目录**（`~/Tracking`）执行：

```bash
cmake -S src -B src/build
cmake --build src/build -j$(nproc)
```

构建后可执行文件位于：

| 文件 | 用途 |
|------|------|
| `src/build/detect/detect_demo` | 级联检测 demo（轨迹可视化 + 录制 + PlotJuggler） |
| `src/build/control/control_system_demo` | 完整系统（相机+检测+控制+串口） |
| `src/build/camera/galaxy_camera_demo` | 相机单独测试 |
| `build/serial` | 云台串口收发测试 |
| `src/build/boresight/boresight` | 激光-相机同轴校准工具 |

---

## 测试命令

> 所有命令均在项目根目录 `~/Tracking` 下执行。

### 相机

```bash
# 预览画面，验证帧率与图像质量
./src/build/camera/galaxy_camera_demo \
  --config config/camera.yaml --show
```

### 级联检测 demo

```bash
# 使用 Galaxy 相机
./src/build/detect/detect_demo \
  --config src/detect/config/cascade.yaml \
  --camera config/camera.yaml

# 使用视频文件
./src/build/detect/detect_demo \
  --config src/detect/config/cascade.yaml \
  --video /path/to/video.mp4

# 比赛模式 (全分辨率录制)
./src/build/detect/detect_demo \
  --config src/detect/config/cascade.yaml \
  --camera config/camera.yaml \
  --enemy config/enemy.yaml
```

### 云台串口

```bash
# 测试收发，确认帧格式与频率（--dump-hex 打印原始字节）
./build/serial \
  --port /dev/ttyUSB0 --baud 115200 \
  --send-hz 100 --stats --dump-hex
```

### 完整系统（相机 + 检测 + 控制 + 云台）

```bash
./src/build/control/control_system_demo \
  --camera config/camera.yaml \
  --cascade src/detect/config/cascade.yaml \
  --control src/control/config/control.yaml \
  --port /dev/ttyACM0 --send-hz 400
```

---

## 日志

`control_system_demo` 会自动将终端输出追加写入：

```
src/log/log.txt
```

每次启动会在日志中追加空行与时间戳。

---

## 同轴校准（boresight）

激光点与相机光轴有偏差时，需先用此工具确定 `u_L/v_L`（激光在图像中的像素坐标），再写回 `control.yaml`。

```bash
./src/build/boresight/boresight \
  --camera config/camera.yaml \
  --config src/detect/config/cascade.yaml \
  --enemy config/enemy.yaml \
  --boresight-out boresight.yaml
```

校准流程：
1. 固定目标，让检测框稳定。
2. 用界面微调红色十字到激光点位置。
3. 保存后将 `boresight_out.yaml` 中的 `u_L/v_L` 填入 `src/control/config/control.yaml`。

> `u_L/v_L` 对目标距离敏感，建议在常用工作距离下校准。

---

## 关键配置

| 文件 | 修改频率 | 内容 |
|------|---------|------|
| `config/enemy.yaml` | **每场比赛前** | 敌方颜色、预设档位、曝光参数 |
| `config/camera.yaml` | 偶尔 | 相机 ROI、GPU 管线、`compute_mode` |
| `src/detect/config/cascade.yaml` | 平时调参 | 级联检测 / 跟踪 / 比赛分析详细参数 |
| `src/control/config/control.yaml` | 标定后 | PID 增益、死区、搜索策略、`u_L/v_L` |

**加载顺序**: `cascade.yaml`(默认值) → `enemy.yaml`(赛前参数覆盖)

---

## 常见问题

- **可执行文件找不到**：先执行 `cmake --build src/build -j$(nproc)` 完整构建。
- **串口打不开**：`ls /dev/ttyACM*` 确认设备名；`sudo chmod 666 /dev/ttyACM0` 或加入 `dialout` 组。
- **没有检测到目标**：用 `detect_demo` 配合测试视频先验证检测器，再联调整机系统。
- **相机帧率低**：检查 `config/camera.yaml` 中 `frame_rate` 与 `roi_enable`；确保 USB 3.0 接口。


---
## 线程模型参考

<p align="center">
  <img src="docs/Thread-Model.png"
       alt="Thread Model"
       height="">
</p>
<p align="center"><em>Thread Model Overview</em></p>

注：提供系统线程模型供分析参考

---

## 测试程序

位于 `tests/`，用 `cmake -S tests -B tests/build && cmake --build tests/build -j$(nproc)` 构建。

| # | 文件 | 功能 |
|---|------|------|
| 1 | `serial.cpp` | 串口联通性测试 |
| 2 | `laser_rx.cpp` | 激光接收装置识别（Layer 2 单独） |
| 3 | `plane.cpp` | 无人机识别（Layer 1 单独） |
| 4 | `cascade.cpp` | 级联检测（Layer 1 → Layer 2 → 过滤） |
| 5 | `track.cpp` | 跟踪 + 照射（级联 + Kalman + 云台） |
| 6 | `offset.cpp` | 偏移推断（激光未发射时预测落点） |
| 7 | `hit.cpp` | 打中测试（全链路：检测→跟踪→控制→评估） |
| 8 | `color.cpp` | 颜色过滤（HSV 灯带拒绝调试） |
| 9 | `latency.cpp` | 延迟测量（端到端） |

> 同轴校准工具已移至 `src/boresight/`，随主工程构建。
