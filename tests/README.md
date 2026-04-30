# 测试程序说明

本目录包含 9 个独立测试程序，用于逐层验证系统各模块功能。
测试按编号从底层到全流程递进，建议按顺序执行。

## 编译

```bash
# 从 Tracking/ 根目录，一键编译整个项目（含所有测试）
cmake -S tests -B build
cmake --build build -j$(nproc)
```

也可以只编译主工程（库 + demo，不含测试）：

```bash
cmake -S src -B build_src
cmake --build build_src -j$(nproc)
```

## 前置条件

| 需求 | 说明 |
|------|------|
| 相机 | 大恒 Galaxy 工业相机，通过 USB3 连接 |
| 云台串口 | `/dev/ttyUSB0`（默认），需要 `sudo chmod 666 /dev/ttyUSB0` |
| TRT 模型 | `src/detect/model/export/layer1_plane_fp16.engine` 和 `layer2_laser_rx_fp16.engine` |
| 配置文件 | `config/camera.yaml`、`src/detect/config/cascade.yaml`、`config/control.yaml` |
| 视频替代 | 无相机时可用 `--video test.mp4` 替代实时输入 |

## 测试列表

### 1. `serial` — 串口联通性测试

验证云台串口通信是否正常，收发帧并打印内容。

```bash
./build/serial [--port /dev/ttyUSB0] [--baud 115200]
# Ctrl+C 退出
```

- **无需相机/模型**，只需串口设备
- 验证：能看到云台回传帧即为正常

---

### 2. `laser_rx` — 激光接收装置识别（近距离台面测试）

直接用 Layer 2 YOLO 在全帧上检测激光接收装置，**不走级联流程**。
适合近距离桌面放一个激光模块来验证 Layer 2 模型效果。

```bash
./build/laser_rx [--config ../config/camera.yaml] \
                       [--model ../src/detect/model/export/layer2_laser_rx_fp16.engine] \
                       [--video test.mp4]
# q/ESC 退出
```

- **需要**：相机（或视频）+ Layer 2 TRT engine
- 验证：能正确框出激光接收装置

---

### 3. `plane` — 无人机识别（Layer 1 Only）

只用 Layer 1 YOLO 检测无人机，不做 Layer 2 级联。
验证无人机检测效果和置信度。

```bash
./build/plane [--config ../config/camera.yaml] \
                    [--model ../src/detect/model/export/layer1_plane_fp16.engine] \
                    [--video test.mp4]
# q/ESC 退出
```

- **需要**：相机（或视频）+ Layer 1 TRT engine
- 验证：能正确框出无人机

---

### 4. `cascade` — 级联检测（Layer 1 → ROI → Layer 2）

运行完整级联流程：Layer 1 检测无人机 → 裁剪 ROI → Layer 2 检测激光模块 → 后处理过滤。
显示无人机 bbox、ROI 区域、激光模块检测结果。

```bash
./build/cascade [--config ../src/detect/config/cascade.yaml] \
                      [--camera ../config/camera.yaml] \
                      [--video test.mp4]
# q/ESC 退出
```

- **需要**：相机（或视频）+ 两层 TRT engine + cascade.yaml
- 验证：级联流程端到端工作，激光模块在无人机上方被正确检出

---

### 5. `track` — 识别 + 跟踪照射 + 颜色检测

在级联检测基础上加入：Kalman+IoU 跟踪器、HSV 颜色检测（红/蓝灯带拒绝）、
云台闭环控制（向云台发送跟踪坐标）。近距离桌面测试。

```bash
./build/track [--config ../config/camera.yaml] \
                    [--model ../src/detect/model/export/layer2_laser_rx_fp16.engine] \
                    [--port /dev/ttyUSB0] [--color blue] \
                    [--video test.mp4]
# q/ESC 退出, c 切换颜色显示
```

- **需要**：相机 + Layer 2 engine + 云台串口
- 验证：目标被稳定跟踪，云台能跟随目标运动

---

### 6. `offset` — 不发光时偏移推断测试

模拟第 3 次锁定后激光模块停止发光的场景：
先正常检测+跟踪积累偏移量，按 `b` 模拟模块不发光（屏蔽 Layer 2），
系统用历史偏移推断预测模块位置。

```bash
./build/offset [--config ../src/detect/config/cascade.yaml] \
                     [--camera ../config/camera.yaml] \
                     [--video test.mp4]
# b=切换模拟不发光, q/ESC 退出
```

- **需要**：相机 + 两层 engine + cascade.yaml
- 验证：绿色=实际位置，黄色=预测位置，偏差应较小

---

### 7. `hit` — 追踪打中测试（全流程）

完整流程：级联检测 → 跟踪 → 偏移推断 → 云台控制 → 评估命中。
记录命中率统计，用于验证整体系统性能。

```bash
./build/hit [--config ../src/detect/config/cascade.yaml] \
                  [--camera ../config/camera.yaml] \
                  [--control ../src/control/config/control.yaml] \
                  [--port /dev/ttyUSB0] [--video test.mp4]
# b=模拟不发光, r=重置统计, q/ESC 退出
```

- **需要**：相机 + 两层 engine + 云台串口 + control.yaml
- 验证：命中率统计，评估激光能否持续命中目标

---

### 8a. `color` — 颜色过滤算法独立测试

不需要 YOLO 模型，纯 HSV 颜色分割可视化。
实时显示颜色掩码、轮廓形状分析（紧凑=模块 vs 细长=灯带），
带 Trackbar 实时调参。

```bash
./build/color [--config ../config/camera.yaml] \
                    [--color blue] [--video test.mp4]
# q/ESC 退出, r/b 切换红蓝
```

- **需要**：相机（或视频），**无需模型**
- 验证：调试灯带拒绝的 HSV 阈值和长宽比参数

---

### 8b. `latency` — 端到端延迟测量 & system_delay_ms 标定

**测量从相机取帧到串口发送的完整延迟链，给出 `system_delay_ms` 建议值。**

延迟链示意：
```
  相机取帧 → YOLO推理 → 跟踪器 → 控制器 → 串口发送 → 电控接收
  [grab]     [detect]   [track]  [ctrl]   [tx]        [serial_rtt/2]
  ├────────── PC 软件延迟 (sw_total) ──────┤ ├── 串口延迟 ──┤
  ├────────────── 建议 system_delay_ms ────────────────────┤
```

```bash
# 完整测量（有相机 + 有串口）
./build/latency

# 用视频替代相机
./build/latency --video test.mp4

# 没有串口也能测 PC 软件延迟
./build/latency --no-serial

# 指定串口设备
./build/latency --port /dev/ttyUSB0

# 无显示窗口（纯终端输出）
./build/latency --no-show

# Ctrl+C 或 q 退出，退出后打印最终报告
```

退出后输出最终报告，例如：
```
  ╔══════════════════════════════════════════════════════════════╗
  ║                    延迟测试最终报告                          ║
  ╠══════════════════════════════════════════════════════════════╣
  ║  环节          平均(ms)   P95(ms)   最大(ms)                ║
  ╟──────────────────────────────────────────────────────────────╢
  ║  取帧           0.5       0.8       1.2                     ║
  ║  推理           3.2       4.1       5.8                     ║
  ║  跟踪           0.1       0.2       0.3                     ║
  ║  控制           0.1       0.1       0.2                     ║
  ║  串口发送       0.3       0.5       0.8                     ║
  ╟──────────────────────────────────────────────────────────────╢
  ║  PC总计         4.2       5.5       7.1                     ║
  ║  串口RTT        3.0   (电控时间戳→PC接收)                    ║
  ╠══════════════════════════════════════════════════════════════╣
  ║  ★ 建议 system_delay_ms = 5.7                              ║
  ║    = PC软件 4.2 + 串口单程 1.5                              ║
  ║                                                              ║
  ║  用法: 把上面的值填到 config/control.yaml:                   ║
  ║        system_delay_ms: 5.7                                  ║
  ╚══════════════════════════════════════════════════════════════╝
```

- **需要**：相机 + 两层 engine + cascade.yaml + control.yaml；串口可选
- **用法**：把输出的 `★ 建议 system_delay_ms` 填到 `config/control.yaml` 的 `system_delay_ms`

---

## 推荐测试顺序

```
硬件验证          模型验证            集成测试            性能测试
─────────       ──────────        ──────────         ──────────
1. serial  →    2. laser_rx  →    4. cascade    →    8b. latency
                3. plane     →    5. track      →    7.  hit
                8a. color         6. offset
```

1. **先验证硬件**：`serial`（串口）、`color`（相机+颜色）
2. **再验证模型**：`plane`（Layer 1）、`laser_rx`（Layer 2）
3. **逐步集成**：`cascade` → `track` → `offset` → `hit`
4. **最后测性能**：`latency`

## 无相机时的离线测试

所有视觉测试都支持 `--video` 参数，可以用录制的视频替代实时相机：

```bash
./build/cascade --video /path/to/recorded.mp4 \
                      --config ../src/detect/config/cascade.yaml
```

`detect_demo` 运行时会自动录制视频到 `records/` 目录，可以用这些视频回放测试。
