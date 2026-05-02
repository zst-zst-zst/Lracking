# 标定与配置

> 合并自: calibrate/README、config/README

---

## 配置文件概览

| 文件 | 作用 | 何时改 |
|------|------|--------|
| `config/camera.yaml` | 相机内参、曝光、ROI、去畸变、GPU 管线 | 换相机/镜头/光照 |
| `config/cascade.yaml` | 两层检测、跟踪器、比赛分析 | 调检测/跟踪 |
| `config/control.yaml` | 控制器、激光补偿、搜索策略 | 调控制或标定后 |
| `config/enemy.yaml` | 敌方颜色、侧面、稳定性预设 | 每场比赛前 |

### camera.yaml 要点

- `camera_matrix.data`: 内参来源 (fx, fy, cx, cy)
- `output_bgr`: 1=上层拿 BGR 彩色图, 0=原始 Bayer/灰度
- `undistort_enable` / `calib_path`: 去畸变开关与参数文件 (支持相对路径)
- `gpu_pipeline_enable` / `gpu_demosaic_enable`: GPU 去马赛克 (没 GPU 就关掉)

### cascade.yaml 要点

- layer1 模型: `src/detect/model/export/layer1_plane_fp16.engine`
- layer2 模型: `src/detect/model/export/layer2_laser_rx_fp16.engine`
- layer2 三类: blue / purple / red
- `enemy.yaml` 运行时覆盖 `enemy_color`, `enemy_side`, `preset_stability_tier`

### control.yaml 要点

```yaml
# 激光同轴补偿 (laser_boresight 标定输出)
laser_offset_x: 0.000318       # 物理水平偏移 (m)
laser_offset_y: 0.029600       # 物理垂直偏移 (m)
boresight_yaw_deg: 0.4176      # 角度偏差 yaw (deg, 常数)
boresight_pitch_deg: 1.3838    # 角度偏差 pitch (deg, 常数)

# 目标物理尺寸 (估距用)
target_height_m: 0.050         # S190 模块高度 50mm
```

### enemy.yaml 要点

- `enemy_color`: `red` 或 `blue`
- `enemy_side`: `left` 或 `right`
- `preset_stability_tier`: 1~4

---

## 激光-相机标定 (laser_boresight)

### 运行

```bash
# CLI
./src/build/calibrate/laser_boresight \
  --config config/camera.yaml \
  --control config/control.yaml \
  --port /dev/ttyUSB0

# GUI
python3 tools/laser_calib_gui.py
```

可选参数: `--baud 115200`, `--threshold 200`, `--exposure 200`, `--session logs/laser_calib_session.yaml`, `--resume-session`, `--fresh-session`

### 手动模式

1. `W/A/S/D` 或 `0` 微调云台
2. 数字 `1-6` 选距离 (0.5/1.0/1.5/2.0/3.0/5.0m)
3. 稳定后按 `c` 采样
4. 按 `f` 拟合
5. 按 `r` 写回 `control.yaml`

### 引导式多距离模式

按 `g` 启动 → 按空格推进 → 自动多距离多轮扫描

### 快捷键

| 键 | 功能 |
|----|------|
| W/A/S/D | 云台微调 |
| 0 | 回正 |
| 1-6 | 设置距离 |
| Space | 确认引导步骤 |
| g | 开始多距离引导 |
| c | 采样当前距离 |
| x / X | 删除最后一个 / 清空全部样本 |
| f | 拟合 |
| r | 写回 control.yaml |
| e / t | 提高 / 降低曝光 |
| +/- | 调激光点阈值 |
| q / ESC | 退出 |

### 标定输出

写回 `config/control.yaml`:
- `laser_offset_x`, `laser_offset_y` — 物理偏移 (m)
- `boresight_yaw_deg`, `boresight_pitch_deg` — 角度偏差 (deg)

### 运行时文件

| 文件 | 用途 |
|------|------|
| `logs/laser_calib_session.yaml` | 会话进度, 支持断点续标 |
| `config/.boresight_exposure` | 曝光值缓存, 下次自动加载 |

---

## 配置加载关系

| 程序 | 读取 |
|------|------|
| `tracking` | `camera.yaml` + `control.yaml` |
| `tracking_system` | `camera.yaml` + `cascade.yaml` + `enemy.yaml` + `control.yaml` |
| `detect_demo` | `cascade.yaml` + `enemy.yaml` + `control.yaml` |
| `latency` | `cascade.yaml` + `enemy.yaml` + `camera.yaml` + `control.yaml` |
| `laser_boresight` | `camera.yaml` + 读写 `control.yaml` |
