# 配置目录说明

本项目运行时主要读取 `config/` 下四个文件：

## 文件与职责

| 文件 | 作用 | 何时改 |
|---|---|---|
| `camera.yaml` | 相机参数、ROI、畸变校正、GPU 管线、构建时 `compute_mode` | 换相机、换镜头、换赛场光照时 |
| `cascade.yaml` | 两层检测、ROI 裁剪、灯带拒绝、跟踪器、比赛分析器参数 | 调检测 / 跟踪时 |
| `control.yaml` | 控制器、搜索策略、延迟补偿、`u_L/v_L`、`laser_offset_x/y` | 调控制或标定后 |
| `enemy.yaml` | 赛前覆盖项：敌方颜色、敌方所在画面侧、稳定性预设 | 每场比赛前 |

## 当前加载关系

- `tests/build/detect_demo`
  - `--config` 读取 `cascade.yaml`
  - `--enemy` 再覆盖 `enemy.yaml`
  - `--control` 读取 `u_L/v_L`

- `src/build/control/tracking_system`
  - `--camera-config` 读取 `camera.yaml`
  - `--cascade-config` 读取 `cascade.yaml`
  - `--enemy` 读取 `enemy.yaml`
  - `--control-config` 读取 `control.yaml`

- `tests/build/latency`
  - 同时读 `cascade.yaml`、`enemy.yaml`、`camera.yaml`、`control.yaml`

- `src/build/calibrate/laser_boresight`
  - `--config` 读取 `camera.yaml`
  - `--control` 读取并回写 `control.yaml`

## 现在最需要知道的几点

### `camera.yaml`

- `compute_mode` 会在配置 `src/camera` 时决定是否默认开启 CUDA / NPP
- `camera_matrix.data` 是当前控制链路读取内参的真实来源
- `calib_path` 支持相对路径，加载时会按配置文件目录解析

### `enemy.yaml`

- `enemy_color`：`red` 或 `blue`
- `enemy_side`：`left` 或 `right`
- `preset_stability_tier`：1 到 4

传入 `--enemy config/enemy.yaml` 时，检测器和比赛分析器都会被覆盖到这组赛前参数。

### `control.yaml`

当前同时包含两类“对准”信息：

- `u_L` / `v_L`
  - 仍被 `Controller` 和多处可视化链路直接使用
  - 本质上是像素坐标系里的参考落点

- `laser_offset_x` / `laser_offset_y`
  - 由 `laser_boresight` 当前版本写回
  - 代表激光与相机的物理偏移量，单位米
  - 当前 `tracking` 会直接用到这组量做角度域补偿

### `cascade.yaml`

- 当前默认模型路径指向 `src/detect/model/export/*.engine`
- 当前 layer2 配置是三类模型：`blue` / `purple` / `red`
- 同时保留了经典 CV 灯带拒绝与偏移预测逻辑

## 自动生成文件

- `config/.boresight_exposure`
  - `laser_boresight` 保存的上次曝光值缓存
  - 不需要手动维护

## 推荐运行方式

从仓库根目录运行程序时，始终显式传配置路径。完整命令范例请见顶层 [README.md](../README.md) 和 [tests/README.md](../tests/README.md)。这样可以避开各个程序源码里按“构建目录内启动”写的相对路径默认值。
