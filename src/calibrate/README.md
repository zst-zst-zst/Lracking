# laser_boresight

`src/calibrate` 当前提供的标定工具是：

- `laser_boresight`

它已经不是旧版“只写 `u_L/v_L`”的校准器；当前版本的主输出是 `config/control.yaml` 里的：

- `laser_offset_x`
- `laser_offset_y`

构建、可执行文件路径和运行命令请见顶层 [README.md](../../README.md)。本页只保留标定流程本身的说明。

常用可选参数：

- `--baud 115200`
- `--threshold 200`
- `--exposure 200`
- `--session logs/laser_calib_session.yaml`
- `--resume-session`
- `--fresh-session`

## 当前工作流

### 手动模式

1. 启动程序
2. 用 `W/A/S/D` 或 `0` 微调云台
3. 用数字键 `1-6` 选择距离
4. 目标稳定后按 `c` 采样
5. 按 `f` 拟合
6. 按 `r` 写回 `config/control.yaml`

距离预设：

- `1 = 0.5m`
- `2 = 1.0m`
- `3 = 1.5m`
- `4 = 2.0m`
- `5 = 3.0m`
- `6 = 5.0m`

### 引导式多距离模式

- 按 `g` 启动多距离引导流程
- 按空格进入下一步
- 程序会在多个距离和多轮扫描间自动记录进度

## 当前快捷键

- `W/A/S/D`：云台微调
- `0`：回正前方
- `1-6`：设置距离
- `SPACE`：确认当前引导步骤
- `g`：开始多距离引导标定
- `c`：保存当前距离样本
- `x`：删除最后一个样本
- `X`：清空全部样本
- `f`：拟合
- `r`：写回 `control.yaml`
- `e`：提高曝光
- `t`：降低曝光
- `+/-`：调激光点阈值
- `q` / `ESC`：退出

## 输出文件

### 回写配置

写回：

- `config/control.yaml`

当前保存的是：

- `laser_offset_x`
- `laser_offset_y`

### 运行时文件

- 进度文件：`logs/laser_calib_session.yaml`
- 快照目录：`logs/laser_calib_snapshots/`
- 曝光缓存：`config/.boresight_exposure`

## GUI 前端

仓库还提供了 PyQt5 前端，入口见顶层 README。它默认调用 `laser_boresight`，因此使用 GUI 时请先按顶层文档把 `src/build` 构建出来。

## 现状提醒

- 当前 `laser_boresight` 保存的是物理偏移量，不是旧版 README 里写的 `u_L/v_L`
- `control.yaml` 中的 `u_L/v_L` 仍然被主控制链路使用，需要和当前控制逻辑分开理解
