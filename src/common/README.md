# common

`common` 提供被多个模块共享的基础类型和小工具。

## 当前内容

- `include/common/types.h`
  - `TargetMeasurement`
  - `GimbalState`
  - `GimbalCommand`
  - `CameraModel`
  - `Boresight`

- `include/common/time_utils.h`
  - 单调时钟与系统时钟的毫秒时间工具

- `include/common/tee_buf.h`
  - 把 `stdout` / `stderr` 同时写到终端和日志文件

- `include/common/config_io.h`
  - `loadCameraModel`
  - `loadBoresight`

## 现在的实际使用方式

- 主控制链路通常通过 `control::loadControlConfig()` 从 `config/camera.yaml` 的 `camera_matrix.data` 里读取内参
- `common::loadBoresight()` 读取的是 `control.yaml` 里的 `u_L` / `v_L`

注意：

- `common::loadCameraModel()` 读取的是扁平键 `fx` / `fy` / `cx` / `cy`
- 当前仓库主用的 `config/camera.yaml` 不是这种扁平格式，因此这个接口更适合旧配置或额外小工具，不是主链路首选

## 单位约定

- 图像坐标：`u` 向右为正，`v` 向下为正
- 云台角度：内部使用 `deg`
- 云台角速度：内部使用 `deg/s`
- 串口线缆侧现在也直接使用 `deg` / `deg/s` / `deg/s^2`
