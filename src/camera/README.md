# galaxy_camera

- 当前现场型号：`MER-139-210U3C`。
- 原生分辨率：`1280x1024 @ 210fps`。
- 当前实现按 USB 相机链路运行（仅枚举 U3V/USB2 设备）。
- 默认配置从 `1280x1024` 中心裁出 `640x480` ROI，便于直接联调 detector。

## 构建

默认使用仓库内置 SDK：`src/camera/galaxy_sdk`。

```bash
cmake -S src -B src/build
cmake --build src/build --target galaxy_camera_demo detector_galaxy_demo -j$(nproc)
```

GPU/CPU 切换改成配置文件控制（不需要两条 CMake 命令）：

```bash
vim config/camera_build.yaml   # compute_mode: "gpu" 或 "cpu"
cmake -S src -B src/build
```

## 运行

相机 demo：

```bash
./src/build/camera/galaxy_camera_demo --config config/camera.yaml --show
```

检测 demo：

```bash
./src/build/detector/detector_galaxy_demo --camera-config config/camera.yaml
```

## 配置说明

- `serial_number` / `use_first_device` / `device_index`：设备选择。
- `trigger_mode` / `trigger_source`：连续采集或外部触发。
- `frame_rate_enable` / `frame_rate`：默认 `120fps`，比旧配置里的 `19.2fps` 更适合跟踪联调。
- `roi_enable` / `roi_width` / `roi_height`：默认启用从 `1280x1024` 中心裁出的 `640x480` ROI。
- `undistort_enable` + `calib_path`：当前默认开启并使用 `config/camera_ost.yaml`。
- `feature_load_enable`：默认关闭；启用前请换成当前相机重新导出的 `.mfs`，否则可能覆盖 ROI / Width / Height。
- `gpu_pipeline_enable` / `gpu_demosaic_enable`：GPU 去马赛克开关（NPP 路径）。
- 若希望尽量走纯 GPU 链路，建议设置 `output_bgr=0`（避免每帧 D2H 拷回 CPU）。
- 红蓝通道相关 Bayer 处理按相机每帧上报的 `pixel_type` 自动判断，不从配置读取 `bayer_pattern`。

如需回到全分辨率，直接把 `roi_enable` 改成 `0`，或按相机支持范围改 `roi_width` / `roi_height`。

## 说明

- 相对路径默认以配置文件所在目录解析。
- `config/` 开头的相对路径会自动映射到项目根目录 `config/`。
- 若运行时报找不到 `libgxiapi.so`，检查 `GALAXY_SDK_ROOT` 和运行时库路径是否一致。
- 如需回到相机全分辨率，直接把 `roi_enable` 设为 `0`。
