# galaxy_camera

当前仓库里的相机模块是一个静态库，不再单独在 `src/camera` 下生成 demo；常用验证入口已经移到 [tests/README.md](../../tests/README.md)。

## 当前职责

- 调用仓库内置 `Galaxy SDK`
- 加载 `config/camera.yaml`
- 完成采集、ROI、可选畸变校正
- 决定出图是走 CPU 还是 GPU 去马赛克路径

`src/camera/CMakeLists.txt` 会优先使用 `src/camera/galaxy_sdk` 作为 `GALAXY_SDK_ROOT`。

## 你最常改的几个开关

### `output_bgr`

这不是“相机会不会拍出彩色图”，而是“上层拿到的是不是已经转好的 BGR 图”。

- 设为 1：上层更容易直接用 OpenCV 显示彩色画面
- 设为 0：上层可能拿到原始 Bayer 或灰度数据，很多 demo 就不会自动显示彩色图

如果你在某台机器上只想先把链路跑通，通常保留 `output_bgr: 1` 更省事。

### `undistort_enable` / `calib_path`

这组参数只管一件事：出图前要不要做去畸变。

- `undistort_enable`：是否开启去畸变
- `calib_path`：畸变参数文件路径
- `camera_matrix` / `distortion_coefficients`：相机内参和畸变参数本身

`loadCameraConfig()` 会按配置文件所在目录解析相对路径，所以你在 `config/camera.yaml` 里写相对路径时，不用手工改成绝对路径。

### GPU 管线

这几项只在你想走 GPU 去马赛克时才有意义：

- `gpu_pipeline_enable`
- `gpu_demosaic_enable`
- `gpu_demosaic_backend`
- `gpu_device_id`
- `force_swap_rb`

如果机器上没有独立 NVIDIA GPU，或者你只是想保证比赛现场能稳定跑，直接把 `gpu_pipeline_enable` 和 `gpu_demosaic_enable` 关掉就行，走 CPU 路径最稳。

`gpu_device_id` 只在多 GPU 机器上需要调，`force_swap_rb` 只有颜色通道顺序不对时才改。

## 现状说明

- 当前默认配置文件是 `config/camera.yaml`
- 不存在单独的 `config/camera_build.yaml`
- 当前源码里不再维护 `detector_galaxy_demo`
- 相机相关验证入口请看 [tests/README.md](../../tests/README.md)
