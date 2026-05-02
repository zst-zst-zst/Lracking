# TRTInferX

这是当前仓库 vendored 进来的 TensorRT 推理层，`src/detect` 默认直接依赖它。

## 当前接入方式

- `src/detect/CMakeLists.txt` 会把 `TRTInferX` 作为子目录加入
- 主库目标名：`trt_yolo`
- 示例程序目标名：`trt_yolo_example`

## 当前仓库里的用途

主要服务于两套 engine：

- `src/detect/model/export/layer1_plane_fp16.engine`
- `src/detect/model/export/layer2_laser_rx_fp16.engine`

上层程序包括：

- `tests/build/plane`
- `tests/build/laser_rx`
- `tests/build/detect_demo`
- `src/build/control/tracking_system`

## 相关脚本

目录下保留了导出与校准脚本：

- `scripts/export_onnx.py`
- `scripts/export_fp16_engine.sh`
- `scripts/export_int8_engine.sh`
- `scripts/export_all.sh`
- `scripts/gen_calib_bin.py`

这些脚本适合单独维护模型导出流程，但不属于当前仓库“开箱即跑”的主链路。

## 说明

这里的 README 只描述当前项目内的集成状态；上游性能宣传、外部样例和历史工程结构不再作为本仓库的主文档。
