# detect

`src/detect` 是当前仓库的检测层实现。主流程是：

1. Layer 1 在全图检测无人机
2. 根据无人机框裁剪上方 ROI
3. Layer 2 在 ROI 内检测激光接收装置
4. 经典 CV 灯带拒绝
5. 输出给跟踪器与比赛分析器

## 当前源码结构

```text
src/detect/
├── CMakeLists.txt
├── cascade_detector.cpp/h
├── detector.h
├── measurement.cpp/h
├── onnx_detector.cpp/h
├── TRTInferX/
├── scripts/
├── model/
└── DATA_COLLECTION_GUIDE.md
```

## 当前构建关系

- `detect` 是静态库
- 默认依赖 `TRTInferX`
- `DETECT_WITH_ONNX` 默认是 `OFF`
- 当前源码不再在本目录单独生成 `detect_demo`

可视化入口与运行命令请见顶层 [README.md](../../README.md) 和 [tests/README.md](../../tests/README.md)。

## 当前运行配置

主配置文件是仓库根目录下的：

- `config/cascade.yaml`

不是早期目录里的 `src/detect/config/...`。

当前 `config/cascade.yaml` 的几个要点：

- layer1 模型：`src/detect/model/export/layer1_plane_fp16.engine`
- layer2 模型：`src/detect/model/export/layer2_laser_rx_fp16.engine`
- layer2 当前按三类模型配置：`blue` / `purple` / `red`
- `enemy.yaml` 会在运行时覆盖 `enemy_color`、`enemy_side`、`preset_stability_tier`
- 启用了灯带拒绝、跟踪器配置、偏移预测和比赛分析参数

## 运行入口

- 级联检测可视化、Layer 1 / Layer 2 单测入口都已收敛到顶层 README 和 [tests/README.md](../../tests/README.md)
- 本页只保留检测配置和模型组织约定

## 训练与脚本

当前保留的脚本包括：

- `scripts/train_layer1.py`
- `scripts/train_layer2.py`
- `scripts/train_layer2_improved.py`
- `scripts/train_layer2_ensemble.py`
- `scripts/export_onnx.py`
- `scripts/validate_cascade.py`
- `scripts/generate_layer2_crops.py`
- `scripts/annotate_laser_rx.py`
- `scripts/balance_purple_samples.py`

数据采集说明见：

- [DATA_COLLECTION_GUIDE.md](DATA_COLLECTION_GUIDE.md)

## 现在需要注意的差异

- 本目录 README 以“当前工程接入状态”为准，不再使用早期 `src/detect/config`、`detect_demo` 本地 target、旧数据目录描述
- 代码里仍能看到一些早期“两类 layer2”注释，但当前实际运行配置已经是三类颜色模型
- 从仓库根目录运行 `detect_demo` 时，建议显式传 `--record-dir`，否则默认录制目录会按旧相对路径解析
