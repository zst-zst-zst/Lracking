# 检测模块

> 合并自: detect/README、DATA_COLLECTION_GUIDE

---

## 流水线

1. Layer 1: 全图检测无人机
2. 根据无人机框裁剪上方 ROI
3. Layer 2: ROI 内检测激光接收装置 (blue / purple / red)
4. 经典 CV 灯带拒绝
5. 输出给跟踪器与比赛分析器

## 构建

`detect` 编译为静态库, 默认依赖 `TRTInferX`。`DETECT_WITH_ONNX` 默认 OFF。

## 配置

主配置: `config/cascade.yaml`

- layer1 模型: `src/detect/model/export/layer1_plane_fp16.engine`
- layer2 模型: `src/detect/model/export/layer2_laser_rx_fp16.engine`
- layer2 三类: blue / purple / red
- `enemy.yaml` 运行时覆盖 `enemy_color`, `enemy_side`

## 训练脚本

```
src/detect/scripts/
├── train_layer1.py
├── train_layer2.py
├── train_layer2_improved.py
├── train_layer2_ensemble.py
├── export_onnx.py
├── validate_cascade.py
├── generate_layer2_crops.py
├── annotate_laser_rx.py
└── balance_purple_samples.py
```

---

## 数据采集指南

### 当前数据集

- train: 579, valid: 131, test: 67 (共 777 张)
- 目标: **2000+ 张**训练图片

### 困难样本重点

- **光照**: 强光/弱光/逆光/快速变化
- **角度**: 俯视/侧视/仰视/倾斜
- **距离**: 近(<5m) / 中(10-20m) / 远(20-30m+)
- **运动模糊**: 快速移动/相机抖动
- **遮挡**: 部分遮挡/画面边缘/多目标重叠
- **颜色混淆**: 紫白色/红紫/蓝紫过渡
- **背景干扰**: 复杂背景/相似颜色/灯光

### 采集方法

1. 比赛录制: 提取关键帧, 重点漏检和误检
2. 模拟环境: 控制光照/角度/距离, 系统性收集
3. 数据增强: 噪声/变形/合成困难样本
4. Roboflow 上传标注

### 标注

- 类别: `blue`, `purple`, `red`
- bbox 紧贴目标, 不遗漏小目标
- 推荐划分: train 80% / valid 10% / test 10%

### 质量检查

```bash
python3 src/detect/scripts/verify_dataset.py --data laser_rx/dataset.yaml
```

### 迭代

收集 → 标注 → 训练 → 测试 → 分析错误 → 针对性采集 → 重训
