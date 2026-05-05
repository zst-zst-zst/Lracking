# SR (超分) 模块使用说明

## 现状

- **模型**: ESPCN ×2，64K 参数，PSNR 36.13 dB（合成下采样基准）
- **推理延迟**: TRT FP16 约 0.34 ms（128×128 输入）
- **真实效果**: 在现有合成训练数据上视觉提升微小，需 35mm 真实远距素材重训才能显著
- **默认状态**: **关闭**。整套基础设施保留待重训

## 文件清单

### 训练管线（Python, tools/）
| 文件 | 作用 |
|------|------|
| `sr_extract_hr.py` | 从 layer2 数据集抽 HR patch（`merged/` → `sr_dataset/hr/<class>/`） |
| `sr_dataset.py` | 数据加载: HR 在线下采样生成 LR/HR 对 |
| `sr_model.py` | ESPCN ×2 网络定义 |
| `sr_train.py` | 训练脚本，最佳权重存 `src/detect/model/sr/espcn_x2_best.pt` |
| `sr_export.py` | 导出 ONNX + 用 trtexec 构建 FP16 engine |
| `sr_visualize.py` | 真实/合成对比图，输出到 `src/detect/model/sr/visualize/` |
| `sr_extract_video.py` | 从视频自动抽 HR patch（待重录后用） |

### 运行时（C++, src/detect/）
| 文件 | 作用 |
|------|------|
| `sr_inferer.h/.cpp` | ONNX Runtime + CUDA 包装，动态分辨率，简单 BGR→BGR ×2 接口 |
| `cascade_detector.{h,cpp}` | 在 layer2 之前可选地对小 ROI 上采样 |
| `CMakeLists.txt` | `DETECT_WITH_ONNX=ON` 时自动编译 SR |

### 模型文件
- `src/detect/model/sr/espcn_x2_best.pt` — PyTorch 权重
- `src/detect/model/export/espcn_x2.onnx` — ONNX 导出
- `src/detect/model/export/espcn_x2_fp16.engine` — TRT FP16 (留作未来直接 TRT 集成用)

## 启用 SR (A/B 测试)

1. **编译时启用 ONNX 后端**:
   ```bash
   cmake -S /home/zst/Tracking/src -B /home/zst/Tracking/src/build -DDETECT_WITH_ONNX=ON
   cmake --build /home/zst/Tracking/src/build -j
   ```

2. **运行时打开 flag**: 编辑 `config/cascade.yaml`:
   ```yaml
   sr_enable: 1                # 0=关 (默认), 1=开
   sr_max_roi_size: 320        # ROI 短边 < 此值才上 SR
   ```

3. **A/B 对比**: 跑两次 `tests/hit` 或 `tests/detect_demo`，分别 `sr_enable=0` 和 `=1`，对比远距小目标 recall。

## 何时重训

拿到 35mm 真实比赛场地远距素材（≥10 分钟视频或 ≥500 张帧）后：

```bash
# 1. 用 sr_extract_video.py 自动从视频抽 HR (需调相机区域 crop 参数)
python tools/sr_extract_video.py --videos <video.mp4> ...

# 2. 重训 (复用现有合成 HR + 新真实 HR)
python tools/sr_train.py

# 3. 重新导出
python tools/sr_export.py
```

## 已知局限

- SR 在合成下采样数据上 work，但真实远距小目标的 LR/HR 对差异未训练
- 6mm 镜头的 `records/tracking/` mp4 全是空文件（258 字节失败录制），不可用
- `~/Videos/录屏/` 的 24 分钟录屏是 T 项目的，与 laser_rx 无关
- 截图 `Tracking/图片/` 中只有 2 张真实 35mm 场地帧，不够重训

下次有真实素材时，遵循本 README 即可重训 + 集成。
