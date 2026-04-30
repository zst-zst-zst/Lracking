# Plane Laser RX 两层级联检测系统

## 概述

本项目实现针对 RoboMaster 空中机器人（无人机）上**激光检测模块（Laser RX）**的两层级联神经网络检测系统，
用于雷达系统的激光发射装置自动瞄准。

设计思路参考 T 项目的级联检测架构：
- **T 项目**：YOLO 检测车辆 → YOLO 检测装甲板（在车辆区域内） → 分类器识别编号
- **本项目**：YOLO 检测无人机（plane） → YOLO 检测激光接收模块 + 灯带（在无人机上方区域）→ 后处理过滤

## 架构

```
全帧图像
   │
   ▼
┌──────────────────────────────┐
│  Layer 1: YOLO               │  检测无人机（plane）
│  输入: 全帧                   │  输出: plane bbox list
│  过滤: 只保留敌方（右侧）无人机 │
└────────────┬─────────────────┘
             │
             ▼
┌──────────────────────────────┐
│  ROI 裁剪                     │  从 plane bbox 上方扩展约 1.0 倍 bbox 高度
│  水平居中于无人机              │  （无人机可见机体 ~20-30cm，激光模块在
│                               │   ~20-30cm 上方）
└────────────┬─────────────────┘
             │
             ▼
┌──────────────────────────────┐
│  Layer 2: YOLO (2 classes)   │  检测:
│  输入: ROI 裁剪               │    class 0: laser_rx（目标）
│                               │    class 1: light_strip（干扰，丢弃）
└────────────┬─────────────────┘
             │
             ▼
┌──────────────────────────────┐
│  后处理过滤                    │  - 丢弃 class 1 (light_strip)
│                               │  - 宽高比过滤（laser_rx 近正方形，灯带细长）
│                               │  - 面积比过滤
│                               │  - 映射回全帧坐标
└──────────────────────────────┘
```

## 关键设计约束

### 1. 敌友区分
- 画面**右侧** = 敌方无人机（目标），**左侧** = 己方无人机（忽略）
- 通过 `enemy_x_threshold`（归一化 x 坐标阈值，默认 0.5）判定
- 如果己方无人机突然出现在画面中，不会误识别

### 2. 激光检测模块（来自制作规范手册）
- 整体尺寸: **72mm × 50mm × 50mm**
- 检测区域: **φ42mm**（圆形）
- 中空区域: **φ20mm**（套在刚性保护杆上）
- 安装位置: 刚性保护杆顶部，距螺旋桨中心面 **350±50mm**
- 形状: **近似正方形**，宽高比 ~1.0-1.5

### 3. 无人机可见尺寸
- 规范中的 1.1m/0.8m 包含底部起落架，**标注时不会包含起落架**
- 实际可识别的无人机机体高度约 **20-30cm**
- 因此激光模块在 bbox 上方约 **1.2-1.75 倍 bbox 高度**处

### 4. 灯带干扰问题（关键难点）
- 无人机上有**航行外观灯带**，与激光模块**同色系**
- 灯带朝上安装，从雷达相机角度清晰可见
- 各队形状不同：折线/V形/平行四边形/交叉等，但共性是**细长线条、强饱和队伍色**
- 灯带总长度 > 1500mm，距激光模块最外边沿 > 250mm
- **极易与 laser_rx 混淆**

**解决方案（四层防御，不需要灯带标注数据）：**
1. **经典 CV 灯带拒绝（核心，无需标注）**: 对每个 Layer 2 检测结果的 bbox 区域做：
   - **HSV 颜色分析**: 统计敌方队伍色（蓝 H∈[90,130] 或红 H∈[0,15]∪[165,180]）像素占比
   - **轮廓形状分析**: 颜色区域的 `minAreaRect` 长宽比
   - **判定**: 队伍色占比 > 25% **且** 形状细长（elongation > 2.5）→ 拒绝为灯带
   - 极高颜色占比 > 60% 时即使形状紧凑也拒绝（交叉灯带可能填满区域）
2. **双类训练（可选）**: 如有标注数据，Layer 2 可训练 `light_strip`（class 1）辅助拒绝
3. **宽高比过滤**: laser_rx 近正方形（72×50mm, aspect ≤ 2.5），灯带细长（aspect >> 2.0）
4. **面积比过滤**: laser_rx 在 ROI 中的面积占比有合理范围（0.2%-50%）

### 5. ROI 裁剪策略
- ROI 水平: 以无人机 bbox 水平中心为准，宽度 = bbox 宽度 × 1.2
- ROI 垂直: 从 bbox 顶部**向上扩展 1.0 倍 bbox 高度**，向下包含 0.3 倍
- 激光模块在无人机上方约 20-30cm，与可见机体高度相当，所以 1.0x 即可覆盖
- 由于视角问题，目标可能在 bbox 内偏上，也可能在 bbox 上方

## 比赛场景与物理环境

### 场地
- 战场: **28m × 15m**，围挡高度 2.4m
- **雷达基座**: 平台高度 ~2.5m，面积 3.4m × 1.16m，围栏 1.1m
- **相机**: 安装在雷达基座传感器架上，离地约 **3.2-3.5m**，位于己方短边 5-10m 位置
- 对面 15m 处是敌方，无人机在右侧（敌方半场）

### 无人机飞行行为
- **停机坪**: 比赛前无人机停在停机坪上，停机坪高度低于飞行高度
- **飞行高度**: 2.0-2.4m（规则最低 1.5m, 最高不超围挡 2.4m）
- **安全绳**: 2.4m，钢丝绳上卡环距己方场边 ~14m
- **飞行路径**: 沿钢丝绳下方，在细长平行四边形区域内飞行

**飞行阶段:**
1. **IDLE** — 停机坪上等待，电控上电
2. **TAKEOFF** — 起飞上升，离开停机坪
3. **CRUISING** — 沿绳索路径飞向前哨站
4. **ATTACKING** — ★ 最佳照射时机 ★ 接近前哨站，悬停或缓慢移动击打
5. **RETURNING** — 返航回停机坪
6. **LANDING** — 降落到停机坪，断电

### 激光锁定机制 (P 值)
- 仅在敌方**空中支援**期间，己方激光装置上电
- P 值 0→100，达到 P0 时**锁定发射机构 45s**，P 归零
- 每局最多 **3 次**锁定
- **P 增长**: 连续照射每 0.1s，P += n（n 从 1 递增），即 P = n(n+1)/2
  - 第 1 次锁定: P0=50 → 需 **≥1.0s** 连续照射
  - 第 2 次锁定: P0=100, 检测区域→3/5 → 需 **≥1.4s**
  - 第 3 次锁定: P0=100, 检测区域→1/5 且**停止发光** → 需 **≥1.4s**
- **关键**: 中断 <0.1s 导致 n 归零！**连续性至关重要**

### 无人机稳定性分档
赛前预设，比赛中系统自动观察修正。基准线在 2-3 档之间。

| 档位 | 描述 | 策略 | 抖动阈值 | 平滑系数 |
|------|------|------|---------|---------|
| 1 | 极致丝滑，顶级学校 | `precise` 精确跟踪 | 3px | 0.7 |
| 2 | 正常水平，偶有小抖 | `track` 正常跟踪 | 6px | 0.5 |
| 3 | 经常抖动，不够稳定 | `predict` 预测跟踪 | 12px | 0.3 |
| 4 | 严重抖动，幅度很大 | `opportune` 瞄准质量评估 | 25px | 0.2 |

**自适应校准**: 开始跟踪后观察 3s 内抖动幅度，自动修正档位，之后每 10s 重评一次。
同一学校不同飞机、不同场次可能不同，以实际表现为准。

### 目标跟踪器 (Kalman + IoU)
类似 DeepSORT 的帧间关联跟踪器，维护激光模块目标轨迹。

**核心功能：**
- **Kalman 滤波**: 状态 [cx, cy, vx, vy, w, h]，匀速模型预测
- **IoU + 距离关联**: 贪心匹配（目标少，通常 1-3 个）
- **轨迹状态**: TENTATIVE(暂定) → CONFIRMED(确认) → LOST(丢失)
  - 连续 3 帧命中 → 确认
  - 确认后 15 帧未命中 → 丢失删除
- **检测丢失时预测**: 已确认轨迹在检测短暂丢失时继续预测位置
- **速度估计**: Kalman 滤波器输出速度，HUD 显示速度箭头
- **轨迹日志**: 程序退出时保存完整轨迹到 `trajectory_log.yaml`

**赛后分析**: 轨迹日志记录每帧的 (timestamp, x, y, vx, vy, w, h, confidence, detected/predicted)，
可以重建无人机激光模块的完整运动路径。



## 数据集

**数据收集指南：** 参见 [DATA_COLLECTION_GUIDE.md](DATA_COLLECTION_GUIDE.md)

两个**独立**数据集，分别训练：

| Layer | 数据集 | 图片数 | 类别 | 来源 |
|-------|--------|--------|------|------|
| Layer 1 (plane) | `detect/plane/` | 10478 train / 1309 val | 1 类: plane | 自采集 |
| Layer 2 (laser_rx) | `detect/laser_rx/` | 579 train / 131 valid / 67 test | 3 类: blue/purple/red | Roboflow + 自采集 |

> **注意**: plane 数据集里的无人机**不带**激光检测装置（只有红蓝灯带）。
> laser_rx 是独立数据集，包含带激光模块的全帧图像。

## 使用流程

所有命令在 `src/detect/` 目录下执行，需先激活虚拟环境：
```bash
source /home/zst/Tracking/.venv/bin/activate
cd src/detect
```

### 1. 训练 Layer 1（plane 检测）
```bash
python3 scripts/train_layer1.py
```
约 50 分钟 ~ 1.5 小时（RTX 4060 Laptop, 100 epochs, patience=20）。
输出: `model/layer1/train/weights/best.pt`

### 2. 训练 Layer 2（laser_rx 检测）

**基础训练（快速验证）：**
```bash
python3 scripts/train_layer2.py --data laser_rx/dataset.yaml
```

**最大精度训练（推荐）：**
```bash
python3 scripts/train_layer2_improved.py
```

**集成学习训练（最高精度）：**
```bash
python3 scripts/train_layer2_ensemble.py --n-models 3
```

**训练参数对比：**

| 参数 | 基础 | 最大精度 | 集成学习 |
|------|------|---------|---------|
| 模型 | yolov8n | yolov8s | yolov8s × 3 |
| 分辨率 | 192×192 | 256×256 | 256×256 |
| Epochs | 150 | 300 | 300 × 3 |
| 数据增强 | 中等 | 激进 | 激进 |
| TTA | 无 | 支持 | 支持 |
| 训练时间 | ~2h | ~6h | ~18h |
| 预期精度 | 基线 | +5-10% | +8-15% |
Layer 2 使用独立的 laser_rx 数据集，**与 Layer 1 互不依赖，可同时训练**。
输出: `model/layer2/train/weights/best.pt`

> 已有预训练权重: `laser_rx/runs/detect/train2/weights/best.pt`

### 3. 导出 ONNX / TRT Engine
```bash
python3 scripts/export_onnx.py \
    --layer1 model/layer1/train/weights/best.pt \
    --layer2 model/layer2/train/weights/best.pt \
    --trt
```

### 4. 验证级联管线
```bash
python3 scripts/validate_cascade.py \
    --layer1 model/layer1/train/weights/best.pt \
    --layer2 model/layer2/train/weights/best.pt \
    --video <video_path> \
    --enemy-side right
```

### 补充工具（可选）

如果将来有**带激光装置的无人机**视频，可以用以下工具从 Layer 1 检测结果裁剪 ROI，生成新的 Layer 2 训练数据：

```bash
# 从全帧图裁剪无人机上方 ROI
python3 scripts/generate_layer2_crops.py \
    --model model/layer1/train/weights/best.pt \
    --images <带激光装置的图片目录> \
    --output dataset/layer2/images/train \
    --labels-out dataset/layer2/labels/train

# 手动标注裁剪图中的 laser_rx
python3 scripts/annotate_laser_rx.py \
    --images dataset/layer2/images/train \
    --labels dataset/layer2/labels/train
```

## 与 Tracking 项目集成

`detect` 模块通过 `src/CMakeLists.txt` 的 `BUILD_DETECT` 选项自动编译：

```bash
cmake -S src -B src/build
cmake --build src/build -j$(nproc)
```

demo 用法：

```bash
# 平时测试 (自动录制半分辨率 → records/test/)
./detect_demo --video input.mp4

# ★ 比赛模式 ★ (自动录制全分辨率 → records/match/)
./detect_demo --match

# 全部选项
./detect_demo \
    --config ../config/cascade.yaml \
    --enemy ../../../config/enemy.yaml \
    --match \
    --control ../../control/config/control.yaml \
    --video input.mp4 \
    --record-dir /custom/path \
    --no-plot
```

### 自动录制

**默认自动开启**，文件名格式: `YYYY-MM-DD_HH-MM-SS.mp4`

| 模式 | 触发条件 | 分辨率 | 输出目录 |
|------|---------|--------|---------|
| **比赛** | 传了 `--match` | **全分辨率** | `records/match/` |
| **测试** | 未传 `--match` | 半分辨率 | `records/test/` |

用 `--no-record` 关闭录制，`--record-dir` 覆盖输出目录。

### 参数列表

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--config` | `../config/cascade.yaml` | 级联检测参数 |
| `--match` | 无 | 赛前参数覆盖 (同时激活比赛模式) |
| `--control` | `../../control/config/control.yaml` | 读取 boresight (u_L, v_L) |
| `--video` | 摄像头 | 视频文件输入 |
| `--cam` | `0` | 摄像头 ID |
| `--record-dir` | 自动 (match/test) | 自定义录制目录 |
| `--no-record` | — | 关闭自动录制 |
| `--no-show` | — | 不显示窗口 |
| `--no-plot` | — | 不发送 PlotJuggler |

### 可视化内容

- **绿色轨迹线**: 目标检测中心历史 (最近180帧)
- **红色轨迹线**: 激光预估落点 (基于 boresight 偏移)
- **黄色虚线**: 目标→激光落点偏移距离 (px)
- **红色十字**: boresight 固定位置 (u_L, v_L)
- PlotJuggler 额外输出: `laser_u`, `laser_v`, `laser_offset_px`

## 配置文件说明

| 文件 | 修改频率 | 内容 |
|------|---------|------|
| `config/enemy.yaml` | **每场比赛前** | 敌方颜色、位置、预设档位 |
| `cascade.yaml` | 平时调参 | 检测/跟踪/分析的详细参数 |
| `layer1/2_*.yaml` | 训练时 | 数据集配置 |

**加载顺序**: `cascade.yaml`(默认值) → `enemy.yaml`(赛前参数覆盖)

**★ 激光工作方式**: 上电即常亮，不存在"开不开火"的概念。系统输出的是瞄准坐标+瞄准质量评估。
