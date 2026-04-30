# laser_boresight — 激光同轴校准

自动检测激光光点，多距离采集拟合，一次校准适用所有距离。

## 原理

激光点在画面中的位置由两部分决定：

- **物理偏移** (dx, dy)：激光与相机不在同一点，近距离视差大
- **角度偏差** (α, β)：安装不完全平行，远距离也有恒定偏移

模型：`u = cx + fx·α + fx·dx/Z`

通过在多个距离采集激光点坐标，最小二乘拟合出 dx, dy, α, β。

## 构建

```bash
cd src/calibrate/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## 使用

```bash
./laser_boresight \
  --config ../../../config/camera.yaml \
  --control ../control/config/control.yaml \
  --port /dev/ttyUSB0
```

## 操作流程

1. WASD 控制云台，激光照白墙
2. 按 `z` 输入距离（米），开始自动采集
3. 按 `c` 保存当前距离样本
4. 换距离重复（建议 0.5, 1, 2, 3, 5, 10m 等）
5. 按 `f` 拟合
6. 按 `r` 保存到 control.yaml

## 快捷键

| 按键 | 功能 |
|------|------|
| W/A/S/D | 云台控制 |
| e | 切换低/正常曝光 |
| +/- | 调节检测阈值 |
| z | 输入距离，开始采集 |
| c | 保存当前样本 |
| x | 删除上一个样本 |
| f | 拟合计算 |
| r | 保存结果 |
| q/ESC | 退出 |
