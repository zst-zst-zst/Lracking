# 配置文件说明

所有配置集中在 `config/` 目录下。

## 文件列表

| 文件 | 用途 | 何时修改 |
|------|------|----------|
| `camera.yaml` | 相机内参、曝光、ROI、GPU 加速 | 换镜头/换相机时 |
| `enemy.yaml` | 敌方颜色、位置、稳定性预设 | **每场比赛前** |
| `cascade.yaml` | 检测模型、跟踪器、灯带拒绝、比赛规则 | 调参时 |
| `control.yaml` | PID 控制、云台参数、激光偏移 | 调参时 |

## 赛前检查清单

1. 修改 `enemy.yaml` 中的 `enemy_color` 和 `enemy_side`
2. 确认 `camera.yaml` 的曝光参数适合赛场光照
3. 其他参数一般不用改

## 运行方式

所有可执行文件都从**项目根目录**运行，配置路径自动正确：

```bash
# 从项目根目录
./tests/build/track
./tests/build/hit
./tests/build/detect_demo
./src/calibrate/build/laser_boresight
```
