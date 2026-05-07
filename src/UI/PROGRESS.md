# Control Panel (Qt5 GUI) 开发进度

> 参考 `/home/zst/T/src/utils/rm_control_panel/` (ROS2 雷达站的 RmControlPanel)
> 目标：给 Tracking 项目做完整的 Qt5 启动器 + 配置面板

## 范围

完整版（参考 T 的 RmControlPanel 所有档次）：
- [x] 启动/启停 tracking 进程
- [ ] 进程预检 (串口/相机/模型存在)
- [ ] 常用任务按钮 (训练/导出/部署/标定)
- [ ] config tab × 3 (cascade.yaml / control.yaml / camera.yaml)
- [ ] 实时状态面板 (fps / 跟踪 ID / 命中率 / 当前敌方颜色)
- [ ] 视频回放选择 (records/ 浏览)
- [ ] 多进程调度 (tracking + plotter + 标定不冲突)
- [ ] 主题 (深色)
- [ ] 持久化偏好 (上次选择)

## 设计

### 架构
```
src/control_panel/
├── CMakeLists.txt         # Qt5 检测 + AUTOMOC + AUTOUIC
├── main.cpp               # QApplication 入口
├── ControlPanel.{h,cpp}   # 主窗口 (QMainWindow + tabs)
├── tabs/
│   ├── LaunchTab.{h,cpp}    # 启动/停止/参数
│   ├── ConfigTab.{h,cpp}    # 三个 yaml 编辑
│   ├── StatusTab.{h,cpp}    # 实时 status 显示
│   ├── RecordsTab.{h,cpp}   # 视频文件管理
│   └── ToolsTab.{h,cpp}     # 训练/导出/标定脚本调用
├── widgets/
│   ├── LogView.{h,cpp}      # tail-style 日志显示
│   └── StatusLight.{h,cpp}  # 状态指示灯
└── utils/
    ├── ProcessManager.{h,cpp}  # QProcess 包装
    └── YamlForm.{h,cpp}        # YAML <-> 表单字段绑定
```

### 与 tracking 进程通信
- **stdout/stderr**: tail 显示在日志窗
- **可选**: tracking 进程加 IPC (UDP / 命名管道) 推送实时状态
  → 暂用 stdout 解析 + 日志正则提取 fps/track_id/color

### Qt5 检测策略
顶层 CMakeLists.txt 加 `option(BUILD_CONTROL_PANEL "Build Qt5 GUI" AUTO)`
- AUTO: `find_package(Qt5)` 找到则启用, 否则跳过
- ON: 找不到则报错
- OFF: 不构建

## 实施步骤 (多会话)

### Session 1: 骨架 ✅
- [x] 顶层 CMakeLists.txt 加 BUILD_CONTROL_PANEL 选项 + Qt5 检测
- [x] src/control_panel/CMakeLists.txt 子模块入口 (AUTOMOC)
- [x] main.cpp + ControlPanel.{h,cpp} 空主窗口 (5 个 tab 占位)
- [x] 深色主题
- [x] 验证: cmake -B build -S . → "Qt5 found (5.15.13)", build 成功
- [x] 验证: ./build/bin/control_panel -platform offscreen 跑起来无错误
- 输出: build/bin/control_panel

### Session 2: LaunchTab + ProcessManager ✅
- [x] ProcessManager: QProcess 包装, 启动二进制 + 参数
- [x] **模式切换 dropdown**: 调试 (tracking, src/tracking/track.cpp) / 比赛 (tracking_main, src/control/examples/tracking_main.cpp)
- [x] LaunchTab: 颜色 combo / 端口 / video 文件 / no-gimbal checkbox / Start/Stop
- [x] 实时 stdout 接到 LogView (默认折叠, 用户可展开)

### Session 3: ConfigTab × 3 ✅
- [x] YamlForm: cv::FileStorage 加载 + 文本替换保存 (保留注释) 加载/保存 + 字段 <-> widget 双向绑定
- [x] cascade.yaml 表单 (阈值, conf 按色)
- [x] control.yaml 表单 (PID/FF gain)
- [x] camera.yaml 表单 (曝光/增益)

### Session 4: StatusTab + LogView ✅
- [x] (LogView 已在 Session 2 完成) + 颜色标记
- [x] StatusTab: 解析 tracking stdout 提取 fps/track/color, 实时 label 显示

### Session 5: RecordsTab + ToolsTab ✅
- [x] RecordsTab: 列出 records/ 视频 (大小+时间), 双击/送出 → LaunchTab.setVideoPath() + 自动切到启动 tab
- [x] RecordsTab: 切换目录 / 刷新 / 删除录像 (二次确认)
- [x] ToolsTab: 7 个一键脚本 (挖负样本×2 / 微调 / 部署 / 标定GUI / SR训练 / SR导出)
- [x] ToolsTab: 共用 ProcessManager, 实时日志窗 + 终止按钮; 标定 GUI 用 startDetached

### Session 6: 收尾 ✅
- [x] 主题 (深色 stylesheet, 已在 Session 1 完成)
- [x] **偏好持久化 (QSettings)**: LaunchTab 全部字段 + 窗口尺寸/状态/上次 tab + RecordsTab 目录
- [x] **预检按钮**: 二进制 / 配置 / 串口 / 视频 / /dev/video* 一键检查, 失败强制展开日志
- [x] **plotter 接入**: PlotTab 内嵌 LinePlot (FPS / tracks / conf) 滚动曲线, 喂 STATUS 行;
       外部 PlotJuggler 仍可监听 UDP:9870
- [x] **日志窗可折叠**: 已在 Session 2 完成 (LaunchTab `▶ 显示日志` toggle)

## 自定义 LinePlot 而非 QtCharts
- 无需额外 Qt 依赖 (Qt5Charts 不一定装)
- 多通道 / 自动 Y / 固定/自动量程 / 滚动窗口 / 图例 — 够用
- 后续若要更复杂图 (散点 / 双轴 / 鼠标交互), 再考虑 QtCharts 或 QCustomPlot

## 当前状态

**[Session 6 完成 ✅ — GUI 主体功能闭环]**

剩余可选增强:
- tracking stdout 加更多结构化字段 (yaw_err / pitch_err / ff_yaw / ff_pitch / cmd_yaw / cmd_pitch)
  → 让 PlotTab 直接画 PID 调试曲线 (替换 PlotJuggler 的核心场景)
- 多 tracking 进程并发管理 (当前一份 ProcessManager 实例)
- 主题切换 (亮/暗)
