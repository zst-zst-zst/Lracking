#ifndef CONTROL_PANEL_H
#define CONTROL_PANEL_H

#include <QMainWindow>

class QTabWidget;
class QLabel;
class QStatusBar;

namespace tracking_app {

// Tracking 项目的 Qt5 启动器 + 配置面板.
// 灵感来自 /home/zst/T/src/utils/rm_control_panel/RmControlPanel.h
//
// 五个 tab:
//   Launch   - 启动/停止 tracking 进程, 选颜色/端口/video
//   Config   - 编辑 cascade.yaml / control.yaml / camera.yaml
//   Status   - 实时 fps / 跟踪 ID / 命中率
//   Records  - 浏览 records/ 视频, 双击送给 Launch
//   Tools    - 训练/导出/部署/标定 一键按钮
//
// 当前为 Session 1 骨架, tab 内为占位.
class ControlPanel : public QMainWindow {
    Q_OBJECT

public:
    explicit ControlPanel(QWidget* parent = nullptr);
    ~ControlPanel() override = default;

protected:
    void closeEvent(QCloseEvent* e) override;

private:
    void buildUi();
    void applyDarkTheme();
    void restoreGeometryFromSettings();

    QTabWidget* tabs_ = nullptr;
    QLabel* status_label_ = nullptr;
};

}  // namespace tracking_app

#endif  // CONTROL_PANEL_H
