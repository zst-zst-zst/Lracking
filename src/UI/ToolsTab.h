#ifndef CONTROL_PANEL_TOOLS_TAB_H
#define CONTROL_PANEL_TOOLS_TAB_H

#include <QString>
#include <QStringList>
#include <QVector>
#include <QWidget>

class QPlainTextEdit;
class QPushButton;
class QLabel;

namespace tracking_app {

class ProcessManager;

// Tools tab: 一键调用 tools/ 下的脚本.
//   - 训练 / 微调:     tools/mine_hard_negatives.py --finetune
//   - 挖负样本:        tools/mine_hard_negatives.py --from-layer1
//   - 部署 layer2:     tools/deploy_retrained_layer2.sh
//   - 激光标定 GUI:    tools/laser_calib_gui.py
//   - SR 训练 / 导出:  tools/sr_train.py / sr_export.py
struct ToolEntry {
    QString label;
    QString program;       // python3 / bash
    QStringList args;      // 完整参数 (相对项目根)
    QString tooltip;
    bool detached = false; // 标定 GUI 这种独立 GUI 用 detached 启动
};

class ToolsTab : public QWidget {
    Q_OBJECT

public:
    explicit ToolsTab(QWidget* parent = nullptr);

private slots:
    void onRunTool(int index);
    void onStop();
    void onProcStarted(qint64 pid);
    void onProcStopped(int code, int status);
    void onProcOutput(const QString& line);
    void onProcError(const QString& msg);

private:
    void buildUi();
    QString projectRoot() const;

    QVector<ToolEntry> tools_;
    QVector<QPushButton*> buttons_;
    QPushButton* stop_btn_ = nullptr;
    QLabel*      status_lbl_ = nullptr;
    QPlainTextEdit* log_view_ = nullptr;
    ProcessManager* proc_ = nullptr;
};

}  // namespace tracking_app

#endif  // CONTROL_PANEL_TOOLS_TAB_H
