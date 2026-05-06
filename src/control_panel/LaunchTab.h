#ifndef CONTROL_PANEL_LAUNCH_TAB_H
#define CONTROL_PANEL_LAUNCH_TAB_H

#include <QWidget>

class QComboBox;
class QLineEdit;
class QCheckBox;
class QPushButton;
class QPlainTextEdit;
class QLabel;

namespace tracking_app {

class ProcessManager;

// 启动 tab: 模式切换 (调试/比赛) + 参数 + Start/Stop + 可折叠日志窗
class LaunchTab : public QWidget {
    Q_OBJECT

public:
    explicit LaunchTab(QWidget* parent = nullptr);

    // 暴露给 RecordsTab: 设置 video 路径并切到调试模式
    void setVideoPath(const QString& path);

private slots:
    void onStart();
    void onStop();
    void onBrowseVideo();
    void onProcStarted(qint64 pid);
    void onProcStopped(int code, int status);
    void onProcOutput(const QString& line);
    void onProcError(const QString& msg);
    void onToggleLog();

private:
    void buildUi();
    QString resolveExecutable() const;
    QStringList collectArgs() const;
    void setRunningState(bool running);

    QComboBox*   mode_combo_   = nullptr;   // 调试 (tracking) / 比赛 (tracking_main)
    QComboBox*   color_combo_  = nullptr;   // red / blue
    QLineEdit*   port_edit_    = nullptr;
    QLineEdit*   video_edit_   = nullptr;
    QPushButton* video_browse_ = nullptr;
    QCheckBox*   no_gimbal_    = nullptr;
    QCheckBox*   no_record_    = nullptr;
    QPushButton* start_btn_    = nullptr;
    QPushButton* stop_btn_     = nullptr;
    QLabel*      status_lbl_   = nullptr;

    QPushButton*    log_toggle_btn_ = nullptr;
    QPlainTextEdit* log_view_       = nullptr;

    ProcessManager* proc_ = nullptr;
};

}  // namespace tracking_app

#endif  // CONTROL_PANEL_LAUNCH_TAB_H
