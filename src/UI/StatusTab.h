#ifndef CONTROL_PANEL_STATUS_TAB_H
#define CONTROL_PANEL_STATUS_TAB_H

#include <QWidget>

class QLabel;

namespace tracking_app {

// 解析 tracking stdout 中的 STATUS 行, 实时显示
// 格式: STATUS fps=120.5 tracks=2 primary_id=42 color=blue conf=0.91 enemy=red
class StatusTab : public QWidget {
    Q_OBJECT

public:
    explicit StatusTab(QWidget* parent = nullptr);

public slots:
    // 由 LaunchTab 转发: 每条 stdout 行喂进来
    void onProcLine(const QString& line);
    // 进程结束清空
    void onProcStopped();

private:
    void buildUi();
    void updateLabels(double fps, int tracks, int primary_id,
                      const QString& color, double conf, const QString& enemy);

    QLabel* fps_lbl_       = nullptr;
    QLabel* tracks_lbl_    = nullptr;
    QLabel* primary_lbl_   = nullptr;
    QLabel* color_lbl_     = nullptr;
    QLabel* conf_lbl_      = nullptr;
    QLabel* enemy_lbl_     = nullptr;
    QLabel* last_seen_lbl_ = nullptr;
};

}  // namespace tracking_app

#endif  // CONTROL_PANEL_STATUS_TAB_H
