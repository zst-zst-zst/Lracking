#ifndef CONTROL_PANEL_PLOT_TAB_H
#define CONTROL_PANEL_PLOT_TAB_H

#include <QHash>
#include <QString>
#include <QWidget>

class QLabel;

namespace tracking_app {

class LinePlot;
class UdpPlotReceiver;

// 实时曲线 tab. 数据源:
//   1. UDP 9870 (与 tracking::plotter / 外部 PlotJuggler 同源):
//      cmd_yaw / state_yaw / error_yaw
//      cmd_pitch / state_pitch / error_pitch
//      target_u / target_v / bbox_h
//   2. tracking stdout 的 STATUS 行:
//      fps / tracks / conf
class PlotTab : public QWidget {
    Q_OBJECT

public:
    explicit PlotTab(QWidget* parent = nullptr);

public slots:
    void onProcLine(const QString& line);
    void onProcStopped();

private slots:
    void onUdpSamples(const QHash<QString, double>& kv);
    void onUdpBoundChanged(bool bound, const QString& note);

private:
    void buildUi();

    // STATUS 来源
    LinePlot* fps_plot_   = nullptr;
    LinePlot* track_plot_ = nullptr;

    // UDP plotter 来源
    LinePlot* yaw_plot_   = nullptr;   // cmd_yaw / state_yaw / error_yaw
    LinePlot* pitch_plot_ = nullptr;   // cmd_pitch / state_pitch / error_pitch
    LinePlot* pix_plot_   = nullptr;   // target_u / target_v
    LinePlot* bbox_plot_  = nullptr;   // bbox_h

    QLabel*   src_lbl_    = nullptr;
    UdpPlotReceiver* udp_ = nullptr;
};

}  // namespace tracking_app

#endif  // CONTROL_PANEL_PLOT_TAB_H
