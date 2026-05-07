#ifndef CONTROL_PANEL_UDP_PLOT_RECEIVER_H
#define CONTROL_PANEL_UDP_PLOT_RECEIVER_H

#include <QHash>
#include <QObject>
#include <QString>

class QUdpSocket;

namespace tracking_app {

// 监听 tracking 进程的 plotter UDP JSON (默认 127.0.0.1:9870),
// 与外部 PlotJuggler 同源 (用 ShareAddress 共存).
//
// 协议: 每帧一个 JSON 对象, key=字段名, value=number.
// 例如: {"cmd_yaw":1.2,"state_yaw":1.1,...}
class UdpPlotReceiver : public QObject {
    Q_OBJECT

public:
    explicit UdpPlotReceiver(quint16 port = 9870, QObject* parent = nullptr);
    bool isBound() const { return bound_; }
    QString lastError() const { return last_err_; }

signals:
    // 每帧解析出的数值字段; key=曲线名, value=double
    void samples(const QHash<QString, double>& kv);
    // 绑定/解绑状态变化 (用于状态栏指示)
    void boundChanged(bool bound, const QString& note);

private slots:
    void onReadyRead();

private:
    QUdpSocket* sock_ = nullptr;
    quint16     port_ = 9870;
    bool        bound_ = false;
    QString     last_err_;
};

}  // namespace tracking_app

#endif  // CONTROL_PANEL_UDP_PLOT_RECEIVER_H
