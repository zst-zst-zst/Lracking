#ifndef CONTROL_PANEL_LINE_PLOT_H
#define CONTROL_PANEL_LINE_PLOT_H

#include <QColor>
#include <QString>
#include <QVector>
#include <QWidget>
#include <deque>

namespace tracking_app {

// 轻量滚动折线图: 多通道, 自动 Y 缩放, 固定窗口长度.
// 用 QPainter 绘制, 不依赖 QtCharts.
class LinePlot : public QWidget {
    Q_OBJECT

public:
    struct Channel {
        QString name;
        QColor  color;
        std::deque<double> samples;
    };

    explicit LinePlot(const QString& title, QWidget* parent = nullptr);

    void addChannel(const QString& name, const QColor& color);
    void appendSample(int channel_index, double v);
    void appendSamples(const QVector<double>& vs);   // 同序号
    void setMaxPoints(int n);
    void setYRange(double lo, double hi);            // 关闭自动缩放
    void setAutoYRange(bool on);
    void clear();

protected:
    void paintEvent(QPaintEvent*) override;

private:
    QString title_;
    QVector<Channel> channels_;
    int max_points_ = 240;     // ~ 4 秒 60Hz / 8 秒 30Hz
    bool auto_y_ = true;
    double y_lo_ = 0.0, y_hi_ = 1.0;
};

}  // namespace tracking_app

#endif  // CONTROL_PANEL_LINE_PLOT_H
