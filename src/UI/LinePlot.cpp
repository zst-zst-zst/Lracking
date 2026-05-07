#include "LinePlot.h"

#include <QFontMetrics>
#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

namespace tracking_app {

LinePlot::LinePlot(const QString& title, QWidget* parent)
    : QWidget(parent), title_(title) {
    setMinimumHeight(140);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void LinePlot::addChannel(const QString& name, const QColor& color) {
    channels_.push_back({name, color, {}});
}

void LinePlot::appendSample(int idx, double v) {
    if (idx < 0 || idx >= channels_.size()) return;
    auto& c = channels_[idx];
    c.samples.push_back(v);
    while (static_cast<int>(c.samples.size()) > max_points_) c.samples.pop_front();
    update();
}

void LinePlot::appendSamples(const QVector<double>& vs) {
    int n = std::min<int>(vs.size(), channels_.size());
    for (int i = 0; i < n; ++i) {
        auto& c = channels_[i];
        c.samples.push_back(vs[i]);
        while (static_cast<int>(c.samples.size()) > max_points_) c.samples.pop_front();
    }
    update();
}

void LinePlot::setMaxPoints(int n) { max_points_ = std::max(10, n); }
void LinePlot::setYRange(double lo, double hi) { y_lo_ = lo; y_hi_ = hi; auto_y_ = false; }
void LinePlot::setAutoYRange(bool on) { auto_y_ = on; }
void LinePlot::clear() {
    for (auto& c : channels_) c.samples.clear();
    update();
}

void LinePlot::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const QRect r = rect();
    p.fillRect(r, QColor(20, 20, 20));

    constexpr int kPadL = 44, kPadR = 10, kPadT = 22, kPadB = 18;
    const QRect plot(r.left() + kPadL, r.top() + kPadT,
                     r.width() - kPadL - kPadR, r.height() - kPadT - kPadB);
    if (plot.width() <= 1 || plot.height() <= 1) return;

    // 轴框
    p.setPen(QColor(60, 60, 60));
    p.drawRect(plot);

    // 标题
    p.setPen(QColor(170, 170, 170));
    QFont tf = p.font(); tf.setPointSize(9); tf.setBold(true);
    p.setFont(tf);
    p.drawText(r.left() + kPadL, r.top() + 14, title_);

    // 计算 Y 范围
    double y_lo = y_lo_, y_hi = y_hi_;
    if (auto_y_) {
        y_lo = 1e30; y_hi = -1e30;
        for (const auto& c : channels_)
            for (double v : c.samples) {
                y_lo = std::min(y_lo, v);
                y_hi = std::max(y_hi, v);
            }
        if (y_lo > y_hi) { y_lo = 0; y_hi = 1; }
        if (std::abs(y_hi - y_lo) < 1e-9) { y_hi = y_lo + 1.0; }
        // 加 5% 边距
        double pad = (y_hi - y_lo) * 0.08;
        y_lo -= pad; y_hi += pad;
    }

    // Y 轴刻度 (3 道)
    QFont sf = p.font(); sf.setPointSize(8); sf.setBold(false);
    p.setFont(sf);
    p.setPen(QColor(120, 120, 120));
    for (int i = 0; i <= 2; ++i) {
        double f = i / 2.0;
        double y = plot.bottom() - f * plot.height();
        double v = y_lo + f * (y_hi - y_lo);
        p.setPen(QColor(40, 40, 40));
        p.drawLine(plot.left(), y, plot.right(), y);
        p.setPen(QColor(140, 140, 140));
        p.drawText(QRectF(0, y - 8, kPadL - 4, 16),
                   Qt::AlignRight | Qt::AlignVCenter,
                   QString::number(v, 'f', std::abs(y_hi - y_lo) > 10 ? 0 : 2));
    }

    // 各通道折线
    auto y2px = [&](double v) {
        double t = (v - y_lo) / (y_hi - y_lo);
        return plot.bottom() - t * plot.height();
    };

    int legend_x = plot.right() - 6;
    for (int ci = 0; ci < channels_.size(); ++ci) {
        const auto& c = channels_[ci];
        if (c.samples.size() < 2) continue;
        QPainterPath path;
        const int n = static_cast<int>(c.samples.size());
        const double dx = static_cast<double>(plot.width()) / (max_points_ - 1);
        // 右对齐: 最新样本在右
        const double x0 = plot.right() - (n - 1) * dx;
        for (int i = 0; i < n; ++i) {
            double x = x0 + i * dx;
            double y = y2px(c.samples[i]);
            if (i == 0) path.moveTo(x, y);
            else        path.lineTo(x, y);
        }
        p.setPen(QPen(c.color, 1.6));
        p.drawPath(path);

        // 图例
        QFontMetrics fm(p.font());
        int lw = fm.horizontalAdvance(c.name) + 18;
        p.setPen(c.color);
        p.drawLine(legend_x - lw, plot.top() - 6, legend_x - lw + 12, plot.top() - 6);
        p.drawText(legend_x - lw + 14, plot.top() - 2, c.name);
        legend_x -= (lw + 8);
    }
}

}  // namespace tracking_app
