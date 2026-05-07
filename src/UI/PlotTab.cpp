#include "PlotTab.h"
#include "LinePlot.h"
#include "UdpPlotReceiver.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRegularExpression>
#include <QVBoxLayout>

namespace tracking_app {

PlotTab::PlotTab(QWidget* parent) : QWidget(parent) {
    buildUi();

    udp_ = new UdpPlotReceiver(9870, this);
    connect(udp_, &UdpPlotReceiver::samples, this, &PlotTab::onUdpSamples);
    connect(udp_, &UdpPlotReceiver::boundChanged, this, &PlotTab::onUdpBoundChanged);
    onUdpBoundChanged(udp_->isBound(),
                      udp_->isBound()
                          ? "UDP 9870 监听中 (与 PlotJuggler 共享)"
                          : QString("UDP 9870 绑定失败: %1").arg(udp_->lastError()));
}

void PlotTab::buildUi() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(14, 10, 14, 10);
    root->setSpacing(8);

    src_lbl_ = new QLabel;
    src_lbl_->setStyleSheet("color: #aaa; font-size: 11px;");
    root->addWidget(src_lbl_);

    // ── plotter (UDP) 曲线 ──
    yaw_plot_ = new LinePlot("Yaw  (cmd / state / error, deg)");
    yaw_plot_->addChannel("cmd_yaw",   QColor(110, 212, 110));
    yaw_plot_->addChannel("state_yaw", QColor( 90, 170, 230));
    yaw_plot_->addChannel("error_yaw", QColor(230, 110, 110));
    yaw_plot_->setMaxPoints(600);

    pitch_plot_ = new LinePlot("Pitch  (cmd / state / error, deg)");
    pitch_plot_->addChannel("cmd_pitch",   QColor(110, 212, 110));
    pitch_plot_->addChannel("state_pitch", QColor( 90, 170, 230));
    pitch_plot_->addChannel("error_pitch", QColor(230, 110, 110));
    pitch_plot_->setMaxPoints(600);

    pix_plot_ = new LinePlot("目标像素 (target_u / target_v)");
    pix_plot_->addChannel("target_u", QColor(212, 180, 110));
    pix_plot_->addChannel("target_v", QColor(180, 130, 220));
    pix_plot_->setMaxPoints(600);

    bbox_plot_ = new LinePlot("bbox_h  (近似距离反比)");
    bbox_plot_->addChannel("bbox_h", QColor(220, 200, 110));
    bbox_plot_->setMaxPoints(600);

    // ── STATUS (stdout) 曲线 ──
    fps_plot_ = new LinePlot("FPS");
    fps_plot_->addChannel("fps", QColor(110, 212, 110));
    fps_plot_->setMaxPoints(600);

    track_plot_ = new LinePlot("tracks");
    track_plot_->addChannel("tracks", QColor(110, 200, 220));
    track_plot_->setMaxPoints(600);

    // 2 列网格: yaw | pitch / pix | bbox / fps | tracks
    auto* grid = new QGridLayout;
    grid->setHorizontalSpacing(8);
    grid->setVerticalSpacing(8);
    grid->addWidget(yaw_plot_,   0, 0);
    grid->addWidget(pitch_plot_, 0, 1);
    grid->addWidget(pix_plot_,   1, 0);
    grid->addWidget(bbox_plot_,  1, 1);
    grid->addWidget(fps_plot_,   2, 0);
    grid->addWidget(track_plot_, 2, 1);
    grid->setRowStretch(0, 1);
    grid->setRowStretch(1, 1);
    grid->setRowStretch(2, 1);
    root->addLayout(grid, 1);

    auto* clear_btn = new QPushButton("清空");
    clear_btn->setMaximumWidth(80);
    connect(clear_btn, &QPushButton::clicked, this, [this] {
        yaw_plot_->clear(); pitch_plot_->clear();
        pix_plot_->clear(); bbox_plot_->clear();
        fps_plot_->clear(); track_plot_->clear();
    });

    auto* bottom = new QHBoxLayout;
    bottom->addWidget(clear_btn);
    bottom->addStretch();
    root->addLayout(bottom);

    // ── 速查: 看图调参 (跟 docs/调参速查.md 同源, 改 control.yaml 即时生效) ──
    auto* tip = new QLabel;
    tip->setTextFormat(Qt::RichText);
    tip->setWordWrap(true);
    tip->setText(
        "<span style='color:#bbb;'>"
        "<b>调参速查</b> (改 <code style='color:#9cf'>control.yaml</code> 即时生效) &nbsp;&nbsp; "
        "<b style='color:#6ed46e'>跟得慢→</b> <code>↑kp</code> → <code>+ff_gain</code> → <code>↑lowpass_alpha</code> &nbsp;&nbsp; "
        "<b style='color:#d46e6e'>振荡→</b> <code>↓kp</code> → <code>+damping_kd</code> → <code>↓lowpass_alpha</code> &nbsp;&nbsp; "
        "<b style='color:#d4b46e'>稳态误差→</b> <code>+ki</code> (kp/ff 调好再开)"
        "</span>");
    tip->setStyleSheet(
        "QLabel { padding: 8px 12px; background: #181818;"
        "         border: 1px solid #333; border-radius: 4px; }");
    tip->setMinimumHeight(36);
    root->addWidget(tip);
}

void PlotTab::onUdpBoundChanged(bool bound, const QString& note) {
    src_lbl_->setText(QString("● %1").arg(note));
    src_lbl_->setStyleSheet(bound ? "color: #6ed46e; font-size: 11px;"
                                  : "color: #d46e6e; font-size: 11px;");
}

void PlotTab::onUdpSamples(const QHash<QString, double>& kv) {
    auto push = [&](LinePlot* p, int ch, const QString& key) {
        auto it = kv.constFind(key);
        if (it != kv.constEnd()) p->appendSample(ch, it.value());
    };
    push(yaw_plot_,   0, "cmd_yaw");
    push(yaw_plot_,   1, "state_yaw");
    push(yaw_plot_,   2, "error_yaw");
    push(pitch_plot_, 0, "cmd_pitch");
    push(pitch_plot_, 1, "state_pitch");
    push(pitch_plot_, 2, "error_pitch");
    push(pix_plot_,   0, "target_u");
    push(pix_plot_,   1, "target_v");
    push(bbox_plot_,  0, "bbox_h");
}

void PlotTab::onProcLine(const QString& line) {
    if (!line.startsWith("STATUS ")) return;
    static const QRegularExpression re_fps   ("fps=([\\d.]+)");
    static const QRegularExpression re_track ("tracks=(\\d+)");
    auto m = re_fps.match(line);
    if (m.hasMatch()) fps_plot_->appendSample(0, m.captured(1).toDouble());
    m = re_track.match(line);
    if (m.hasMatch()) track_plot_->appendSample(0, m.captured(1).toDouble());
}

void PlotTab::onProcStopped() {
    // 留作回看, 不清空
}

}  // namespace tracking_app
