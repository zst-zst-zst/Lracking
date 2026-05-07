#include "StatusTab.h"

#include <QDateTime>
#include <QFont>
#include <QFormLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QLabel>
#include <QRegularExpression>
#include <QVBoxLayout>

namespace tracking_app {

namespace {

// 大数字标签: 突出显示关键指标
QLabel* makeBigLabel(const QString& init = "—") {
    auto* lbl = new QLabel(init);
    QFont f = lbl->font();
    f.setPointSize(28);
    f.setBold(true);
    lbl->setFont(f);
    lbl->setAlignment(Qt::AlignCenter);
    lbl->setStyleSheet("color: #6ed4d4; padding: 14px 20px;");
    lbl->setMinimumWidth(160);
    return lbl;
}

QGroupBox* makeCard(const QString& title, QWidget* content) {
    auto* box = new QGroupBox(title);
    auto* l = new QVBoxLayout(box);
    l->setContentsMargins(10, 18, 10, 10);
    l->addWidget(content);
    return box;
}

}  // namespace

StatusTab::StatusTab(QWidget* parent) : QWidget(parent) {
    buildUi();
}

void StatusTab::buildUi() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(20, 20, 20, 20);
    root->setSpacing(14);

    // ── 第一行: 大数字 (FPS / Tracks) ──
    auto* row1 = new QGridLayout;
    row1->setHorizontalSpacing(14);

    fps_lbl_ = makeBigLabel();
    tracks_lbl_ = makeBigLabel();

    row1->addWidget(makeCard("FPS", fps_lbl_), 0, 0);
    row1->addWidget(makeCard("跟踪目标数", tracks_lbl_), 0, 1);
    root->addLayout(row1);

    // ── 第二行: 主目标信息 ──
    auto* primary_box = new QGroupBox("主目标");
    auto* form = new QFormLayout(primary_box);
    form->setHorizontalSpacing(20);
    form->setVerticalSpacing(8);

    primary_lbl_ = new QLabel("—");
    color_lbl_   = new QLabel("—");
    conf_lbl_    = new QLabel("—");
    enemy_lbl_   = new QLabel("—");

    QString lbl_style = "font-size: 16px; padding: 4px 8px;";
    primary_lbl_->setStyleSheet(lbl_style);
    color_lbl_->setStyleSheet(lbl_style);
    conf_lbl_->setStyleSheet(lbl_style);
    enemy_lbl_->setStyleSheet(lbl_style);

    form->addRow("主目标 ID", primary_lbl_);
    form->addRow("当前颜色", color_lbl_);
    form->addRow("置信度", conf_lbl_);
    form->addRow("敌方设定", enemy_lbl_);

    root->addWidget(primary_box);

    // ── 第三行: 最后更新时间 ──
    last_seen_lbl_ = new QLabel("● 等待 tracking 启动…");
    last_seen_lbl_->setStyleSheet("color: #888; padding: 8px;");
    root->addWidget(last_seen_lbl_);

    root->addStretch();
}

void StatusTab::onProcLine(const QString& line) {
    // STATUS fps=120.5 tracks=2 primary_id=42 color=blue conf=0.91 enemy=red
    if (!line.startsWith("STATUS ")) return;

    static const QRegularExpression re_fps    ("fps=([\\d.]+)");
    static const QRegularExpression re_tracks ("tracks=(\\d+)");
    static const QRegularExpression re_pid    ("primary_id=(-?\\d+)");
    static const QRegularExpression re_color  ("color=(\\S+)");
    static const QRegularExpression re_conf   ("conf=([\\d.]+)");
    static const QRegularExpression re_enemy  ("enemy=(\\S+)");

    auto get_d = [&](const QRegularExpression& re, double def) {
        auto m = re.match(line);
        return m.hasMatch() ? m.captured(1).toDouble() : def;
    };
    auto get_i = [&](const QRegularExpression& re, int def) {
        auto m = re.match(line);
        return m.hasMatch() ? m.captured(1).toInt() : def;
    };
    auto get_s = [&](const QRegularExpression& re, const QString& def) {
        auto m = re.match(line);
        return m.hasMatch() ? m.captured(1) : def;
    };

    updateLabels(get_d(re_fps, 0),
                 get_i(re_tracks, 0),
                 get_i(re_pid, -1),
                 get_s(re_color, "none"),
                 get_d(re_conf, 0),
                 get_s(re_enemy, "—"));
}

void StatusTab::onProcStopped() {
    fps_lbl_->setText("—");
    tracks_lbl_->setText("—");
    primary_lbl_->setText("—");
    color_lbl_->setText("—");
    conf_lbl_->setText("—");
    enemy_lbl_->setText("—");
    last_seen_lbl_->setText("● tracking 已停止");
    last_seen_lbl_->setStyleSheet("color: #d46e6e; padding: 8px;");
}

void StatusTab::updateLabels(double fps, int tracks, int primary_id,
                             const QString& color, double conf, const QString& enemy) {
    fps_lbl_->setText(QString::number(fps, 'f', 1));
    fps_lbl_->setStyleSheet(fps >= 60.0
                            ? "color: #6ed46e; padding: 14px 20px;"   // 绿
                            : (fps >= 30.0 ? "color: #d4d46e; padding: 14px 20px;" // 黄
                                            : "color: #d46e6e; padding: 14px 20px;")); // 红

    tracks_lbl_->setText(QString::number(tracks));
    tracks_lbl_->setStyleSheet(tracks > 0
                               ? "color: #6ed46e; padding: 14px 20px;"
                               : "color: #888; padding: 14px 20px;");

    primary_lbl_->setText(primary_id < 0 ? "(无)" : QString::number(primary_id));

    QString color_style = "font-size: 16px; padding: 4px 8px; ";
    if (color == "red")        color_style += "color: #ff4444; font-weight: bold;";
    else if (color == "blue")  color_style += "color: #4488ff; font-weight: bold;";
    else if (color == "purple") color_style += "color: #c470ff; font-weight: bold;";
    else                       color_style += "color: #888;";
    color_lbl_->setText(color);
    color_lbl_->setStyleSheet(color_style);

    conf_lbl_->setText(QString::number(conf, 'f', 2));
    enemy_lbl_->setText(enemy);

    last_seen_lbl_->setText("● 最后更新 " + QDateTime::currentDateTime().toString("HH:mm:ss"));
    last_seen_lbl_->setStyleSheet("color: #6ed46e; padding: 8px;");
}

}  // namespace tracking_app
