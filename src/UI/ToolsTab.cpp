#include "ToolsTab.h"
#include "ProcessManager.h"

#include <QCoreApplication>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QVBoxLayout>

namespace tracking_app {

ToolsTab::ToolsTab(QWidget* parent) : QWidget(parent) {
    proc_ = new ProcessManager(this);
    connect(proc_, &ProcessManager::started, this, &ToolsTab::onProcStarted);
    connect(proc_, &ProcessManager::stopped, this,
            [this](int c, QProcess::ExitStatus s) { onProcStopped(c, static_cast<int>(s)); });
    connect(proc_, &ProcessManager::output, this, &ToolsTab::onProcOutput);
    connect(proc_, &ProcessManager::error, this, &ToolsTab::onProcError);

    // 工具清单 (相对项目根)
    tools_ = {
        {"挖 layer1 负样本",
         "python3", {"tools/mine_hard_negatives.py", "--from-layer1"},
         "从 layer1 数据集 ROI 中挖掘 layer2 的硬负样本"},
        {"挖视频负样本",
         "python3", {"tools/mine_hard_negatives.py", "--from-videos"},
         "从 records/ 视频中挖掘硬负样本 (耗时长)"},
        {"YOLOv11 微调",
         "python3", {"tools/mine_hard_negatives.py", "--finetune"},
         "用挖掘到的负样本微调 layer2 模型"},
        {"部署 layer2 (ONNX+TRT)",
         "bash", {"tools/deploy_retrained_layer2.sh"},
         "导出 ONNX → TensorRT engine → 同步到 weights/"},
        {"激光标定 GUI",
         "python3", {"tools/laser_calib_gui.py"},
         "弹出独立窗口标定激光偏置"},
        {"SR 训练",
         "python3", {"tools/sr_train.py"},
         "训练超分模型"},
        {"SR 导出",
         "python3", {"tools/sr_export.py"},
         "导出超分 ONNX/engine"},
    };
    // 标定 GUI 用 detached
    if (tools_.size() > 4) tools_[4].detached = true;

    buildUi();
}

QString ToolsTab::projectRoot() const {
    return QFileInfo(QCoreApplication::applicationDirPath() + "/../..").absoluteFilePath();
}

void ToolsTab::buildUi() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(20, 20, 20, 20);
    root->setSpacing(12);

    auto* tools_box = new QGroupBox("工具脚本");
    auto* grid = new QGridLayout(tools_box);
    grid->setHorizontalSpacing(10);
    grid->setVerticalSpacing(8);
    constexpr int kCols = 3;
    for (int i = 0; i < tools_.size(); ++i) {
        const auto& t = tools_[i];
        auto* btn = new QPushButton(t.label);
        btn->setMinimumHeight(38);
        btn->setToolTip(QString("%1\n→ %2 %3").arg(t.tooltip, t.program, t.args.join(' ')));
        connect(btn, &QPushButton::clicked, this, [this, i] { onRunTool(i); });
        buttons_.push_back(btn);
        grid->addWidget(btn, i / kCols, i % kCols);
    }
    root->addWidget(tools_box);

    // ── 控制行 ──
    auto* ctrl = new QHBoxLayout;
    stop_btn_ = new QPushButton("■ 终止当前任务");
    stop_btn_->setStyleSheet("QPushButton { background: #6e2d2d; }"
                             "QPushButton:hover { background: #8a3a3a; }");
    stop_btn_->setEnabled(false);
    connect(stop_btn_, &QPushButton::clicked, this, &ToolsTab::onStop);
    status_lbl_ = new QLabel("● 空闲");
    status_lbl_->setStyleSheet("color: #aaa;");
    ctrl->addWidget(stop_btn_);
    ctrl->addSpacing(20);
    ctrl->addWidget(status_lbl_);
    ctrl->addStretch();
    root->addLayout(ctrl);

    // ── 日志窗 (常驻显示, 工具脚本输出量大需要看) ──
    log_view_ = new QPlainTextEdit;
    log_view_->setReadOnly(true);
    log_view_->setMaximumBlockCount(5000);
    log_view_->setStyleSheet("QPlainTextEdit { font-family: 'Source Code Pro', 'Courier New', monospace;"
                             "font-size: 11px; background: #141414; color: #c0c0c0; }");
    root->addWidget(log_view_, 1);
}

void ToolsTab::onRunTool(int index) {
    if (index < 0 || index >= tools_.size()) return;
    if (proc_->isRunning()) {
        QMessageBox::warning(this, "忙",
            "已有任务在运行, 请先终止或等待结束.");
        return;
    }
    const auto& t = tools_[index];
    QString cwd = projectRoot();

    if (t.detached) {
        // 独立 GUI 用 startDetached, 不占用 ToolsTab 的进程槽
        log_view_->appendPlainText(QString("[detached] %1 %2  (cwd=%3)")
            .arg(t.program, t.args.join(' '), cwd));
        bool ok = QProcess::startDetached(t.program, t.args, cwd);
        if (!ok) onProcError("无法启动 detached 进程");
        return;
    }

    log_view_->appendPlainText(QString("\n========  %1  ========").arg(t.label));
    log_view_->appendPlainText(QString("[run] %1 %2  (cwd=%3)")
        .arg(t.program, t.args.join(' '), cwd));
    proc_->start(t.program, t.args, cwd);
}

void ToolsTab::onStop() {
    proc_->stop();
}

void ToolsTab::onProcStarted(qint64 pid) {
    status_lbl_->setText(QString("● 运行中  pid=%1").arg(pid));
    status_lbl_->setStyleSheet("color: #6ed46e;");
    stop_btn_->setEnabled(true);
    for (auto* b : buttons_) b->setEnabled(false);
}

void ToolsTab::onProcStopped(int code, int /*status*/) {
    status_lbl_->setText(QString("● 完成  退出码=%1").arg(code));
    status_lbl_->setStyleSheet(code == 0 ? "color: #aaa;" : "color: #d46e6e;");
    stop_btn_->setEnabled(false);
    for (auto* b : buttons_) b->setEnabled(true);
    log_view_->appendPlainText(QString("[done] exit=%1\n").arg(code));
}

void ToolsTab::onProcOutput(const QString& line) {
    log_view_->appendPlainText(line);
}

void ToolsTab::onProcError(const QString& msg) {
    log_view_->appendPlainText(QString("[error] %1").arg(msg));
    status_lbl_->setText("● 错误");
    status_lbl_->setStyleSheet("color: #d46e6e;");
}

}  // namespace tracking_app
