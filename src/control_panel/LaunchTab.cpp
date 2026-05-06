#include "LaunchTab.h"
#include "ProcessManager.h"

#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QStringList>
#include <QVBoxLayout>

namespace tracking_app {

LaunchTab::LaunchTab(QWidget* parent) : QWidget(parent) {
    proc_ = new ProcessManager(this);
    connect(proc_, &ProcessManager::started,
            this, &LaunchTab::onProcStarted);
    connect(proc_, &ProcessManager::stopped,
            this, [this](int code, QProcess::ExitStatus s) { onProcStopped(code, static_cast<int>(s)); });
    connect(proc_, &ProcessManager::output,
            this, &LaunchTab::onProcOutput);
    connect(proc_, &ProcessManager::error,
            this, &LaunchTab::onProcError);
    buildUi();
    setRunningState(false);
}

void LaunchTab::buildUi() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(20, 20, 20, 20);
    root->setSpacing(14);

    // ── 模式 + 参数表单 ──
    auto* form_box = new QGroupBox("启动参数");
    auto* form = new QFormLayout(form_box);
    form->setHorizontalSpacing(14);
    form->setVerticalSpacing(10);

    mode_combo_ = new QComboBox;
    mode_combo_->addItem("调试 (tracking, 桌面/视频测试)", "tracking");
    mode_combo_->addItem("比赛 (tracking_main, 真机完整链路)", "tracking_main");
    form->addRow("模式", mode_combo_);

    color_combo_ = new QComboBox;
    color_combo_->addItems({"red", "blue"});
    form->addRow("敌方颜色", color_combo_);

    port_edit_ = new QLineEdit("/dev/ttyUSB0");
    form->addRow("串口", port_edit_);

    auto* video_row = new QHBoxLayout;
    video_edit_ = new QLineEdit;
    video_edit_->setPlaceholderText("(留空使用相机) 或 选择视频文件");
    video_browse_ = new QPushButton("浏览…");
    connect(video_browse_, &QPushButton::clicked, this, &LaunchTab::onBrowseVideo);
    video_row->addWidget(video_edit_, 1);
    video_row->addWidget(video_browse_);
    form->addRow("视频回放", video_row);

    auto* checks = new QHBoxLayout;
    no_gimbal_ = new QCheckBox("不接云台");
    no_record_ = new QCheckBox("不录制");
    checks->addWidget(no_gimbal_);
    checks->addWidget(no_record_);
    checks->addStretch();
    form->addRow("选项", checks);

    root->addWidget(form_box);

    // ── 控制按钮 + 状态 ──
    auto* ctrl_row = new QHBoxLayout;
    start_btn_ = new QPushButton("▶  Start");
    start_btn_->setMinimumHeight(36);
    start_btn_->setStyleSheet("QPushButton { font-weight: bold; background: #2d6e2d; }"
                              "QPushButton:hover { background: #3a8a3a; }");
    stop_btn_ = new QPushButton("■  Stop");
    stop_btn_->setMinimumHeight(36);
    stop_btn_->setStyleSheet("QPushButton { font-weight: bold; background: #6e2d2d; }"
                             "QPushButton:hover { background: #8a3a3a; }");
    connect(start_btn_, &QPushButton::clicked, this, &LaunchTab::onStart);
    connect(stop_btn_, &QPushButton::clicked, this, &LaunchTab::onStop);
    status_lbl_ = new QLabel("● 已停止");
    status_lbl_->setStyleSheet("color: #aaaaaa;");
    ctrl_row->addWidget(start_btn_);
    ctrl_row->addWidget(stop_btn_);
    ctrl_row->addSpacing(20);
    ctrl_row->addWidget(status_lbl_);
    ctrl_row->addStretch();
    root->addLayout(ctrl_row);

    // ── 可折叠日志窗 (默认折叠) ──
    log_toggle_btn_ = new QPushButton("▶ 显示日志");
    log_toggle_btn_->setStyleSheet("QPushButton { text-align: left; padding: 4px 10px; background: transparent; border: none; color: #888; }"
                                   "QPushButton:hover { color: #ccc; }");
    log_toggle_btn_->setCheckable(true);
    connect(log_toggle_btn_, &QPushButton::clicked, this, &LaunchTab::onToggleLog);
    root->addWidget(log_toggle_btn_);

    log_view_ = new QPlainTextEdit;
    log_view_->setReadOnly(true);
    log_view_->setMaximumBlockCount(2000);
    log_view_->setStyleSheet("QPlainTextEdit { font-family: 'Source Code Pro', 'Courier New', monospace; font-size: 11px; background: #141414; color: #c0c0c0; }");
    log_view_->setVisible(false);
    root->addWidget(log_view_, 1);
}

void LaunchTab::onToggleLog() {
    bool show = log_toggle_btn_->isChecked();
    log_view_->setVisible(show);
    log_toggle_btn_->setText(show ? "▼ 隐藏日志" : "▶ 显示日志");
}

QString LaunchTab::resolveExecutable() const {
    QString target = mode_combo_->currentData().toString();
    QString app_dir = QCoreApplication::applicationDirPath();  // build/bin
    return QFileInfo(app_dir + "/" + target).absoluteFilePath();
}

QStringList LaunchTab::collectArgs() const {
    QStringList args;
    args << "--color" << color_combo_->currentText();
    args << "--port" << port_edit_->text();
    if (!video_edit_->text().isEmpty()) args << "--video" << video_edit_->text();
    if (no_gimbal_->isChecked()) args << "--no-gimbal";
    if (no_record_->isChecked()) args << "--no-record";
    return args;
}

void LaunchTab::onStart() {
    QString exe = resolveExecutable();
    if (!QFileInfo::exists(exe)) {
        onProcError(QString("二进制不存在: %1\n请先 cmake --build build").arg(exe));
        return;
    }
    QStringList args = collectArgs();
    log_view_->appendPlainText(QString("[run] %1 %2").arg(exe, args.join(' ')));
    // 工作目录: 项目根 (cwd 影响 config/ logs/ 等相对路径)
    QString proj_root = QFileInfo(exe).dir().absolutePath() + "/../..";
    proj_root = QFileInfo(proj_root).absoluteFilePath();
    proc_->start(exe, args, proj_root);
}

void LaunchTab::onStop() {
    proc_->stop();
}

void LaunchTab::onBrowseVideo() {
    QString f = QFileDialog::getOpenFileName(this, "选择视频", QString(),
                                             "视频文件 (*.mp4 *.avi *.mov *.mkv);;所有文件 (*)");
    if (!f.isEmpty()) video_edit_->setText(f);
}

void LaunchTab::setVideoPath(const QString& path) {
    video_edit_->setText(path);
    // 切回调试模式 (回放只在 tracking 入口里有意义)
    int idx = mode_combo_->findData("tracking");
    if (idx >= 0) mode_combo_->setCurrentIndex(idx);
}

void LaunchTab::onProcStarted(qint64 pid) {
    setRunningState(true);
    status_lbl_->setText(QString("● 运行中  pid=%1").arg(pid));
    status_lbl_->setStyleSheet("color: #6ed46e;");
}

void LaunchTab::onProcStopped(int code, int /*status*/) {
    setRunningState(false);
    status_lbl_->setText(QString("● 已停止  退出码=%1").arg(code));
    status_lbl_->setStyleSheet(code == 0 ? "color: #aaaaaa;" : "color: #d46e6e;");
    emit procStopped();
}

void LaunchTab::onProcOutput(const QString& line) {
    log_view_->appendPlainText(line);
    emit procOutput(line);
}

void LaunchTab::onProcError(const QString& msg) {
    log_view_->appendPlainText(QString("[error] %1").arg(msg));
    // 错误时强制展开日志窗
    if (!log_toggle_btn_->isChecked()) {
        log_toggle_btn_->setChecked(true);
        onToggleLog();
    }
}

void LaunchTab::setRunningState(bool running) {
    start_btn_->setEnabled(!running);
    stop_btn_->setEnabled(running);
    mode_combo_->setEnabled(!running);
    color_combo_->setEnabled(!running);
    port_edit_->setEnabled(!running);
    video_edit_->setEnabled(!running);
    video_browse_->setEnabled(!running);
    no_gimbal_->setEnabled(!running);
    no_record_->setEnabled(!running);
}

}  // namespace tracking_app
