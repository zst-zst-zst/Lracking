#include "LaunchTab.h"
#include "ProcessManager.h"

#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QSettings>
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
    loadSettings();
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
    // 任何变更都持久化
    auto save_on_change = [this] { saveSettings(); };
    connect(no_gimbal_, &QCheckBox::toggled, this, save_on_change);
    connect(no_record_, &QCheckBox::toggled, this, save_on_change);
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
    preflight_btn_ = new QPushButton("✓ 预检");
    preflight_btn_->setMinimumHeight(36);
    preflight_btn_->setStyleSheet("QPushButton { background: #2d4a6e; }"
                                  "QPushButton:hover { background: #3a5a8a; }");
    connect(start_btn_, &QPushButton::clicked, this, &LaunchTab::onStart);
    connect(stop_btn_, &QPushButton::clicked, this, &LaunchTab::onStop);
    connect(preflight_btn_, &QPushButton::clicked, this, &LaunchTab::onPreflight);
    status_lbl_ = new QLabel("● 已停止");
    status_lbl_->setStyleSheet("color: #aaaaaa;");
    ctrl_row->addWidget(start_btn_);
    ctrl_row->addWidget(stop_btn_);
    ctrl_row->addWidget(preflight_btn_);
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
    if (!f.isEmpty()) { video_edit_->setText(f); saveSettings(); }
}

void LaunchTab::onPreflight() {
    QString proj = QFileInfo(QCoreApplication::applicationDirPath() + "/../..").absoluteFilePath();
    QString exe  = resolveExecutable();
    bool need_serial = !no_gimbal_->isChecked();
    bool use_video   = !video_edit_->text().isEmpty();

    auto chk = [&](bool ok, const QString& msg) {
        QString tag = ok ? "[✓]" : "[✗]";
        log_view_->appendPlainText(QString("%1 %2").arg(tag, msg));
        return ok;
    };

    log_view_->appendPlainText("\n========  预检  ========");
    bool all = true;
    all &= chk(QFileInfo::exists(exe),
               QString("二进制: %1").arg(exe));

    // 模型/配置
    QStringList must = {
        proj + "/config/cascade.yaml",
        proj + "/config/control.yaml",
        proj + "/config/camera.yaml",
    };
    for (const QString& p : must) all &= chk(QFileInfo::exists(p), QString("配置: %1").arg(p));

    // 串口
    if (need_serial) {
        QString port = port_edit_->text();
        all &= chk(QFileInfo::exists(port),
                   QString("串口: %1").arg(port));
    } else {
        log_view_->appendPlainText("[ ] 串口: 跳过 (--no-gimbal)");
    }

    // 相机 / 视频
    if (use_video) {
        all &= chk(QFileInfo::exists(video_edit_->text()),
                   QString("视频: %1").arg(video_edit_->text()));
    } else {
        // 带 GPU 接口的 GxIAPI MER 相机不走 /dev/video*; 这里只提示
        QDir vdir("/dev");
        QStringList vs = vdir.entryList(QStringList() << "video*", QDir::System);
        log_view_->appendPlainText(QString("[ ] /dev/video* 设备 %1 个%2")
                                   .arg(vs.size())
                                   .arg(vs.isEmpty() ? " (如用 USB 相机需检查)" : ""));
    }

    log_view_->appendPlainText(all ? "预检通过 ✓" : "预检未通过 ✗");
    if (!log_toggle_btn_->isChecked()) {
        log_toggle_btn_->setChecked(true);
        onToggleLog();
    }
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
    if (preflight_btn_) preflight_btn_->setEnabled(!running);
    mode_combo_->setEnabled(!running);
    color_combo_->setEnabled(!running);
    port_edit_->setEnabled(!running);
    video_edit_->setEnabled(!running);
    video_browse_->setEnabled(!running);
    no_gimbal_->setEnabled(!running);
    no_record_->setEnabled(!running);
}

void LaunchTab::loadSettings() {
    QSettings s("tracking", "control_panel");
    s.beginGroup("launch");
    int idx = mode_combo_->findData(s.value("mode", "tracking").toString());
    if (idx >= 0) mode_combo_->setCurrentIndex(idx);
    color_combo_->setCurrentText(s.value("color", "red").toString());
    port_edit_->setText(s.value("port", "/dev/ttyUSB0").toString());
    video_edit_->setText(s.value("video", "").toString());
    no_gimbal_->setChecked(s.value("no_gimbal", false).toBool());
    no_record_->setChecked(s.value("no_record", false).toBool());
    s.endGroup();

    // 以上 setText/setCurrent* 不会触发 saveSettings (loadSettings 期间信号已连, 但值与默认差异会触发
    // toggled/textChanged); 为避免反复写入 INI, 连接 saveSettings 仅在 buildUi 中完成
    // 连接 mode/color/port/video 变更 → 保存
    connect(mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) { saveSettings(); });
    connect(color_combo_, &QComboBox::currentTextChanged,
            this, [this](const QString&) { saveSettings(); });
    connect(port_edit_, &QLineEdit::editingFinished, this, [this] { saveSettings(); });
    connect(video_edit_, &QLineEdit::editingFinished, this, [this] { saveSettings(); });
}

void LaunchTab::saveSettings() const {
    QSettings s("tracking", "control_panel");
    s.beginGroup("launch");
    s.setValue("mode", mode_combo_->currentData().toString());
    s.setValue("color", color_combo_->currentText());
    s.setValue("port", port_edit_->text());
    s.setValue("video", video_edit_->text());
    s.setValue("no_gimbal", no_gimbal_->isChecked());
    s.setValue("no_record", no_record_->isChecked());
    s.endGroup();
}

}  // namespace tracking_app
