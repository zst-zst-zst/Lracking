#include "RecordsTab.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QVBoxLayout>

namespace tracking_app {

namespace {
constexpr const char* kVideoExts =
    "*.mp4 *.avi *.mov *.mkv *.MP4 *.AVI *.MOV *.MKV";

QString defaultRecordsDir() {
    // build/bin/control_panel → project root /records
    QString root = QFileInfo(QCoreApplication::applicationDirPath() + "/../..")
                       .absoluteFilePath();
    return root + "/records";
}
}  // namespace

RecordsTab::RecordsTab(QWidget* parent) : QWidget(parent) {
    QSettings s("tracking", "control_panel");
    records_dir_ = s.value("records/dir", defaultRecordsDir()).toString();
    buildUi();
    scanDirectory();
}

void RecordsTab::buildUi() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(20, 20, 20, 20);
    root->setSpacing(10);

    // ── 目录行 ──
    auto* dir_row = new QHBoxLayout;
    dir_row->addWidget(new QLabel("录像目录:"));
    dir_edit_ = new QLineEdit(records_dir_);
    dir_edit_->setReadOnly(true);
    dir_browse_ = new QPushButton("更换…");
    refresh_btn_ = new QPushButton("刷新");
    connect(dir_browse_, &QPushButton::clicked, this, &RecordsTab::onChangeDirClicked);
    connect(refresh_btn_, &QPushButton::clicked, this, &RecordsTab::onRefresh);
    dir_row->addWidget(dir_edit_, 1);
    dir_row->addWidget(dir_browse_);
    dir_row->addWidget(refresh_btn_);
    root->addLayout(dir_row);

    // ── 列表 ──
    file_list_ = new QListWidget;
    file_list_->setAlternatingRowColors(true);
    file_list_->setStyleSheet(
        "QListWidget { background: #141414; alternate-background-color: #1a1a1a; }"
        "QListWidget::item { padding: 6px 8px; }"
        "QListWidget::item:selected { background: #4a90d9; }");
    connect(file_list_, &QListWidget::itemActivated, this, &RecordsTab::onItemActivated);
    connect(file_list_, &QListWidget::itemSelectionChanged, this, &RecordsTab::onSelectionChanged);
    root->addWidget(file_list_, 1);

    // ── 操作行 ──
    auto* btn_row = new QHBoxLayout;
    info_lbl_ = new QLabel("(目录扫描中…)");
    info_lbl_->setStyleSheet("color: #888;");
    send_btn_ = new QPushButton("送至 启动 ▶");
    send_btn_->setStyleSheet("QPushButton { background: #2d6e2d; font-weight: bold; }"
                             "QPushButton:hover { background: #3a8a3a; }");
    delete_btn_ = new QPushButton("删除");
    delete_btn_->setStyleSheet("QPushButton { background: #6e2d2d; }"
                               "QPushButton:hover { background: #8a3a3a; }");
    connect(send_btn_, &QPushButton::clicked, this, &RecordsTab::onSendClicked);
    connect(delete_btn_, &QPushButton::clicked, this, &RecordsTab::onDeleteClicked);
    btn_row->addWidget(info_lbl_, 1);
    btn_row->addWidget(send_btn_);
    btn_row->addWidget(delete_btn_);
    root->addLayout(btn_row);

    send_btn_->setEnabled(false);
    delete_btn_->setEnabled(false);
}

QString RecordsTab::humanSize(qint64 bytes) const {
    constexpr double kKB = 1024.0;
    if (bytes < 1024) return QString("%1 B").arg(bytes);
    double kb = bytes / kKB;
    if (kb < 1024.0) return QString::number(kb, 'f', 1) + " KB";
    double mb = kb / kKB;
    if (mb < 1024.0) return QString::number(mb, 'f', 1) + " MB";
    return QString::number(mb / kKB, 'f', 2) + " GB";
}

void RecordsTab::scanDirectory() {
    file_list_->clear();
    QDir d(records_dir_);
    if (!d.exists()) {
        info_lbl_->setText(QString("⚠ 目录不存在: %1").arg(records_dir_));
        return;
    }
    QStringList filters = QString(kVideoExts).split(' ', Qt::SkipEmptyParts);
    QFileInfoList entries = d.entryInfoList(filters, QDir::Files, QDir::Time);

    qint64 total = 0;
    for (const QFileInfo& fi : entries) {
        QString label = QString("%1   |   %2   |   %3")
                            .arg(fi.fileName(), -38)
                            .arg(humanSize(fi.size()), 10)
                            .arg(fi.lastModified().toString("yyyy-MM-dd hh:mm"));
        auto* it = new QListWidgetItem(label, file_list_);
        it->setData(Qt::UserRole, fi.absoluteFilePath());
        total += fi.size();
    }
    info_lbl_->setText(QString("共 %1 个文件,  合计 %2")
                           .arg(entries.size())
                           .arg(humanSize(total)));
}

void RecordsTab::onRefresh() { scanDirectory(); }

void RecordsTab::onChangeDirClicked() {
    QString d = QFileDialog::getExistingDirectory(this, "选择录像目录", records_dir_);
    if (d.isEmpty()) return;
    records_dir_ = d;
    dir_edit_->setText(d);
    QSettings("tracking", "control_panel").setValue("records/dir", d);
    scanDirectory();
}

void RecordsTab::onSelectionChanged() {
    bool has = file_list_->currentItem() != nullptr;
    send_btn_->setEnabled(has);
    delete_btn_->setEnabled(has);
}

void RecordsTab::onItemActivated(QListWidgetItem* item) {
    if (!item) return;
    emit sendToLaunch(item->data(Qt::UserRole).toString());
}

void RecordsTab::onSendClicked() {
    auto* it = file_list_->currentItem();
    if (it) emit sendToLaunch(it->data(Qt::UserRole).toString());
}

void RecordsTab::onDeleteClicked() {
    auto* it = file_list_->currentItem();
    if (!it) return;
    QString path = it->data(Qt::UserRole).toString();
    auto reply = QMessageBox::question(this, "删除录像",
        QString("确认删除?\n%1").arg(path),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (reply != QMessageBox::Yes) return;
    if (QFile::remove(path)) {
        scanDirectory();
    } else {
        QMessageBox::warning(this, "删除失败", QString("无法删除:\n%1").arg(path));
    }
}

}  // namespace tracking_app
