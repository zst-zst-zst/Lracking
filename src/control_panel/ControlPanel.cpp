#include "ControlPanel.h"
#include "LaunchTab.h"

#include <QApplication>
#include <QLabel>
#include <QStatusBar>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

namespace tracking_app {

namespace {

// 占位 tab: 正中显示 "Coming in session N" 文字
QWidget* makePlaceholderTab(const QString& title, const QString& session_note) {
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);
    layout->setAlignment(Qt::AlignCenter);

    auto* big = new QLabel(title);
    QFont big_font = big->font();
    big_font.setPointSize(24);
    big_font.setBold(true);
    big->setFont(big_font);
    big->setAlignment(Qt::AlignCenter);
    big->setStyleSheet("color: #888;");

    auto* note = new QLabel(session_note);
    note->setAlignment(Qt::AlignCenter);
    note->setStyleSheet("color: #555; font-size: 14px;");

    layout->addWidget(big);
    layout->addWidget(note);
    return page;
}

}  // namespace

ControlPanel::ControlPanel(QWidget* parent) : QMainWindow(parent) {
    buildUi();
    applyDarkTheme();
    resize(1100, 700);
    setWindowTitle("Tracking 控制面板");
}

void ControlPanel::buildUi() {
    tabs_ = new QTabWidget(this);

    tabs_->addTab(new LaunchTab, "启动");
    tabs_->addTab(makePlaceholderTab("Config",
                                     "编辑 cascade.yaml / control.yaml / camera.yaml\n(Session 3 实现)"),
                  "配置");
    tabs_->addTab(makePlaceholderTab("Status",
                                     "实时 fps / 跟踪 ID / 命中率 / 当前敌方颜色\n(Session 4 实现)"),
                  "状态");
    tabs_->addTab(makePlaceholderTab("Records",
                                     "浏览 records/ 录像, 双击送回放\n(Session 5 实现)"),
                  "录像");
    tabs_->addTab(makePlaceholderTab("Tools",
                                     "训练 / 导出 ONNX&TRT / 部署 / 标定 一键调用\n(Session 5 实现)"),
                  "工具");

    setCentralWidget(tabs_);

    status_label_ = new QLabel("Ready  |  Session 1 骨架");
    statusBar()->addPermanentWidget(status_label_);
}

void ControlPanel::applyDarkTheme() {
    qApp->setStyleSheet(R"(
        QMainWindow, QWidget { background: #1e1e1e; color: #e6e6e6; }
        QTabWidget::pane { border: 1px solid #3c3c3c; }
        QTabBar::tab {
            background: #2a2a2a; color: #c8c8c8;
            padding: 8px 18px; border: 1px solid #3c3c3c;
            border-bottom: none;
            border-top-left-radius: 4px; border-top-right-radius: 4px;
        }
        QTabBar::tab:selected { background: #3a3a3a; color: #ffffff; }
        QTabBar::tab:hover:!selected { background: #333333; }
        QStatusBar { background: #2a2a2a; color: #aaaaaa; }
        QLabel { color: #e6e6e6; }
        QPushButton {
            background: #3a3a3a; color: #ffffff;
            border: 1px solid #555555; padding: 6px 14px;
            border-radius: 3px;
        }
        QPushButton:hover { background: #4a4a4a; }
        QPushButton:pressed { background: #2a2a2a; }
        QPushButton:disabled { color: #666; background: #2a2a2a; }
        QLineEdit, QComboBox, QTextEdit, QPlainTextEdit {
            background: #2a2a2a; color: #e6e6e6;
            border: 1px solid #555; padding: 4px;
            selection-background-color: #4a90d9;
        }
        QGroupBox {
            border: 1px solid #3c3c3c; border-radius: 4px;
            margin-top: 10px; padding-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin; subcontrol-position: top left;
            padding: 0 6px; color: #aaaaaa;
        }
    )");
}

}  // namespace tracking_app
