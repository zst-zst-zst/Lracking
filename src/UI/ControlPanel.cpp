#include "ControlPanel.h"
#include "LaunchTab.h"
#include "ConfigTab.h"
#include "StatusTab.h"
#include "PlotTab.h"
#include "RecordsTab.h"
#include "ToolsTab.h"

#include <QApplication>
#include <QCloseEvent>
#include <QLabel>
#include <QSettings>
#include <QStatusBar>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

namespace tracking_app {

ControlPanel::ControlPanel(QWidget* parent) : QMainWindow(parent) {
    buildUi();
    applyDarkTheme();
    setWindowTitle("Tracking 控制面板");
    restoreGeometryFromSettings();
}

void ControlPanel::closeEvent(QCloseEvent* e) {
    QSettings s("tracking", "control_panel");
    s.setValue("window/geometry", saveGeometry());
    s.setValue("window/state", saveState());
    s.setValue("window/tab_index", tabs_->currentIndex());
    QMainWindow::closeEvent(e);
}

void ControlPanel::restoreGeometryFromSettings() {
    QSettings s("tracking", "control_panel");
    QByteArray g = s.value("window/geometry").toByteArray();
    if (!g.isEmpty()) restoreGeometry(g);
    else              resize(1100, 700);
    QByteArray st = s.value("window/state").toByteArray();
    if (!st.isEmpty()) restoreState(st);
    int ti = s.value("window/tab_index", 0).toInt();
    if (ti >= 0 && ti < tabs_->count()) tabs_->setCurrentIndex(ti);
}

void ControlPanel::buildUi() {
    tabs_ = new QTabWidget(this);

    auto* launch = new LaunchTab;
    auto* status = new StatusTab;
    auto* plot   = new PlotTab;
    auto* records = new RecordsTab;
    QObject::connect(launch, &LaunchTab::procOutput, status, &StatusTab::onProcLine);
    QObject::connect(launch, &LaunchTab::procStopped, status, &StatusTab::onProcStopped);
    QObject::connect(launch, &LaunchTab::procOutput, plot, &PlotTab::onProcLine);
    QObject::connect(launch, &LaunchTab::procStopped, plot, &PlotTab::onProcStopped);
    // RecordsTab → LaunchTab: 双击/送出选中视频, 自动切到调试模式并切换到启动 tab
    QObject::connect(records, &RecordsTab::sendToLaunch, this,
                     [this, launch](const QString& path) {
                         launch->setVideoPath(path);
                         tabs_->setCurrentWidget(launch);
                     });

    tabs_->addTab(launch, "启动");
    tabs_->addTab(new ConfigTab, "配置");
    tabs_->addTab(status, "状态");
    tabs_->addTab(plot, "曲线");
    tabs_->addTab(records, "录像");
    tabs_->addTab(new ToolsTab, "工具");

    setCentralWidget(tabs_);

    status_label_ = new QLabel("Ready  |  Session 6 — Plot + Preflight + QSettings");
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
