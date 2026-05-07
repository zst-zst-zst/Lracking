#include <QApplication>
#include <QDir>
#include <QPixmap>
#include <QTabWidget>
#include <QTimer>

#include "ControlPanel.h"

// 用法:
//   control_panel              正常 GUI
//   control_panel --shot DIR   离屏渲染, 把每个 tab 截图保存到 DIR/tab_N_<name>.png 然后退出
//                              (用于 CLI 环境快速预览)
int main(int argc, char** argv) {
    QString shot_dir;
    for (int i = 1; i < argc; ++i) {
        QString a = argv[i];
        if (a == "--shot" && i + 1 < argc) shot_dir = argv[++i];
    }

    QApplication app(argc, argv);
    app.setApplicationName("Tracking Control Panel");
    app.setOrganizationName("zst-tracking");

    tracking_app::ControlPanel window;
    window.resize(1200, 760);
    window.show();

    if (!shot_dir.isEmpty()) {
        QDir().mkpath(shot_dir);
        // 等控件 layout/paint 一遍再开始抓
        QTimer::singleShot(200, &app, [&] {
            auto* tabs = qobject_cast<QTabWidget*>(window.centralWidget());
            if (!tabs) { app.quit(); return; }
            for (int i = 0; i < tabs->count(); ++i) {
                tabs->setCurrentIndex(i);
                QApplication::processEvents();
                QApplication::processEvents();
                QPixmap pm = window.grab();
                QString name = QString("%1/tab_%2_%3.png")
                                   .arg(shot_dir)
                                   .arg(i)
                                   .arg(tabs->tabText(i));
                pm.save(name);
            }
            QTimer::singleShot(50, &app, &QApplication::quit);
        });
    }

    return app.exec();
}
