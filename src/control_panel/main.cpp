#include <QApplication>

#include "ControlPanel.h"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    app.setApplicationName("Tracking Control Panel");
    app.setOrganizationName("zst-tracking");

    tracking_app::ControlPanel window;
    window.show();
    return app.exec();
}
