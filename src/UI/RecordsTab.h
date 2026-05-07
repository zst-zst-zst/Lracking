#ifndef CONTROL_PANEL_RECORDS_TAB_H
#define CONTROL_PANEL_RECORDS_TAB_H

#include <QWidget>

class QListWidget;
class QListWidgetItem;
class QLabel;
class QPushButton;
class QLineEdit;

namespace tracking_app {

// Records tab: 浏览 records/ 目录下的视频, 双击/选中后通过 sendToLaunch 信号送给 LaunchTab.
class RecordsTab : public QWidget {
    Q_OBJECT

public:
    explicit RecordsTab(QWidget* parent = nullptr);

signals:
    // RecordsTab → ControlPanel → LaunchTab::setVideoPath()
    void sendToLaunch(const QString& video_path);

private slots:
    void onRefresh();
    void onItemActivated(QListWidgetItem* item);
    void onSendClicked();
    void onDeleteClicked();
    void onChangeDirClicked();
    void onSelectionChanged();

private:
    void buildUi();
    void scanDirectory();
    QString humanSize(qint64 bytes) const;

    QString      records_dir_;
    QLineEdit*   dir_edit_      = nullptr;
    QPushButton* dir_browse_    = nullptr;
    QPushButton* refresh_btn_   = nullptr;
    QListWidget* file_list_     = nullptr;
    QLabel*      info_lbl_      = nullptr;
    QPushButton* send_btn_      = nullptr;
    QPushButton* delete_btn_    = nullptr;
};

}  // namespace tracking_app

#endif  // CONTROL_PANEL_RECORDS_TAB_H
