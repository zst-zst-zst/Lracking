#ifndef CONTROL_PANEL_CONFIG_TAB_H
#define CONTROL_PANEL_CONFIG_TAB_H

#include <QString>
#include <QStringList>
#include <QVariant>
#include <QVector>
#include <QWidget>

class QFormLayout;
class QPushButton;
class QLabel;

namespace tracking_app {

// 单个 yaml 文件的可视化编辑面板
// 使用 cv::FileStorage (OpenCV YAML, 兼容 %YAML:1.0)
class YamlForm : public QWidget {
    Q_OBJECT

public:
    enum FieldType { Int, Double, String, Bool };

    struct FieldSpec {
        QString key;        // yaml 顶级 key
        QString label;      // 表单显示
        FieldType type;
        QString tooltip;    // 字段说明
        QStringList options;  // String 类型: 可选值 (空=自由输入)
    };

    YamlForm(const QString& yaml_path, const QVector<FieldSpec>& spec, QWidget* parent = nullptr);

private slots:
    void load();
    void save();

private:
    QString yaml_path_;
    QVector<FieldSpec> spec_;
    QVector<QWidget*> widgets_;  // 与 spec_ 一一对应
    QLabel* status_lbl_ = nullptr;
};

// 三个 sub-tab: cascade / control / camera
class ConfigTab : public QWidget {
    Q_OBJECT

public:
    explicit ConfigTab(QWidget* parent = nullptr);
};

}  // namespace tracking_app

#endif  // CONTROL_PANEL_CONFIG_TAB_H
