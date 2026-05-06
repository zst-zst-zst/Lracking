#include "ConfigTab.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFile>
#include <QRegularExpression>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QVBoxLayout>

#include <opencv2/core.hpp>

namespace tracking_app {

// ── YamlForm ──────────────────────────────────────────────────────────

YamlForm::YamlForm(const QString& yaml_path, const QVector<FieldSpec>& spec, QWidget* parent)
    : QWidget(parent), yaml_path_(yaml_path), spec_(spec) {

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(20, 20, 20, 20);

    auto* path_lbl = new QLabel(QString("📄 %1").arg(yaml_path_));
    path_lbl->setStyleSheet("color: #888; font-size: 11px;");
    root->addWidget(path_lbl);

    auto* form_box = new QGroupBox("常用字段");
    auto* form = new QFormLayout(form_box);
    form->setHorizontalSpacing(14);
    form->setVerticalSpacing(8);

    widgets_.reserve(spec_.size());
    for (const auto& f : spec_) {
        QWidget* w = nullptr;
        switch (f.type) {
            case Int: {
                auto* sb = new QSpinBox;
                sb->setRange(-999999, 999999);
                w = sb;
                break;
            }
            case Double: {
                auto* sb = new QDoubleSpinBox;
                sb->setRange(-1e9, 1e9);
                sb->setDecimals(4);
                sb->setSingleStep(0.01);
                w = sb;
                break;
            }
            case Bool: {
                auto* cb = new QCheckBox;
                w = cb;
                break;
            }
            case String: {
                if (!f.options.isEmpty()) {
                    auto* combo = new QComboBox;
                    combo->addItems(f.options);
                    w = combo;
                } else {
                    w = new QLineEdit;
                }
                break;
            }
        }
        if (!f.tooltip.isEmpty()) w->setToolTip(f.tooltip);
        widgets_.append(w);
        form->addRow(f.label, w);
    }

    root->addWidget(form_box, 1);

    // ── 操作按钮 ──
    auto* btn_row = new QHBoxLayout;
    auto* reload = new QPushButton("⟳ 重新加载");
    auto* save = new QPushButton("💾 保存");
    save->setStyleSheet("QPushButton { font-weight: bold; background: #2d6e2d; }"
                        "QPushButton:hover { background: #3a8a3a; }");
    status_lbl_ = new QLabel(" ");
    status_lbl_->setStyleSheet("color: #888;");
    btn_row->addWidget(reload);
    btn_row->addWidget(save);
    btn_row->addSpacing(20);
    btn_row->addWidget(status_lbl_);
    btn_row->addStretch();
    root->addLayout(btn_row);

    connect(reload, &QPushButton::clicked, this, &YamlForm::load);
    connect(save, &QPushButton::clicked, this, &YamlForm::save);

    load();
}

void YamlForm::load() {
    cv::FileStorage fs(yaml_path_.toStdString(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
        status_lbl_->setText("⚠ 无法打开文件");
        status_lbl_->setStyleSheet("color: #d46e6e;");
        return;
    }
    for (int i = 0; i < spec_.size(); ++i) {
        const auto& f = spec_[i];
        cv::FileNode node = fs[f.key.toStdString()];
        if (node.empty()) continue;
        switch (f.type) {
            case Int: {
                int v = 0; node >> v;
                static_cast<QSpinBox*>(widgets_[i])->setValue(v);
                break;
            }
            case Double: {
                double v = 0; node >> v;
                static_cast<QDoubleSpinBox*>(widgets_[i])->setValue(v);
                break;
            }
            case Bool: {
                int v = 0; node >> v;
                static_cast<QCheckBox*>(widgets_[i])->setChecked(v != 0);
                break;
            }
            case String: {
                std::string v; node >> v;
                if (auto* combo = qobject_cast<QComboBox*>(widgets_[i])) {
                    int idx = combo->findText(QString::fromStdString(v));
                    if (idx >= 0) combo->setCurrentIndex(idx);
                    else combo->setEditText(QString::fromStdString(v));
                } else {
                    static_cast<QLineEdit*>(widgets_[i])->setText(QString::fromStdString(v));
                }
                break;
            }
        }
    }
    fs.release();
    status_lbl_->setText("✓ 已加载");
    status_lbl_->setStyleSheet("color: #888;");
}

void YamlForm::save() {
    // OpenCV FileStorage::WRITE 会清空文件, 这会丢失我们没暴露的字段!
    // 策略: 先 READ 出所有节点 → 写回 (覆盖暴露字段), 保留其他
    // 实现: 直接用 sed-like 文本替换最简单, 但无法保证 yaml 注释保留
    // 折中: 只写覆盖的字段, 其他不动 → 用文本替换保留注释

    // 方案 A: cv::FileStorage 写所有节点 → 注释丢失
    // 方案 B: 文本替换 → 注释保留, 但需要解析每行

    // 这里用方案 B (保留注释 + 简单 sed 风格替换)
    QFile f(yaml_path_);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "保存失败", "无法打开 " + yaml_path_);
        return;
    }
    QString content = QString::fromUtf8(f.readAll());
    f.close();

    auto qstr = [](QWidget* w, FieldType t) -> QString {
        switch (t) {
            case Int:    return QString::number(static_cast<QSpinBox*>(w)->value());
            case Double: return QString::number(static_cast<QDoubleSpinBox*>(w)->value(), 'g', 6);
            case Bool:   return static_cast<QCheckBox*>(w)->isChecked() ? "1" : "0";
            case String: {
                QString s = qobject_cast<QComboBox*>(w)
                                ? qobject_cast<QComboBox*>(w)->currentText()
                                : static_cast<QLineEdit*>(w)->text();
                return "\"" + s + "\"";
            }
        }
        return {};
    };

    int changed = 0;
    for (int i = 0; i < spec_.size(); ++i) {
        const auto& fld = spec_[i];
        QString new_val = qstr(widgets_[i], fld.type);
        // 匹配 "key: <value>"  (带可选注释)
        QRegularExpression re(QString("^(\\s*%1:\\s*)([^#\\n]+?)(\\s*(#.*)?)$").arg(fld.key),
                              QRegularExpression::MultilineOption);
        QString replaced = content.replace(re, "\\1" + new_val + "\\3");
        if (replaced != content) {
            content = replaced;
            ++changed;
        }
    }

    if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        QMessageBox::warning(this, "保存失败", "无法写入 " + yaml_path_);
        return;
    }
    f.write(content.toUtf8());
    f.close();

    status_lbl_->setText(QString("✓ 已保存 (%1 字段更新, 重启 tracking 生效)").arg(changed));
    status_lbl_->setStyleSheet("color: #6ed46e;");
}

// ── ConfigTab ─────────────────────────────────────────────────────────

ConfigTab::ConfigTab(QWidget* parent) : QWidget(parent) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);

    auto* sub_tabs = new QTabWidget;

    // ── cascade.yaml ──
    QVector<YamlForm::FieldSpec> cascade_spec = {
        {"layer1_conf",        "Layer1 置信度",      YamlForm::Double, "无人机检测阈值 (0~1)", {}},
        {"layer1_iou",         "Layer1 IoU",         YamlForm::Double, "NMS IoU 阈值", {}},
        {"layer2_conf",        "Layer2 默认置信度",  YamlForm::Double, "激光模块默认阈值", {}},
        {"layer2_conf_blue",   "Layer2 蓝色阈值",    YamlForm::Double, "蓝色 (常误检) 单独提高", {}},
        {"layer2_conf_purple", "Layer2 紫色阈值",    YamlForm::Double, "紫色阈值", {}},
        {"layer2_conf_red",    "Layer2 红色阈值",    YamlForm::Double, "红色阈值", {}},
        {"layer2_iou",         "Layer2 IoU",         YamlForm::Double, "Layer2 NMS IoU", {}},
        {"enemy_x_threshold",  "敌方 X 归一化阈值",  YamlForm::Double, "0~1, 区分敌我方位", {}},
        {"strip_reject_enabled", "灯带拒绝",         YamlForm::Bool,   "经典 CV 灯带误检拒绝", {}},
    };
    sub_tabs->addTab(new YamlForm("config/cascade.yaml", cascade_spec), "cascade");

    // ── control.yaml ──
    QVector<YamlForm::FieldSpec> control_spec = {
        {"kp",                 "Kp",                YamlForm::Double, "PID 比例增益", {}},
        {"deadband_px",        "死区 (px)",         YamlForm::Double, "误差小于此值不动", {}},
        {"max_angle_rate",     "Yaw 最大角速度",     YamlForm::Double, "deg/s", {}},
        {"max_angle_rate_pitch", "Pitch 最大角速度", YamlForm::Double, "deg/s", {}},
        {"lowpass_alpha",      "低通 alpha",        YamlForm::Double, "0~1, 越小越平滑", {}},
        {"yaw_sign",           "Yaw 方向",          YamlForm::Double, "-1 或 1", {}},
        {"pitch_sign",         "Pitch 方向",        YamlForm::Double, "-1 或 1", {}},
        {"use_velocity_ff",    "前馈开关",          YamlForm::Bool,   "速度前馈, 减少滞后", {}},
        {"ff_alpha",           "前馈 EMA",          YamlForm::Double, "0~1", {}},
        {"ff_rate_max",        "前馈最大速率",      YamlForm::Double, "deg/s", {}},
        {"use_damping",        "阻尼开关",          YamlForm::Bool,   "减振", {}},
        {"damping_source",     "阻尼来源",          YamlForm::String, "meas=测量速度 / gimbal=云台反馈", {"meas", "gimbal"}},
    };
    sub_tabs->addTab(new YamlForm("config/control.yaml", control_spec), "control");

    // ── camera.yaml ──
    QVector<YamlForm::FieldSpec> camera_spec = {
        {"auto_exposure",      "自动曝光",          YamlForm::Bool,   "", {}},
        {"exposure_time_us",   "曝光时间 (us)",     YamlForm::Double, "手动曝光: 越短越压拖影", {}},
        {"auto_gain",          "自动增益",          YamlForm::Bool,   "", {}},
        {"gain",               "增益",              YamlForm::Double, "auto_gain=0 时生效", {}},
        {"balance_white_auto", "自动白平衡",        YamlForm::Bool,   "", {}},
        {"roi_enable",         "启用 ROI 裁剪",     YamlForm::Bool,   "", {}},
        {"roi_offset_x",       "ROI 偏移 X",        YamlForm::Int,    "", {}},
        {"roi_offset_y",       "ROI 偏移 Y",        YamlForm::Int,    "", {}},
        {"roi_width",          "ROI 宽",            YamlForm::Int,    "", {}},
        {"roi_height",         "ROI 高",            YamlForm::Int,    "", {}},
    };
    sub_tabs->addTab(new YamlForm("config/camera.yaml", camera_spec), "camera");

    root->addWidget(sub_tabs);
}

}  // namespace tracking_app
