#include "cascade_detector.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "api.h"

#ifdef DETECT_WITH_ONNX
#include "sr_inferer.h"
#endif

namespace detect {

// ─── ROI 计算 ─────────────────────────────────────────────────────────

cv::Rect computeRoi(const cv::Rect& plane_bbox, int img_w, int img_h,
                     float top_extend, float bottom_extend,
                     float width_scale, int min_size) {
    float cx = plane_bbox.x + plane_bbox.width / 2.0f;
    float bh = static_cast<float>(plane_bbox.height);
    float bw = static_cast<float>(plane_bbox.width);

    float roi_w = std::max(bw * width_scale, static_cast<float>(min_size));
    float roi_top = plane_bbox.y - top_extend * bh;
    float roi_bottom = plane_bbox.y + bottom_extend * bh;
    float roi_h = roi_bottom - roi_top;
    if (roi_h < min_size) {
        float mid = (roi_top + roi_bottom) / 2.0f;
        roi_top = mid - min_size / 2.0f;
        roi_bottom = mid + min_size / 2.0f;
    }

    int rx1 = std::max(0, static_cast<int>(cx - roi_w / 2.0f));
    int ry1 = std::max(0, static_cast<int>(roi_top));
    int rx2 = std::min(img_w, static_cast<int>(cx + roi_w / 2.0f));
    int ry2 = std::min(img_h, static_cast<int>(roi_bottom));

    if (rx2 - rx1 < min_size / 2 || ry2 - ry1 < min_size / 2) {
        return cv::Rect();
    }
    return cv::Rect(rx1, ry1, rx2 - rx1, ry2 - ry1);
}

// ─── 配置加载 ─────────────────────────────────────────────────────────

bool loadCascadeConfig(const std::string& path, CascadeConfig* out) {
    if (!out) {
        return false;
    }
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "级联检测器配置文件打开失败: " << path << "\n";
        return false;
    }

    const std::filesystem::path base = std::filesystem::path(path).parent_path();

    auto readPath = [&](const std::string& key) -> std::string {
        std::string val;
        if (!fs[key].empty()) {
            fs[key] >> val;
            std::filesystem::path p(val);
            if (p.is_relative()) {
                val = (base / p).string();
            }
        }
        return val;
    };

    out->layer1_engine = readPath("layer1_engine");
    if (!fs["layer1_input_size"].empty()) fs["layer1_input_size"] >> out->layer1_input_size;
    if (!fs["layer1_conf"].empty()) fs["layer1_conf"] >> out->layer1_conf;
    if (!fs["layer1_iou"].empty()) fs["layer1_iou"] >> out->layer1_iou;

    if (!fs["enemy_side"].empty()) fs["enemy_side"] >> out->enemy_side;
    if (!fs["enemy_x_threshold"].empty()) fs["enemy_x_threshold"] >> out->enemy_x_threshold;

    if (!fs["roi_top_extend"].empty()) fs["roi_top_extend"] >> out->roi_top_extend;
    if (!fs["roi_bottom_extend"].empty()) fs["roi_bottom_extend"] >> out->roi_bottom_extend;
    if (!fs["roi_width_scale"].empty()) fs["roi_width_scale"] >> out->roi_width_scale;
    if (!fs["roi_min_size"].empty()) fs["roi_min_size"] >> out->roi_min_size;

    out->layer2_engine = readPath("layer2_engine");
    if (!fs["layer2_input_size"].empty()) fs["layer2_input_size"] >> out->layer2_input_size;
    if (!fs["layer2_conf"].empty()) fs["layer2_conf"] >> out->layer2_conf;
    if (!fs["layer2_iou"].empty()) fs["layer2_iou"] >> out->layer2_iou;
    if (!fs["layer2_conf_blue"].empty())   fs["layer2_conf_blue"]   >> out->layer2_conf_blue;
    if (!fs["layer2_conf_purple"].empty()) fs["layer2_conf_purple"] >> out->layer2_conf_purple;
    if (!fs["layer2_conf_red"].empty())    fs["layer2_conf_red"]    >> out->layer2_conf_red;

    if (!fs["laser_rx_max_aspect_ratio"].empty()) fs["laser_rx_max_aspect_ratio"] >> out->laser_rx_max_aspect_ratio;
    if (!fs["laser_rx_min_area_ratio"].empty()) fs["laser_rx_min_area_ratio"] >> out->laser_rx_min_area_ratio;
    if (!fs["laser_rx_max_area_ratio"].empty()) fs["laser_rx_max_area_ratio"] >> out->laser_rx_max_area_ratio;

    // 经典 CV 灯带拒绝参数
    if (!fs["enemy_color"].empty()) fs["enemy_color"] >> out->enemy_color;
    if (!fs["strip_reject_enabled"].empty()) {
        int v = 0;
        fs["strip_reject_enabled"] >> v;
        out->strip_reject_enabled = (v != 0);
    }
    if (!fs["strip_color_ratio_threshold"].empty()) fs["strip_color_ratio_threshold"] >> out->strip_color_ratio_threshold;
    if (!fs["strip_min_saturation"].empty()) fs["strip_min_saturation"] >> out->strip_min_saturation;
    if (!fs["strip_min_value"].empty()) fs["strip_min_value"] >> out->strip_min_value;
    if (!fs["strip_contour_elongation"].empty()) fs["strip_contour_elongation"] >> out->strip_contour_elongation;

    // 超分 (SR) 参数
    if (!fs["sr_enable"].empty()) {
        int v = 0;
        fs["sr_enable"] >> v;
        out->sr_enable = (v != 0);
    }
    out->sr_onnx = readPath("sr_onnx");
    if (!fs["sr_scale"].empty()) fs["sr_scale"] >> out->sr_scale;
    if (!fs["sr_max_roi_size"].empty()) fs["sr_max_roi_size"] >> out->sr_max_roi_size;
    if (!fs["sr_sharpen_enable"].empty()) {
        int v = 0;
        fs["sr_sharpen_enable"] >> v;
        out->sr_sharpen_enable = (v != 0);
    }
    if (!fs["sr_sharpen_sigma"].empty()) fs["sr_sharpen_sigma"] >> out->sr_sharpen_sigma;
    if (!fs["sr_sharpen_amount"].empty()) fs["sr_sharpen_amount"] >> out->sr_sharpen_amount;

    // 相对偏移位置预测参数
    if (!fs["offset_predict_enabled"].empty()) {
        int v = 0;
        fs["offset_predict_enabled"] >> v;
        out->offset_predict_enabled = (v != 0);
    }
    if (!fs["offset_min_samples"].empty()) fs["offset_min_samples"] >> out->offset_min_samples;
    if (!fs["offset_ema_alpha"].empty()) fs["offset_ema_alpha"] >> out->offset_ema_alpha;
    if (!fs["offset_predict_max_frames"].empty()) fs["offset_predict_max_frames"] >> out->offset_predict_max_frames;

    if (out->layer1_engine.empty() || out->layer2_engine.empty()) {
        std::cerr << "级联配置缺少引擎路径\n";
        return false;
    }
    return true;
}

// ─── 经典 CV 灯带拒绝 ──────────────────────────────────────────────────
// 灯带和激光模块都发队伍色光 (红/蓝)，但形状不同:
//   激光模块: 紧凑光斑 (~72×50mm, φ42mm, 近圆形)
//   灯带: 细长线条 (>1500mm, 折线/V形/平行四边形等)
//   灯带距模块 ≥250mm (S109规则)
//
// 算法 (逐轮廓分析, 处理重叠情况):
//   1) HSV 颜色掩码 (敌方队伍色)
//   2) 找轮廓, 逐个分类:
//      - 紧凑: 圆形度 > 0.3, 宽高比 < 2.0, 面积超过最小阈值 (可能是激光模块)
//      - 细长: 宽高比 > 阈值 (可能是灯带片段)
//   3) 判定:
//      - 存在任何紧凑轮廓 → 保留 (激光模块可能在其中)
//      - 只有细长轮廓 + 颜色占比高 → 拒绝为灯带
//      - 模糊情况 → 保留 (安全优先)

// 构建敌方队伍色 HSV 颜色掩码
static cv::Mat buildColorMask(const cv::Mat& bgr, const CascadeConfig& cfg) {
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    if (cfg.enemy_color == "blue") {
        // 蓝方: H ∈ [90, 130] (OpenCV HSV的H范围是0-180)
        cv::inRange(hsv,
                    cv::Scalar(90, cfg.strip_min_saturation, cfg.strip_min_value),
                    cv::Scalar(130, 255, 255), mask);
    } else {
        // 红方: H ∈ [0, 15] ∪ [165, 180] (HSV中红色跨越0度)
        cv::Mat m1, m2;
        cv::inRange(hsv,
                    cv::Scalar(0, cfg.strip_min_saturation, cfg.strip_min_value),
                    cv::Scalar(15, 255, 255), m1);
        cv::inRange(hsv,
                    cv::Scalar(165, cfg.strip_min_saturation, cfg.strip_min_value),
                    cv::Scalar(180, 255, 255), m2);
        mask = m1 | m2;
    }
    return mask;
}

// 判断检测区域是否为灯带 (返回 true = 灯带, 应拒绝)
static bool isLightStrip(const cv::Mat& det_region, const CascadeConfig& cfg) {
    if (det_region.empty()) return false;

    cv::Mat color_mask = buildColorMask(det_region, cfg);

    // 检查敌方队伍色像素占比
    int total_pixels = det_region.rows * det_region.cols;
    int color_pixels = cv::countNonZero(color_mask);
    float color_ratio = static_cast<float>(color_pixels) / std::max(total_pixels, 1);

    if (color_ratio < cfg.strip_color_ratio_threshold) {
        return false;  // 队伍色像素不够 → 不是灯带
    }

    // 逐轮廓分析: 查找紧凑形状 vs 细长形状
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(color_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return false;
    }

    // 最小轮廓面积阈值 (过滤微小噪声), 至少占检测区域的 1%
    float min_contour_area = total_pixels * 0.01f;

    bool has_compact = false;
    bool has_elongated = false;
    int significant_contours = 0;

    for (const auto& contour : contours) {
        float area = static_cast<float>(cv::contourArea(contour));
        if (area < min_contour_area) continue;
        significant_contours++;

        if (contour.size() < 5) continue;

        // 圆形度: 4π × 面积 / 周长² (1.0 = 完美圆形)
        float perimeter = static_cast<float>(cv::arcLength(contour, true));
        float circularity = (perimeter > 0)
            ? (4.0f * static_cast<float>(CV_PI) * area / (perimeter * perimeter))
            : 0.0f;

        // 最小外接矩形宽高比
        cv::RotatedRect rr = cv::minAreaRect(contour);
        float rw = rr.size.width, rh = rr.size.height;
        float aspect = (rw > rh) ? (rw / std::max(rh, 1.0f))
                                 : (rh / std::max(rw, 1.0f));

        // 紧凑轮廓: 可能是激光模块的 φ42mm 检测区域
        // 圆形度 > 0.3 (模块近圆形/正方形, 不是细线)
        // 宽高比 < 2.0 (模块 72×50mm → 宽高比 ~1.4)
        if (circularity > 0.3f && aspect < 2.0f) {
            has_compact = true;
        }

        if (aspect > cfg.strip_contour_elongation) {
            has_elongated = true;
        }
    }

    // 判定逻辑:
    // 存在紧凑色块 → 激光模块可能在其中, 即使灯带也在 bbox 中, 仍应保留
    if (has_compact) {
        return false;  // 保留 — 紧凑光斑表明激光模块可能存在
    }

    // 只有细长色块 + 没有紧凑色块 → 灯带
    if (has_elongated && significant_contours > 0) {
        return true;  // 拒绝 — 只有细长的队伍色形状
    }

    // 没有显著轮廓或形状模糊 → 安全起见, 保留
    return false;
}

// ─── 级联检测器实现 ─────────────────────────────────────────────────

struct CascadeDetector::Impl {
    CascadeConfig cfg;
    TRTInferX::Api layer1_api;
    TRTInferX::Api layer2_api;
    TRTInferX::EngineConfig layer1_engine_cfg;
    TRTInferX::EngineConfig layer2_engine_cfg;
    TRTInferX::InferOptions layer1_opt;
    TRTInferX::InferOptions layer2_opt;
    bool loaded = false;

    // ── 相对偏移位置预测状态 ──
    // 归一化偏移 = (模块中心 - 无人机bbox中心) / 无人机bbox尺寸
    // 这个偏移在不同距离下保持一致 (透视缩放被归一化抵消)
    cv::Point2f offset_x_ema{0.0f, 0.0f};  // x偏移 EMA (dx/bw, dy/bh)
    int offset_sample_count = 0;             // 已积累的有效检测次数
    int frames_since_detection = 0;          // 距离上次有效检测的帧数

#ifdef DETECT_WITH_ONNX
    std::unique_ptr<SrInferer> sr;            // 可选的超分推理器
#endif
};

CascadeDetector::CascadeDetector() : impl_(std::make_unique<Impl>()) {}
CascadeDetector::~CascadeDetector() = default;

bool CascadeDetector::loadConfig(const std::string& path) {
    if (!loadCascadeConfig(path, &impl_->cfg)) {
        return false;
    }
    auto& cfg = impl_->cfg;

    // 设置第1层引擎 (无人机检测)
    auto& l1_cfg = impl_->layer1_engine_cfg;
    l1_cfg.engine_path = cfg.layer1_engine;
    l1_cfg.target_w = cfg.layer1_input_size;
    l1_cfg.target_h = cfg.layer1_input_size;
    l1_cfg.max_batch = 1;
    l1_cfg.num_classes = 1;
    l1_cfg.prep = TRTInferX::PreprocessMode::LETTERBOX;
    l1_cfg.out_mode = TRTInferX::OutputMode::RAW_ONLY;
    impl_->layer1_opt.conf = cfg.layer1_conf;
    impl_->layer1_opt.iou = cfg.layer1_iou;

    if (!impl_->layer1_api.load(l1_cfg)) {
        std::cerr << "第1层引擎加载失败: " << cfg.layer1_engine << "\n";
        return false;
    }
    std::cout << "第1层 (无人机) 引擎已加载: " << cfg.layer1_engine << "\n";

    // 设置第2层引擎 (激光模块检测)
    auto& l2_cfg = impl_->layer2_engine_cfg;
    l2_cfg.engine_path = cfg.layer2_engine;
    l2_cfg.target_w = cfg.layer2_input_size;
    l2_cfg.target_h = cfg.layer2_input_size;
    l2_cfg.max_batch = 12;  // 最多 12 架无人机
    // 模型类别: blue(0), purple(1), red(2) — 均为激光检测模块颜色变体
    // 经典 CV 灯带拒绝无论模型类数都生效。
    l2_cfg.num_classes = 3;
    l2_cfg.prep = TRTInferX::PreprocessMode::LETTERBOX;
    l2_cfg.out_mode = TRTInferX::OutputMode::RAW_ONLY;
    // TRT 侧用最低的类阈值过滤 (保留低分检测给后续按类过滤)
    float trt_min_conf = cfg.layer2_conf;
    if (cfg.layer2_conf_blue   >= 0.f) trt_min_conf = std::min(trt_min_conf, cfg.layer2_conf_blue);
    if (cfg.layer2_conf_purple >= 0.f) trt_min_conf = std::min(trt_min_conf, cfg.layer2_conf_purple);
    if (cfg.layer2_conf_red    >= 0.f) trt_min_conf = std::min(trt_min_conf, cfg.layer2_conf_red);
    impl_->layer2_opt.conf = trt_min_conf;
    impl_->layer2_opt.iou = cfg.layer2_iou;

    if (!impl_->layer2_api.load(l2_cfg)) {
        std::cerr << "第2层引擎加载失败: " << cfg.layer2_engine << "\n";
        return false;
    }
    std::cout << "第2层 (激光模块) 引擎已加载: " << cfg.layer2_engine << "\n";

    // 可选: SR 推理器
    if (cfg.sr_enable) {
#ifdef DETECT_WITH_ONNX
        if (cfg.sr_onnx.empty()) {
            std::cerr << "sr_enable=1 但 sr_onnx 路径为空, SR 关闭\n";
        } else {
            impl_->sr = std::make_unique<SrInferer>();
            if (!impl_->sr->load(cfg.sr_onnx, cfg.sr_scale)) {
                std::cerr << "SR 模型加载失败, SR 关闭: " << cfg.sr_onnx << "\n";
                impl_->sr.reset();
            } else {
                std::cout << "SR 已启用: " << cfg.sr_onnx
                          << " (scale=" << cfg.sr_scale
                          << ", max_roi=" << cfg.sr_max_roi_size << ")\n";
            }
        }
#else
        std::cerr << "sr_enable=1 但未编译 DETECT_WITH_ONNX, SR 关闭\n";
#endif
    }

    impl_->loaded = true;
    return true;
}

CascadeResult CascadeDetector::detectCascade(const cv::Mat& bgr, int64_t /*timestamp*/) {
    CascadeResult result;
    if (!impl_->loaded || bgr.empty()) {
        return result;
    }

    const int img_w = bgr.cols;
    const int img_h = bgr.rows;
    auto& cfg = impl_->cfg;

    // ── 第1层: 检测无人机 ──
    TRTInferX::ImageInput l1_input{};
    l1_input.mem = TRTInferX::MemoryType::CPU;
    l1_input.data = bgr.data;
    l1_input.width = bgr.cols;
    l1_input.height = bgr.rows;
    l1_input.stride_bytes = static_cast<int>(bgr.step);
    l1_input.color = TRTInferX::ColorSpace::BGR;
    l1_input.layout = TRTInferX::Layout::HWC;
    l1_input.dtype = TRTInferX::DType::UINT8;
    l1_input.prep = impl_->layer1_engine_cfg.prep;
    l1_input.target_w = impl_->layer1_engine_cfg.target_w;
    l1_input.target_h = impl_->layer1_engine_cfg.target_h;

    std::vector<TRTInferX::ImageInput> l1_batch{l1_input};
    auto l1_dets = impl_->layer1_api.infer(l1_batch, impl_->layer1_opt);
    if (l1_dets.empty() || l1_dets.front().empty()) {
        return result;
    }

    // 收集无人机检测结果, 过滤敌/友方
    for (const auto& det : l1_dets.front()) {
        PlaneDetection pd;
        pd.bbox = cv::Rect(cv::Point2f(det.x1, det.y1),
                            cv::Point2f(det.x2, det.y2));
        pd.center = cv::Point2f((det.x1 + det.x2) * 0.5f,
                                (det.y1 + det.y2) * 0.5f);
        pd.confidence = det.score;

        // 敌/友方过滤: 只处理敌方侧的无人机
        float norm_cx = pd.center.x / static_cast<float>(img_w);
        bool is_enemy = (cfg.enemy_side == "right")
                            ? (norm_cx > cfg.enemy_x_threshold)
                            : (norm_cx < cfg.enemy_x_threshold);
        if (!is_enemy) {
            continue;  // 跳过友方无人机
        }

        result.planes.push_back(pd);
    }

    // ── ROI裁剪 + 第2层: 检测激光模块 ──
    // crops 是喂给第2层的图像 (可能经 SR 上采样).
    // sr_scales 记录每个 crop 相对于原 ROI 的缩放系数, 用于映射坐标回全帧.
    std::vector<cv::Rect> rois;
    std::vector<cv::Mat> crops;
    std::vector<float> sr_scales;
    std::vector<int> plane_indices;

#ifdef DETECT_WITH_ONNX
    const bool sr_active = (impl_->sr && impl_->sr->ready());
#endif

    for (int i = 0; i < static_cast<int>(result.planes.size()); ++i) {
        cv::Rect roi = computeRoi(
            result.planes[i].bbox, img_w, img_h,
            cfg.roi_top_extend, cfg.roi_bottom_extend,
            cfg.roi_width_scale, cfg.roi_min_size);
        if (roi.empty()) {
            continue;
        }
        cv::Mat crop = bgr(roi);
        if (crop.empty()) {
            continue;
        }
        float scale = 1.0f;
#ifdef DETECT_WITH_ONNX
        if (sr_active && std::min(crop.cols, crop.rows) < cfg.sr_max_roi_size) {
            cv::Mat sr_crop = impl_->sr->upscale(crop);
            if (!sr_crop.empty()) {
                scale = static_cast<float>(impl_->sr->scale());
                crop = sr_crop;
                // T 项目同款: SR 后 unsharp mask 锐化, 提升边缘让 YOLO 卷积响应更强.
                // 几乎零成本, 默认开启.
                if (cfg.sr_sharpen_enable && cfg.sr_sharpen_amount > 0.0f) {
                    cv::Mat blurred;
                    cv::GaussianBlur(crop, blurred, cv::Size(0, 0),
                                     cfg.sr_sharpen_sigma);
                    cv::addWeighted(crop, 1.0f + cfg.sr_sharpen_amount,
                                    blurred, -cfg.sr_sharpen_amount,
                                    0.0, crop);
                }
            }
        }
#endif
        rois.push_back(roi);
        crops.push_back(crop);
        sr_scales.push_back(scale);
        plane_indices.push_back(i);
    }

    if (crops.empty()) {
        return result;
    }

    // 批量推理所有 ROI
    std::vector<TRTInferX::ImageInput> l2_batch;
    l2_batch.reserve(crops.size());
    for (const auto& crop : crops) {
        TRTInferX::ImageInput inp{};
        inp.mem = TRTInferX::MemoryType::CPU;
        inp.data = crop.data;
        inp.width = crop.cols;
        inp.height = crop.rows;
        inp.stride_bytes = static_cast<int>(crop.step);
        inp.color = TRTInferX::ColorSpace::BGR;
        inp.layout = TRTInferX::Layout::HWC;
        inp.dtype = TRTInferX::DType::UINT8;
        inp.prep = impl_->layer2_engine_cfg.prep;
        inp.target_w = impl_->layer2_engine_cfg.target_w;
        inp.target_h = impl_->layer2_engine_cfg.target_h;
        l2_batch.push_back(inp);
    }

    auto l2_dets = impl_->layer2_api.infer(l2_batch, impl_->layer2_opt);

    for (size_t batch_idx = 0; batch_idx < l2_dets.size(); ++batch_idx) {
        const auto& dets = l2_dets[batch_idx];
        const auto& roi = rois[batch_idx];
        const auto& crop = crops[batch_idx];
        const float sr_s = sr_scales[batch_idx];
        int pi = plane_indices[batch_idx];
        float roi_area = static_cast<float>(roi.width * roi.height);

        for (const auto& det : dets) {
            // 3 类模型: blue(0), purple(1), red(2) 均为激光模块, 全部接受
            // ── 按类置信度阈值 ──
            float cls_thr = cfg.layer2_conf;
            if      (det.cls == 0 && cfg.layer2_conf_blue   >= 0.f) cls_thr = cfg.layer2_conf_blue;
            else if (det.cls == 1 && cfg.layer2_conf_purple >= 0.f) cls_thr = cfg.layer2_conf_purple;
            else if (det.cls == 2 && cfg.layer2_conf_red    >= 0.f) cls_thr = cfg.layer2_conf_red;
            if (det.score < cls_thr) continue;

            // det 坐标处于 (经 SR 后的) crop 空间
            float det_w = (det.x2 - det.x1) / sr_s;
            float det_h = (det.y2 - det.y1) / sr_s;

            // ── 宽高比过滤 ──
            float aspect = (det_w > det_h)
                               ? (det_w / std::max(det_h, 1.0f))
                               : (det_h / std::max(det_w, 1.0f));
            if (aspect > cfg.laser_rx_max_aspect_ratio) {
                continue;
            }

            // ── 面积比过滤 ──
            float det_area = det_w * det_h;
            float area_ratio = det_area / std::max(roi_area, 1.0f);
            if (area_ratio < cfg.laser_rx_min_area_ratio ||
                area_ratio > cfg.laser_rx_max_area_ratio) {
                continue;
            }

            // ── 经典 CV 灯带拒绝 (HSV颜色 + 轮廓形状) ──
            if (cfg.strip_reject_enabled) {
                // 从裁剪图中提取检测区域 (坐标相对于 SR 后的 crop)
                int bx1 = std::max(0, static_cast<int>(det.x1));
                int by1 = std::max(0, static_cast<int>(det.y1));
                int bx2 = std::min(crop.cols, static_cast<int>(det.x2));
                int by2 = std::min(crop.rows, static_cast<int>(det.y2));
                if (bx2 - bx1 > 2 && by2 - by1 > 2) {
                    cv::Mat det_region = crop(cv::Rect(bx1, by1, bx2 - bx1, by2 - by1));
                    if (isLightStrip(det_region, cfg)) {
                        continue;  // 被判定为灯带, 拒绝
                    }
                }
            }

            LaserRxDetection lrx;
            // 从 SR 后的 crop 坐标先除以 sr_s 还原到原始 ROI 坐标, 再加 ROI 偏移
            lrx.bbox = cv::Rect(
                static_cast<int>(det.x1 / sr_s) + roi.x,
                static_cast<int>(det.y1 / sr_s) + roi.y,
                static_cast<int>(det_w),
                static_cast<int>(det_h));
            lrx.center = cv::Point2f(
                (det.x1 + det.x2) * 0.5f / sr_s + roi.x,
                (det.y1 + det.y2) * 0.5f / sr_s + roi.y);
            lrx.confidence = det.score;
            lrx.plane_index = pi;
            lrx.roi = roi;
            result.laser_rxs.push_back(lrx);
        }
    }

    // 找到最佳目标 (最高置信度的 laser_rx)
    if (!result.laser_rxs.empty()) {
        const auto* best = &result.laser_rxs.front();
        for (const auto& lrx : result.laser_rxs) {
            if (lrx.confidence > best->confidence) {
                best = &lrx;
            }
        }
        result.has_target = true;
        result.target_center = best->center;
        result.target_confidence = best->confidence;

        // ── 更新相对偏移估计 ──
        // 记录模块相对无人机bbox的归一化偏移
        // 偏移 = (模块中心 - 无人机bbox中心) / 无人机bbox尺寸
        if (cfg.offset_predict_enabled && best->plane_index >= 0) {
            const auto& plane = result.planes[best->plane_index];
            float bw = static_cast<float>(plane.bbox.width);
            float bh = static_cast<float>(plane.bbox.height);
            if (bw > 10 && bh > 10) {  // 无人机bbox足够大时才有意义
                float dx = (best->center.x - plane.center.x) / bw;
                float dy = (best->center.y - plane.center.y) / bh;
                float alpha = cfg.offset_ema_alpha;
                if (impl_->offset_sample_count == 0) {
                    // 第一次, 直接赋值
                    impl_->offset_x_ema = cv::Point2f(dx, dy);
                } else {
                    // EMA 平滑更新
                    impl_->offset_x_ema.x = alpha * dx + (1.0f - alpha) * impl_->offset_x_ema.x;
                    impl_->offset_x_ema.y = alpha * dy + (1.0f - alpha) * impl_->offset_x_ema.y;
                }
                impl_->offset_sample_count++;
                impl_->frames_since_detection = 0;
            }
        }
    } else {
        // 没有检测到激光模块, 尝试偏移预测
        impl_->frames_since_detection++;

        if (cfg.offset_predict_enabled &&
            impl_->offset_sample_count >= cfg.offset_min_samples &&
            impl_->frames_since_detection <= cfg.offset_predict_max_frames &&
            !result.planes.empty()) {
            // 用无人机bbox + 归一化偏移 推算模块位置
            // 选择置信度最高的无人机
            const auto* best_plane = &result.planes.front();
            for (const auto& p : result.planes) {
                if (p.confidence > best_plane->confidence) best_plane = &p;
            }
            float bw = static_cast<float>(best_plane->bbox.width);
            float bh = static_cast<float>(best_plane->bbox.height);
            float pred_x = best_plane->center.x + impl_->offset_x_ema.x * bw;
            float pred_y = best_plane->center.y + impl_->offset_x_ema.y * bh;

            result.has_predicted_target = true;
            result.predicted_center = cv::Point2f(pred_x, pred_y);
            // 预测置信度随时间衰减 (越久越不可靠)
            float decay = 1.0f - static_cast<float>(impl_->frames_since_detection) /
                                 static_cast<float>(cfg.offset_predict_max_frames);
            result.predicted_confidence = 0.3f * decay;  // 基础置信度 0.3, 随时间衰减
        }
    }

    return result;
}

std::vector<Detection> CascadeDetector::detect(
    const cv::Mat& bgr, int64_t timestamp) {
    std::vector<Detection> out;
    auto cascade = detectCascade(bgr, timestamp);

    for (const auto& lrx : cascade.laser_rxs) {
        Detection det;
        det.valid = true;
        det.class_id = 0;
        det.label = "laser_rx";
        det.center = lrx.center;
        det.radius = std::max(lrx.bbox.width, lrx.bbox.height) * 0.5f;
        det.bbox = lrx.bbox;
        det.confidence = lrx.confidence;
        out.push_back(det);
    }

    // 按置信度降序排列
    std::sort(out.begin(), out.end(),
              [](const Detection& a, const Detection& b) {
                  return a.confidence > b.confidence;
              });

    return out;
}

}  // namespace detect
