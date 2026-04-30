#include "onnx_detector.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <onnxruntime_cxx_api.h>

namespace detect {

// ── Letterbox: 保持比例缩放到正方形 ──
static cv::Mat letterbox(const cv::Mat& src, int target_size,
                         float& scale, float& pad_x, float& pad_y) {
    int sw = src.cols, sh = src.rows;
    scale = std::min(static_cast<float>(target_size) / sw,
                     static_cast<float>(target_size) / sh);
    int nw = static_cast<int>(sw * scale);
    int nh = static_cast<int>(sh * scale);
    pad_x = (target_size - nw) / 2.0f;
    pad_y = (target_size - nh) / 2.0f;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);

    cv::Mat out(target_size, target_size, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(out(cv::Rect(static_cast<int>(pad_x), static_cast<int>(pad_y), nw, nh)));
    return out;
}

// ── BGR HWC uint8 → RGB CHW float32 [0,1] ──
static std::vector<float> hwc2chw(const cv::Mat& img) {
    int h = img.rows, w = img.cols;
    int plane = h * w;
    std::vector<float> out(3 * plane);
    for (int y = 0; y < h; ++y) {
        const uint8_t* row = img.ptr<uint8_t>(y);
        for (int x = 0; x < w; ++x) {
            int i = y * w + x;
            out[0 * plane + i] = row[3 * x + 2] / 255.0f;  // R
            out[1 * plane + i] = row[3 * x + 1] / 255.0f;  // G
            out[2 * plane + i] = row[3 * x + 0] / 255.0f;  // B
        }
    }
    return out;
}

// ── 实现 ──

struct OnnxDetector::Impl {
    OnnxConfig cfg;
    bool loaded = false;

    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "detect_onnx"};
    Ort::SessionOptions session_opts;
    std::unique_ptr<Ort::Session> session;

    std::vector<std::string> input_names_str;
    std::vector<std::string> output_names_str;
    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
};

OnnxDetector::OnnxDetector() : impl_(std::make_unique<Impl>()) {}
OnnxDetector::~OnnxDetector() = default;

bool OnnxDetector::isLoaded() const { return impl_->loaded; }

bool loadOnnxConfig(const std::string& yaml_path, OnnxConfig* out) {
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    auto base = std::filesystem::path(yaml_path).parent_path();

    if (!fs["onnx_model"].empty()) {
        std::string p;
        fs["onnx_model"] >> p;
        out->model_path = std::filesystem::path(p).is_relative()
                              ? (base / p).string() : p;
    }
    if (!fs["input_size"].empty())      fs["input_size"] >> out->input_size;
    if (!fs["conf_threshold"].empty())  fs["conf_threshold"] >> out->conf_threshold;
    if (!fs["iou_threshold"].empty())   fs["iou_threshold"] >> out->iou_threshold;
    if (!fs["use_cuda"].empty())        fs["use_cuda"] >> out->use_cuda;
    if (!fs["device_id"].empty())       fs["device_id"] >> out->device_id;

    if (!fs["class_names"].empty()) {
        cv::FileNode cn = fs["class_names"];
        out->class_names.clear();
        for (const auto& n : cn) {
            std::string s;
            n >> s;
            out->class_names.push_back(s);
        }
    }
    return true;
}

bool OnnxDetector::loadConfig(const std::string& yaml_path) {
    OnnxConfig cfg;
    if (!loadOnnxConfig(yaml_path, &cfg)) return false;
    return loadModel(cfg);
}

bool OnnxDetector::loadModel(const OnnxConfig& cfg) {
    impl_->cfg = cfg;
    impl_->loaded = false;

    if (!std::filesystem::exists(cfg.model_path)) {
        std::cerr << "OnnxDetector: model not found: " << cfg.model_path << "\n";
        return false;
    }

    impl_->session_opts.SetIntraOpNumThreads(4);
    impl_->session_opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    if (cfg.use_cuda) {
        try {
            OrtCUDAProviderOptions cuda_opts{};
            cuda_opts.device_id = cfg.device_id;
            impl_->session_opts.AppendExecutionProvider_CUDA(cuda_opts);
            std::cout << "OnnxDetector: CUDA EP enabled (device " << cfg.device_id << ")\n";
        } catch (const Ort::Exception& e) {
            std::cerr << "OnnxDetector: CUDA EP failed, falling back to CPU: " << e.what() << "\n";
        }
    }

    try {
        impl_->session = std::make_unique<Ort::Session>(
            impl_->env, cfg.model_path.c_str(), impl_->session_opts);
    } catch (const Ort::Exception& e) {
        std::cerr << "OnnxDetector: session creation failed: " << e.what() << "\n";
        return false;
    }

    Ort::AllocatorWithDefaultOptions alloc;

    // Input names
    size_t num_inputs = impl_->session->GetInputCount();
    impl_->input_names_str.resize(num_inputs);
    impl_->input_names.resize(num_inputs);
    for (size_t i = 0; i < num_inputs; ++i) {
        auto name = impl_->session->GetInputNameAllocated(i, alloc);
        impl_->input_names_str[i] = name.get();
        impl_->input_names[i] = impl_->input_names_str[i].c_str();
    }

    // Output names
    size_t num_outputs = impl_->session->GetOutputCount();
    impl_->output_names_str.resize(num_outputs);
    impl_->output_names.resize(num_outputs);
    for (size_t i = 0; i < num_outputs; ++i) {
        auto name = impl_->session->GetOutputNameAllocated(i, alloc);
        impl_->output_names_str[i] = name.get();
        impl_->output_names[i] = impl_->output_names_str[i].c_str();
    }

    impl_->loaded = true;
    std::cout << "OnnxDetector: loaded " << cfg.model_path
              << " (input=" << cfg.input_size << ", conf=" << cfg.conf_threshold
              << ", cuda=" << (cfg.use_cuda ? "yes" : "no") << ")\n";
    return true;
}

std::vector<Detection> OnnxDetector::detect(const cv::Mat& bgr, int64_t /*timestamp*/) {
    std::vector<Detection> out;
    if (!impl_->loaded || bgr.empty()) return out;

    const auto& cfg = impl_->cfg;

    // ── Preprocess: letterbox + HWC→CHW ──
    float scale, pad_x, pad_y;
    cv::Mat lb = letterbox(bgr, cfg.input_size, scale, pad_x, pad_y);
    std::vector<float> input_data = hwc2chw(lb);

    // ── Run inference ──
    std::array<int64_t, 4> input_shape = {1, 3, cfg.input_size, cfg.input_size};
    Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        mem_info, input_data.data(), input_data.size(),
        input_shape.data(), input_shape.size());

    auto output_tensors = impl_->session->Run(
        Ort::RunOptions{nullptr},
        impl_->input_names.data(), &input_tensor, 1,
        impl_->output_names.data(), impl_->output_names.size());

    // ── Parse YOLO11 transposed output [1, 4+nc, num_anchors] ──
    auto& output_tensor = output_tensors.front();
    auto shape = output_tensor.GetTensorTypeAndShapeInfo().GetShape();

    // shape: [1, 4+nc, 8400] (transposed) or [1, 8400, 4+nc] (regular)
    const float* data = output_tensor.GetTensorData<float>();

    int rows, cols;
    bool transposed = false;
    if (shape.size() == 3) {
        if (shape[1] < shape[2]) {
            // Transposed: [1, 4+nc, 8400]
            rows = static_cast<int>(shape[1]);
            cols = static_cast<int>(shape[2]);
            transposed = true;
        } else {
            // Regular: [1, 8400, 4+nc]
            rows = static_cast<int>(shape[1]);
            cols = static_cast<int>(shape[2]);
        }
    } else {
        std::cerr << "OnnxDetector: unexpected output shape\n";
        return out;
    }

    int nc = transposed ? (rows - 4) : (cols - 4);
    int num_anchors = transposed ? cols : rows;

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;

    for (int i = 0; i < num_anchors; ++i) {
        float cx, cy, w, h;
        if (transposed) {
            cx = data[0 * cols + i];
            cy = data[1 * cols + i];
            w  = data[2 * cols + i];
            h  = data[3 * cols + i];
        } else {
            cx = data[i * cols + 0];
            cy = data[i * cols + 1];
            w  = data[i * cols + 2];
            h  = data[i * cols + 3];
        }

        // Find best class
        float max_score = 0;
        int max_cls = 0;
        for (int c = 0; c < nc; ++c) {
            float s = transposed ? data[(4 + c) * cols + i]
                                 : data[i * cols + 4 + c];
            if (s > max_score) {
                max_score = s;
                max_cls = c;
            }
        }

        if (max_score < cfg.conf_threshold) continue;

        // Undo letterbox → original image coordinates
        float x1 = (cx - w / 2.0f - pad_x) / scale;
        float y1 = (cy - h / 2.0f - pad_y) / scale;
        float bw = w / scale;
        float bh = h / scale;

        // Clamp to image bounds
        x1 = std::max(0.0f, std::min(x1, static_cast<float>(bgr.cols)));
        y1 = std::max(0.0f, std::min(y1, static_cast<float>(bgr.rows)));
        bw = std::min(bw, static_cast<float>(bgr.cols) - x1);
        bh = std::min(bh, static_cast<float>(bgr.rows) - y1);

        boxes.emplace_back(static_cast<int>(x1), static_cast<int>(y1),
                           static_cast<int>(bw), static_cast<int>(bh));
        scores.push_back(max_score);
        class_ids.push_back(max_cls);
    }

    // ── NMS ──
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, scores, cfg.conf_threshold, cfg.iou_threshold, indices);

    for (int idx : indices) {
        Detection det;
        det.valid = true;
        det.class_id = class_ids[idx];
        det.label = (class_ids[idx] < static_cast<int>(cfg.class_names.size()))
                        ? cfg.class_names[class_ids[idx]]
                        : std::to_string(class_ids[idx]);
        det.bbox = boxes[idx];
        det.center = cv::Point2f(boxes[idx].x + boxes[idx].width / 2.0f,
                                 boxes[idx].y + boxes[idx].height / 2.0f);
        det.radius = std::max(boxes[idx].width, boxes[idx].height) / 2.0f;
        det.confidence = scores[idx];
        out.push_back(det);
    }

    // Sort by confidence descending
    std::sort(out.begin(), out.end(),
              [](const Detection& a, const Detection& b) {
                  return a.confidence > b.confidence;
              });

    return out;
}

}  // namespace detect
