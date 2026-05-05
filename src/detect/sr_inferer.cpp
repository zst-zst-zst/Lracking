#include "sr_inferer.h"

#include <filesystem>
#include <iostream>
#include <vector>

#include <opencv2/imgproc.hpp>

#include <onnxruntime_cxx_api.h>

namespace detect {

struct SrInferer::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "detect_sr"};
    Ort::SessionOptions opts;
    std::unique_ptr<Ort::Session> session;
    int scale = 2;
    bool loaded = false;

    std::vector<std::string> in_names_str;
    std::vector<std::string> out_names_str;
    std::vector<const char*> in_names;
    std::vector<const char*> out_names;
};

SrInferer::SrInferer() : impl_(std::make_unique<Impl>()) {}
SrInferer::~SrInferer() = default;

bool SrInferer::ready() const { return impl_->loaded; }
int SrInferer::scale() const { return impl_->scale; }

bool SrInferer::load(const std::string& onnx_path, int scale,
                     bool use_cuda, int device_id) {
    impl_->loaded = false;
    impl_->scale = scale;

    if (!std::filesystem::exists(onnx_path)) {
        std::cerr << "SrInferer: model not found: " << onnx_path << "\n";
        return false;
    }

    impl_->opts.SetIntraOpNumThreads(2);
    impl_->opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    if (use_cuda) {
        try {
            OrtCUDAProviderOptions cuda_opts{};
            cuda_opts.device_id = device_id;
            impl_->opts.AppendExecutionProvider_CUDA(cuda_opts);
        } catch (const Ort::Exception& e) {
            std::cerr << "SrInferer: CUDA EP failed, fall back to CPU: " << e.what() << "\n";
        }
    }

    try {
        impl_->session = std::make_unique<Ort::Session>(
            impl_->env, onnx_path.c_str(), impl_->opts);
    } catch (const Ort::Exception& e) {
        std::cerr << "SrInferer: session creation failed: " << e.what() << "\n";
        return false;
    }

    Ort::AllocatorWithDefaultOptions alloc;
    size_t n_in = impl_->session->GetInputCount();
    impl_->in_names_str.resize(n_in);
    impl_->in_names.resize(n_in);
    for (size_t i = 0; i < n_in; ++i) {
        auto name = impl_->session->GetInputNameAllocated(i, alloc);
        impl_->in_names_str[i] = name.get();
        impl_->in_names[i] = impl_->in_names_str[i].c_str();
    }
    size_t n_out = impl_->session->GetOutputCount();
    impl_->out_names_str.resize(n_out);
    impl_->out_names.resize(n_out);
    for (size_t i = 0; i < n_out; ++i) {
        auto name = impl_->session->GetOutputNameAllocated(i, alloc);
        impl_->out_names_str[i] = name.get();
        impl_->out_names[i] = impl_->out_names_str[i].c_str();
    }

    impl_->loaded = true;
    std::cout << "SrInferer: loaded " << onnx_path
              << " (scale=" << scale << ", cuda=" << (use_cuda ? "yes" : "no") << ")\n";
    return true;
}

cv::Mat SrInferer::upscale(const cv::Mat& bgr) {
    if (!impl_->loaded || bgr.empty()) return cv::Mat();

    const int H = bgr.rows;
    const int W = bgr.cols;

    // BGR uint8 → RGB float [0,1] CHW
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);

    std::vector<float> input(static_cast<size_t>(3 * H * W));
    // HWC → CHW
    std::vector<cv::Mat> channels(3);
    for (int c = 0; c < 3; ++c) {
        channels[c] = cv::Mat(H, W, CV_32FC1,
                              input.data() + static_cast<size_t>(c) * H * W);
    }
    cv::split(rgb, channels);

    std::array<int64_t, 4> in_shape{1, 3, H, W};
    auto mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto in_tensor = Ort::Value::CreateTensor<float>(
        mem_info, input.data(), input.size(),
        in_shape.data(), in_shape.size());

    std::vector<Ort::Value> outputs;
    try {
        outputs = impl_->session->Run(
            Ort::RunOptions{nullptr},
            impl_->in_names.data(), &in_tensor, 1,
            impl_->out_names.data(), impl_->out_names.size());
    } catch (const Ort::Exception& e) {
        std::cerr << "SrInferer: inference failed: " << e.what() << "\n";
        return cv::Mat();
    }

    if (outputs.empty()) return cv::Mat();

    auto info = outputs[0].GetTensorTypeAndShapeInfo();
    auto shape = info.GetShape();
    if (shape.size() != 4 || shape[0] != 1 || shape[1] != 3) {
        return cv::Mat();
    }
    const int oh = static_cast<int>(shape[2]);
    const int ow = static_cast<int>(shape[3]);
    const float* out_data = outputs[0].GetTensorData<float>();

    // CHW float [0,1] → HWC BGR uint8
    cv::Mat out(oh, ow, CV_32FC3);
    std::vector<cv::Mat> out_channels(3);
    for (int c = 0; c < 3; ++c) {
        out_channels[c] = cv::Mat(oh, ow, CV_32FC1,
                                  const_cast<float*>(out_data) +
                                      static_cast<size_t>(c) * oh * ow);
    }
    cv::merge(out_channels, out);
    out.convertTo(out, CV_8UC3, 255.0);
    cv::cvtColor(out, out, cv::COLOR_RGB2BGR);
    return out;
}

}  // namespace detect
