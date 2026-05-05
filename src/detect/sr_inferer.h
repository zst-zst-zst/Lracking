#ifndef DETECT_SR_INFERER_H
#define DETECT_SR_INFERER_H

#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace detect {

// 轻量级 SR 推理器, 包装 ESPCN ONNX 模型.
// 通过 ONNX Runtime + CUDA 后端运行, 支持动态输入分辨率.
class SrInferer {
public:
    SrInferer();
    ~SrInferer();

    // 加载 ONNX 模型 (例如 espcn_x2.onnx).
    // scale: 上采样倍数 (与训练时一致, 例 2)
    // 失败返回 false.
    bool load(const std::string& onnx_path, int scale = 2,
              bool use_cuda = true, int device_id = 0);

    bool ready() const;

    // 输入 BGR uint8, 输出 BGR uint8 (尺寸为 scale 倍).
    // 失败时返回空 Mat.
    cv::Mat upscale(const cv::Mat& bgr);

    int scale() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace detect

#endif  // DETECT_SR_INFERER_H
