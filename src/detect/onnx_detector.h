#ifndef DETECT_ONNX_DETECTOR_H
#define DETECT_ONNX_DETECTOR_H

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "detector.h"

namespace detect {

// ── ONNX YOLO 推理配置 ──
struct OnnxConfig {
    std::string model_path;                 // .onnx 模型路径
    int input_size = 640;                   // YOLO 输入分辨率 (正方形)
    float conf_threshold = 0.50f;           // 置信度阈值
    float iou_threshold = 0.45f;            // NMS IoU 阈值
    bool use_cuda = false;                  // 是否使用 CUDA ExecutionProvider
    int device_id = 0;                      // GPU 设备 ID
    std::vector<std::string> class_names;   // 类别名列表 (可选)
};

bool loadOnnxConfig(const std::string& yaml_path, OnnxConfig* out);

// ── ONNX YOLO 检测器 ──
// 不依赖 TensorRT, 适合开发机 / 笔记本调试
// 支持 YOLO11 transposed 输出格式 [1, 4+nc, 8400]
class OnnxDetector : public Detector {
public:
    OnnxDetector();
    ~OnnxDetector() override;

    bool loadConfig(const std::string& yaml_path);
    bool loadModel(const OnnxConfig& cfg);

    std::vector<Detection> detect(const cv::Mat& bgr,
                                  int64_t timestamp) override;

    bool isLoaded() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace detect

#endif  // DETECT_ONNX_DETECTOR_H
