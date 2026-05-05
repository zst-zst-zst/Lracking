#ifndef DETECT_CASCADE_DETECTOR_H
#define DETECT_CASCADE_DETECTOR_H

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "detector.h"

namespace detect {

struct CascadeConfig {
    // ── 第1层: 无人机检测 ──
    std::string layer1_engine;          // TensorRT 引擎路径
    int layer1_input_size = 640;        // YOLO 输入分辨率
    float layer1_conf = 0.50f;          // 置信度阈值
    float layer1_iou = 0.45f;           // NMS IoU 阈值

    // ── 敌/友方过滤 ──
    // 画面右侧 = 敌方 (照射目标), 左侧 = 友方 (忽略)
    std::string enemy_side = "right";   // 敌方所在画面侧
    float enemy_x_threshold = 0.5f;     // 归一化x坐标阈值

    // ── ROI 裁剪 ──
    // 无人机可见机体高约 20-30cm, 激光模块在其上方约 20-30cm
    // → bbox 顶部向上延伸 ≈ 1.0倍 bbox 高度
    float roi_top_extend = 1.0f;        // bbox顶部向上延伸 (bbox高度的倍数)
    float roi_bottom_extend = 0.3f;     // bbox顶部向下延伸 (处理俯视角重叠)
    float roi_width_scale = 1.2f;       // 水平方向缩放比
    int roi_min_size = 64;              // ROI 最小边长 (像素)

    // ── 第2层: 激光检测模块检测 ──
    // class 0 = laser_rx (目标)
    // class 1 = light_strip (可选, 双类模型时自动丢弃)
    std::string layer2_engine;          // TensorRT 引擎路径
    int layer2_input_size = 192;        // 较小输入, ROI 本身就小
    float layer2_conf = 0.40f;          // 置信度阈值 (模块较小, 适当放低)
    float layer2_iou = 0.45f;           // NMS IoU 阈值
    // 按类阈值: <0 表示走 layer2_conf 兜底. 类序 blue(0) purple(1) red(2)
    // blue 有误检偏置, 建议 0.65; purple 最稳, 0.55; red 居中, 0.55
    float layer2_conf_blue = -1.0f;
    float layer2_conf_purple = -1.0f;
    float layer2_conf_red = -1.0f;

    // ── 后过滤: 宽高比 + 面积比 ──
    // 激光模块外形 72×50mm → 近正方形, 宽高比 ~1.0-1.5
    // 灯带: 细长条状, 宽高比 >> 2.0
    float laser_rx_max_aspect_ratio = 2.5f;   // 宽高比上限
    float laser_rx_min_area_ratio = 0.002f;   // bbox面积/ROI面积 下限
    float laser_rx_max_area_ratio = 0.5f;     // bbox面积/ROI面积 上限

    // ── 经典 CV 灯带拒绝 (不需要标注数据) ──
    // 灯带和激光模块都发队伍色光, 但形状不同:
    //   灯带: 细长线条 (aspect > 2.5, 圆形度低)
    //   模块: 紧凑光斑 (φ42mm, 圆形度 > 0.3, aspect < 2.0)
    // 灯带距模块 ≥250mm (S111规则), 在ROI中有明显间隔
    // 算法: 逐轮廓分析, 有紧凑轮廓→保留, 只有细长轮廓→拒绝
    std::string enemy_color = "blue";         // 敌方颜色 ("blue" 或 "red")
    bool strip_reject_enabled = true;         // 是否启用灯带拒绝
    float strip_color_ratio_threshold = 0.25f; // 队伍色像素占比阈值
    int strip_min_saturation = 80;             // HSV 饱和度 S 最低阈值
    int strip_min_value = 80;                  // HSV 明度 V 最低阈值
    float strip_contour_elongation = 2.5f;     // 细长判定: 最小外接矩形长宽比

    // ── 实验性: 第2层 ROI 超分 (SR, 默认关) ──
    // 当 ROI 较小时, 用 ESPCN ×2 上采样后再喂给第2层, 期望提升远距小目标检出.
    // 需要编译时启用 DETECT_WITH_ONNX, 并提供 sr_onnx 路径.
    // 数据稀缺暂不期望大幅提升, 留做实场 A/B 对比.
    bool sr_enable = false;             // 总开关 (默认关)
    std::string sr_onnx;                // ESPCN ONNX 路径 (相对 cascade.yaml)
    int sr_scale = 2;                   // 上采样倍数 (与训练一致)
    int sr_max_roi_size = 320;          // ROI 短边 < 此值才上 SR (大于则跳过, 节省算力)

    // SR 之后的 unsharp mask 锐化 (T 项目同款 trick, 几乎零成本):
    //   sharp = (1+amount)*src + (-amount)*GaussianBlur(src, sigma)
    // 把边缘高频补回去, 帮助 YOLO 卷积核响应更强.
    bool sr_sharpen_enable = true;      // SR 启用时默认开锐化 (sr_enable=0 时本字段无效)
    float sr_sharpen_sigma = 2.0f;      // 高斯模糊核 sigma (与 T 项目一致)
    float sr_sharpen_amount = 0.4f;     // 锐化强度 (T 项目: 0.4)

    // ── 相对偏移位置预测 ──
    // 第3次锁定后模块停止发光, 无法直接视觉检测。
    // 利用前面成功检测时记录的「模块相对无人机bbox的归一化偏移」,
    // 只要还能检测到无人机本体, 就能预测模块位置。
    // 偏移 = (模块中心 - 无人机bbox中心) / 无人机bbox尺寸
    bool offset_predict_enabled = true;       // 是否启用偏移预测
    int offset_min_samples = 10;              // 至少积累多少次有效检测才信任偏移
    float offset_ema_alpha = 0.1f;            // 偏移量 EMA 平滑系数
    int offset_predict_max_frames = 150;      // 偏移预测最多持续帧数
};

// ── 无人机检测结果 ──
struct PlaneDetection {
    cv::Rect bbox;             // 全帧坐标下的 bbox
    cv::Point2f center;        // bbox 中心点
    float confidence = 0.0f;   // 检测置信度
};

// ── 激光检测模块检测结果 ──
struct LaserRxDetection {
    cv::Rect bbox;             // 全帧坐标下的 bbox
    cv::Point2f center;        // 模块中心 (全帧坐标)
    float confidence = 0.0f;   // 检测置信度
    int plane_index = -1;      // 所属无人机的索引
    cv::Rect roi;              // 检测使用的 ROI 区域
};

// ── 级联检测完整结果 ──
struct CascadeResult {
    std::vector<PlaneDetection> planes;      // 检测到的无人机列表
    std::vector<LaserRxDetection> laser_rxs; // 检测到的激光模块列表

    // 最佳目标 (最高置信度的 laser_rx)
    bool has_target = false;
    cv::Point2f target_center;
    float target_confidence = 0.0f;

    // 偏移预测的目标 (当直接检测丢失时)
    bool has_predicted_target = false;
    cv::Point2f predicted_center;
    float predicted_confidence = 0.0f;       // 预测置信度 (低于直接检测)
};

// 加载级联检测器配置
bool loadCascadeConfig(const std::string& path, CascadeConfig* out);

// 计算无人机上方的 ROI 区域
cv::Rect computeRoi(const cv::Rect& plane_bbox, int img_w, int img_h,
                     float top_extend, float bottom_extend,
                     float width_scale, int min_size);

class CascadeDetector : public Detector {
public:
    CascadeDetector();
    ~CascadeDetector() override;

    bool loadConfig(const std::string& path);

    // 标准检测接口: 返回全帧坐标下的 laser_rx 检测结果列表
    std::vector<Detection> detect(const cv::Mat& bgr,
                                    int64_t timestamp) override;

    // 详细级联结果: 包含无人机、激光模块、偏移预测等全部中间数据
    CascadeResult detectCascade(const cv::Mat& bgr, int64_t timestamp);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace detect

#endif  // DETECT_CASCADE_DETECTOR_H
