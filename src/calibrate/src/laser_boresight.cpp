// ============================================================================
// 激光-相机同轴校准工具 (laser_calib)
// ============================================================================
// 原理:
//   激光点在图像中的位置: u = cx + fx*α + fx*dx/Z
//                         v = cy + fy*β + fy*dy/Z
//   其中:
//     (dx, dy) = 激光到相机光心的物理偏移 (米)
//     (α, β)   = 激光与相机光轴的角度偏差 (弧度)
//     Z        = 目标距离 (米)
//
// 用法:
//   1. 启动后自动降低曝光 (白天也能看到激光点)
//   2. WASD 控制云台瞄准白纸/白墙
//   3. 程序自动检测激光点 (低曝光下只有激光可见)
//   4. 按 'c' 采集样本, 在终端输入距离 (米)
//   5. 移动白纸到不同距离, 重复采集 (至少3个距离)
//   6. 按 'f' 拟合并输出校准结果
//   7. 按 'r' 保存到 control.yaml
//
// 编译: cmake --build tests/build --target laser_calib
// 运行: ./tests/build/laser_calib [--config ../config/camera.yaml]
//        [--port /dev/ttyUSB0] [--exposure 200]
// ============================================================================

#include "galaxy_camera/galaxy_camera.h"
#include "gimbal_serial/protocol.h"
#include "gimbal_serial/serial_port.h"
#include "common/time_utils.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/freetype.hpp>

#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <filesystem>
#include <tuple>
#include <limits>
#include <ctime>
#include <cstdio>
#include <poll.h>
#include <unistd.h>

// ── 校准样本 ──
struct CalibSample {
    double Z;               // 距离 (m)
    cv::Point2f dot;        // 激光点像素坐标
    double dot_area;        // 光斑面积 (px)
};

// ── 校准结果 ──
struct CalibResult {
    double dx;              // 物理横向偏移 (m), 正=激光在相机右侧
    double dy;              // 物理纵向偏移 (m), 正=激光在相机下方
    double alpha_deg;       // 角度偏差 yaw (度)
    double beta_deg;        // 角度偏差 pitch (度)
    double fit_error_u;     // u 方向拟合残差 (px)
    double fit_error_v;     // v 方向拟合残差 (px)
    bool valid = false;
};

// ── 检测激光点 (自适应光斑大小) ──
// 低曝光模式下: 画面几乎全黑, 只有激光点是亮的
// 正常曝光下: 找最亮的过曝区域
// 远距离光斑大(发散), 近距离光斑小(聚焦) → 面积范围要宽
cv::Point2f detectLaserDot(const cv::Mat& frame, cv::Mat& vis,
                           int threshold_val, double* out_area = nullptr) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 高阈值提取过曝区域
    cv::Mat bright;
    cv::threshold(gray, bright, threshold_val, 255, cv::THRESH_BINARY);

    // 形态学: 先膨胀连接碎片, 再开运算去噪
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::dilate(bright, bright, kernel, cv::Point(-1, -1), 1);
    cv::Mat kernel_small = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(bright, bright, cv::MORPH_OPEN, kernel_small);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bright, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 选亮度加权质心最亮的连通域
    // 面积范围: 2px (近距离紧聚焦) ~ 50000px (远距离大光斑)
    cv::Point2f best_center(-1, -1);
    double best_score = 0;
    double best_area = 0;

    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area < 2 || area > 50000) continue;

        cv::Rect roi = cv::boundingRect(cnt);
        roi &= cv::Rect(0, 0, gray.cols, gray.rows);
        if (roi.width < 1 || roi.height < 1) continue;

        // 圆度检查: 激光光斑应该是近似圆形的
        double perimeter = cv::arcLength(cnt, true);
        double circularity = (perimeter > 0) ? 4.0 * M_PI * area / (perimeter * perimeter) : 0;
        if (circularity < 0.15) continue;  // 太不圆的排除 (0.15很宽松,允许椭圆)

        // 第1步: 灰度加权质心 (粗定位)
        cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8U);
        cv::drawContours(mask, std::vector<std::vector<cv::Point>>{cnt}, 0, 255, cv::FILLED);

        double sum_w = 0, sum_wx = 0, sum_wy = 0;
        for (int y = roi.y; y < roi.y + roi.height; y++) {
            const uint8_t* m_row = mask.ptr<uint8_t>(y);
            const uint8_t* g_row = gray.ptr<uint8_t>(y);
            for (int x = roi.x; x < roi.x + roi.width; x++) {
                if (m_row[x] == 0) continue;
                double w = g_row[x];
                sum_w += w;
                sum_wx += w * x;
                sum_wy += w * y;
            }
        }
        if (sum_w < 1) continue;

        cv::Point2f coarse_center(sum_wx / sum_w, sum_wy / sum_w);

        // 第2步: 高斯精修 — 在粗定位周围小窗口用梯度加权
        // 解决高曝光时整个光斑饱和到255导致加权质心退化的问题
        cv::Point2f center = coarse_center;
        {
            int half = std::max(3, (int)(std::sqrt(area / M_PI) * 0.8));
            int rx0 = std::max(0, (int)coarse_center.x - half);
            int ry0 = std::max(0, (int)coarse_center.y - half);
            int rx1 = std::min(gray.cols, (int)coarse_center.x + half + 1);
            int ry1 = std::min(gray.rows, (int)coarse_center.y + half + 1);
            // 对小窗口做高斯模糊得到亚像素梯度
            cv::Mat patch = gray(cv::Rect(rx0, ry0, rx1 - rx0, ry1 - ry0));
            cv::Mat patch_f;
            patch.convertTo(patch_f, CV_64F);
            // 用 w = val^4 加权, 让最亮核心权重远大于边缘
            double sw = 0, swx = 0, swy = 0;
            for (int py = 0; py < patch_f.rows; py++) {
                const double* row = patch_f.ptr<double>(py);
                for (int px = 0; px < patch_f.cols; px++) {
                    double v = row[px] / 255.0;
                    double w = v * v * v * v;  // 4次幂
                    sw += w;
                    swx += w * (rx0 + px);
                    swy += w * (ry0 + py);
                }
            }
            if (sw > 1e-6) {
                center = cv::Point2f(swx / sw, swy / sw);
            }
        }

        double mean_brightness = sum_w / area;

        // 评分 = 平均亮度 × 圆度 (亮+圆 = 激光)
        double score = mean_brightness * (0.5 + 0.5 * circularity);

        if (score > best_score) {
            best_score = score;
            best_center = center;
            best_area = area;
        }
    }

    // 可视化
    if (best_center.x >= 0) {
        int r = std::max(8, (int)std::sqrt(best_area / M_PI) + 5);
        cv::circle(vis, best_center, r, cv::Scalar(0, 255, 0), 2);
        cv::circle(vis, best_center, 2, cv::Scalar(0, 255, 0), -1);
        cv::putText(vis, cv::format("(%.1f, %.1f) A=%.0f", best_center.x, best_center.y, best_area),
                    best_center + cv::Point2f(r + 5, -5), cv::FONT_HERSHEY_SIMPLEX,
                    0.45, cv::Scalar(0, 255, 0), 1);
        if (out_area) *out_area = best_area;
    }

    return best_center;
}

// ── 最小二乘拟合 ──
// 模型: pixel = A + B / Z
std::pair<double, double> linearFit(const std::vector<double>& inv_Z,
                                     const std::vector<double>& pixel) {
    int n = inv_Z.size();
    double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
    for (int i = 0; i < n; i++) {
        sum_x += inv_Z[i];
        sum_y += pixel[i];
        sum_xx += inv_Z[i] * inv_Z[i];
        sum_xy += inv_Z[i] * pixel[i];
    }
    double denom = n * sum_xx - sum_x * sum_x;
    if (std::abs(denom) < 1e-12) return {0, 0};
    double B = (n * sum_xy - sum_x * sum_y) / denom;
    double A = (sum_y - B * sum_x) / n;
    return {A, B};
}

double medianOf(std::vector<double> vals) {
    if (vals.empty()) return 0.0;
    size_t mid = vals.size() / 2;
    std::nth_element(vals.begin(), vals.begin() + mid, vals.end());
    double hi = vals[mid];
    if (vals.size() % 2 == 1) return hi;
    std::nth_element(vals.begin(), vals.begin() + mid - 1, vals.begin() + mid);
    return 0.5 * (vals[mid - 1] + hi);
}

size_t pruneDistanceOutliers(std::vector<CalibSample>* samples) {
    if (!samples || samples->size() < 25) return 0;

    std::map<int, std::vector<CalibSample>> by_dist_cm;
    for (const auto& s : *samples) {
        by_dist_cm[static_cast<int>(s.Z * 100.0 + 0.5)].push_back(s);
    }

    std::vector<CalibSample> kept;
    kept.reserve(samples->size());
    std::map<int, int> removed_by_dist;

    for (const auto& [zcm, group] : by_dist_cm) {
        if (group.size() < 25) {
            kept.insert(kept.end(), group.begin(), group.end());
            continue;
        }

        std::vector<double> us, vs, areas;
        us.reserve(group.size());
        vs.reserve(group.size());
        areas.reserve(group.size());
        for (const auto& s : group) {
            us.push_back(s.dot.x);
            vs.push_back(s.dot.y);
            areas.push_back(s.dot_area);
        }

        const double mu = medianOf(us);
        const double mv = medianOf(vs);
        const double ma = medianOf(areas);

        std::vector<double> du, dv, da;
        du.reserve(group.size());
        dv.reserve(group.size());
        da.reserve(group.size());
        for (const auto& s : group) {
            du.push_back(std::abs(s.dot.x - mu));
            dv.push_back(std::abs(s.dot.y - mv));
            da.push_back(std::abs(s.dot_area - ma));
        }

        const double madu = std::max(medianOf(du), 0.05);
        const double madv = std::max(medianOf(dv), 0.05);
        const double mada = std::max(medianOf(da), 1.0);
        const double u_thr = std::max(2.0, 6.0 * madu);
        const double v_thr = std::max(2.0, 6.0 * madv);
        const double a_thr = std::max(20.0, 8.0 * mada);

        for (const auto& s : group) {
            const bool bad_u = std::abs(s.dot.x - mu) > u_thr;
            const bool bad_v = std::abs(s.dot.y - mv) > v_thr;
            const bool bad_a = std::abs(s.dot_area - ma) > a_thr;
            if (bad_u || bad_v || bad_a) {
                removed_by_dist[zcm]++;
            } else {
                kept.push_back(s);
            }
        }
    }

    const size_t removed = samples->size() - kept.size();
    if (removed == 0) return 0;

    std::cout << cv::format("[清洗] 删除疑似误识别样本 %d 个:", static_cast<int>(removed));
    for (const auto& [zcm, cnt] : removed_by_dist) {
        std::cout << cv::format(" %.2fm:%d", zcm / 100.0, cnt);
    }
    std::cout << "\n";

    *samples = std::move(kept);
    return removed;
}

// ── 拟合校准 ──
CalibResult fitCalibration(const std::vector<CalibSample>& samples,
                           double fx, double fy, double cx, double cy) {
    CalibResult result;
    if (samples.size() < 2) {
        std::cerr << "至少需要 2 个不同距离的样本 (建议 3+)\n";
        return result;
    }

    std::vector<double> inv_Z, u_vals, v_vals;
    for (const auto& s : samples) {
        inv_Z.push_back(1.0 / s.Z);
        u_vals.push_back(s.dot.x);
        v_vals.push_back(s.dot.y);
    }

    auto [Au, Bu] = linearFit(inv_Z, u_vals);
    auto [Av, Bv] = linearFit(inv_Z, v_vals);

    result.dx = Bu / fx;
    result.dy = Bv / fy;
    result.alpha_deg = std::atan((Au - cx) / fx) * 180.0 / M_PI;
    result.beta_deg  = std::atan((Av - cy) / fy) * 180.0 / M_PI;

    double err_u = 0, err_v = 0;
    for (int i = 0; i < (int)samples.size(); i++) {
        double pred_u = Au + Bu * inv_Z[i];
        double pred_v = Av + Bv * inv_Z[i];
        err_u += (pred_u - u_vals[i]) * (pred_u - u_vals[i]);
        err_v += (pred_v - v_vals[i]) * (pred_v - v_vals[i]);
    }
    result.fit_error_u = std::sqrt(err_u / samples.size());
    result.fit_error_v = std::sqrt(err_v / samples.size());
    result.valid = true;
    return result;
}

// ── 打印结果 ──
void printResult(const CalibResult& r) {
    std::cout << "\n"
              << "========================================\n"
              << "    激光-相机同轴校准结果\n"
              << "========================================\n"
              << "  物理偏移:\n"
              << cv::format("    laser_offset_x: %+.4f m (%+.1f mm)\n", r.dx, r.dx * 1000)
              << cv::format("    laser_offset_y: %+.4f m (%+.1f mm)\n", r.dy, r.dy * 1000)
              << "  角度偏差 (光轴不平行):\n"
              << cv::format("    yaw:   %+.4f deg\n", r.alpha_deg)
              << cv::format("    pitch: %+.4f deg\n", r.beta_deg)
              << "  拟合精度:\n"
              << cv::format("    RMS_u: %.2f px  RMS_v: %.2f px\n", r.fit_error_u, r.fit_error_v)
              << "========================================\n" << std::flush;
}

struct SavedSession {
    int next_dist_idx = 0;
    int next_repeat = 0;
    std::vector<CalibSample> samples;
    bool valid = false;
};

bool saveSession(const std::string& path,
                 int next_dist_idx,
                 int next_repeat,
                 const std::vector<CalibSample>& samples) {
    std::error_code ec;
    std::filesystem::path p(path);
    if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path(), ec);
    }

    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "[WARN] 无法写入进度文件: " << path << "\n";
        return false;
    }
    fs << "session_version" << 1;
    fs << "next_dist_idx" << next_dist_idx;
    fs << "next_repeat" << next_repeat;
    fs << "samples" << "[";
    for (const auto& s : samples) {
        fs << "{"
           << "Z" << s.Z
           << "u" << s.dot.x
           << "v" << s.dot.y
           << "area" << s.dot_area
           << "}";
    }
    fs << "]";
    fs.release();
    return true;
}

bool saveSessionSnapshot(const std::string& path,
                         int next_dist_idx,
                         int next_repeat,
                         const std::vector<CalibSample>& samples,
                         const std::string& source_session_path) {
    std::error_code ec;
    std::filesystem::path p(path);
    if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path(), ec);
    }

    if (std::filesystem::exists(p, ec)) {
        return true;
    }

    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "[WARN] 无法写入历史快照: " << path << "\n";
        return false;
    }
    fs << "session_version" << 1;
    fs << "snapshot_version" << 1;
    fs << "saved_unix_ms" << static_cast<double>(common::nowMs());
    fs << "source_session_path" << source_session_path;
    fs << "next_dist_idx" << next_dist_idx;
    fs << "next_repeat" << next_repeat;
    fs << "samples" << "[";
    for (const auto& s : samples) {
        fs << "{"
           << "Z" << s.Z
           << "u" << s.dot.x
           << "v" << s.dot.y
           << "area" << s.dot_area
           << "}";
    }
    fs << "]";
    fs.release();
    return true;
}

bool loadSession(const std::string& path, SavedSession* out) {
    if (!out) return false;
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }
    int version = 0;
    fs["session_version"] >> version;
    if (version != 1) {
        std::cerr << "[WARN] 进度文件版本不匹配: " << path << "\n";
        return false;
    }

    fs["next_dist_idx"] >> out->next_dist_idx;
    fs["next_repeat"] >> out->next_repeat;
    cv::FileNode samples_node = fs["samples"];
    out->samples.clear();
    for (auto it = samples_node.begin(); it != samples_node.end(); ++it) {
        CalibSample s{};
        (*it)["Z"] >> s.Z;
        (*it)["u"] >> s.dot.x;
        (*it)["v"] >> s.dot.y;
        (*it)["area"] >> s.dot_area;
        out->samples.push_back(s);
    }
    out->valid = true;
    return true;
}

void clearSession(const std::string& path) {
    std::remove(path.c_str());
}

int main(int argc, char** argv) {
    // ── 参数解析 ──
    std::string camera_config = "config/camera.yaml";
    std::string control_config = "config/control.yaml";
    std::string serial_port = "/dev/ttyUSB0";
    std::string session_path = "logs/laser_calib_session.yaml";
    int serial_baud = 115200;
    int dot_threshold = 200;
    double calib_exposure_us = 200.0;  // 校准专用低曝光
    bool resume_session = false;
    bool fresh_session = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if ((arg == "--config" || arg == "-c") && i + 1 < argc)
            camera_config = argv[++i];
        else if (arg == "--control" && i + 1 < argc)
            control_config = argv[++i];
        else if (arg == "--port" && i + 1 < argc)
            serial_port = argv[++i];
        else if (arg == "--baud" && i + 1 < argc)
            serial_baud = std::stoi(argv[++i]);
        else if (arg == "--threshold" && i + 1 < argc)
            dot_threshold = std::stoi(argv[++i]);
        else if (arg == "--exposure" && i + 1 < argc)
            calib_exposure_us = std::stod(argv[++i]);
        else if (arg == "--session" && i + 1 < argc)
            session_path = argv[++i];
        else if (arg == "--resume-session")
            resume_session = true;
        else if (arg == "--fresh-session")
            fresh_session = true;
    }

    // ── 读取相机内参 ──
    double fx = 1293.0, fy = 1292.3, cx = 641.0, cy = 493.3;
    {
        cv::FileStorage fs(camera_config, cv::FileStorage::READ);
        if (fs.isOpened()) {
            cv::FileNode cm = fs["camera_matrix"];
            if (!cm.empty() && cm.isMap()) {
                std::vector<double> d;
                cm["data"] >> d;
                if (d.size() == 9) { fx = d[0]; fy = d[4]; cx = d[2]; cy = d[5]; }
            }
        }
    }
    std::cout << cv::format("内参: fx=%.1f fy=%.1f cx=%.1f cy=%.1f\n", fx, fy, cx, cy);

    // ── 打开相机 ──
    galaxy_camera::GalaxyCamera galaxy;
    cv::VideoCapture cap;
    bool use_galaxy = true;

    {
        galaxy_camera::CameraConfig gcfg;
        galaxy_camera::loadCameraConfig(camera_config, &gcfg);
        if (!galaxy.open(gcfg) || !galaxy.startGrabbing()) {
            std::cerr << "Galaxy 相机失败, 尝试 OpenCV(0)\n";
            use_galaxy = false;
            cap.open(0);
            if (!cap.isOpened()) {
                std::cerr << "无法打开任何相机\n";
                return 1;
            }
        }
    }

    // ── 中文字体 ──
    auto ft2 = cv::freetype::createFreeType2();
    ft2->loadFontData("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 0);

    // ── 曝光 (手动无级调节, 记忆上次值) ──
    double current_exposure_us = calib_exposure_us;  // 默认低曝光
    {
        // 尝试加载上次保存的曝光值
        std::string exp_file = camera_config.substr(0, camera_config.rfind('/') + 1) + ".boresight_exposure";
        std::ifstream ifs(exp_file);
        if (ifs.is_open()) {
            double saved = 0;
            ifs >> saved;
            if (saved >= 50.0 && saved <= 50000.0) {
                current_exposure_us = saved;
                std::cout << cv::format("加载上次曝光: %.0f us\n", current_exposure_us);
            }
        }
    }
    if (use_galaxy) {
        galaxy.setExposure(current_exposure_us);
        std::cout << cv::format("曝光: %.0f us (e=升高 t=降低)\n", current_exposure_us);
    }

    // ── 串口 (云台控制, 同 track.cpp 主循环架构) ──
    gimbal_serial::SerialPort serial;
    gimbal_serial::FrameParser parser;
    std::vector<uint8_t> rx_buf(512);
    common::GimbalState gimbal_state{};
    common::GimbalState prev_gimbal_state{};
    bool has_gimbal = false;
    bool has_gimbal_state = false;
    bool has_prev_gimbal_state = false;
    int64_t last_rx_ms = 0;
    int64_t last_link_warn_ms = 0;
    constexpr double kStateJumpRejectDeg = 25.0;
    constexpr double kReturnSettleTolDeg = 0.8;
    constexpr int64_t kFeedbackFreshTimeoutMs = 250;
    constexpr int kJumpRejectResyncCount = 8;
    int jump_reject_streak = 0;
    bool gimbal_feedback_fresh = false;
    auto angle_diff_deg = [](float a, float b) {
        double diff = std::fmod(static_cast<double>(a) - static_cast<double>(b), 360.0);
        if (diff > 180.0) diff -= 360.0;
        if (diff <= -180.0) diff += 360.0;
        return diff;
    };

    if (serial.open(serial_port, serial_baud)) {
        has_gimbal = true;
        std::cout << "串口: " << serial_port << " (WASD控制云台)\n" << std::flush;
    } else {
        std::cout << "串口未连接 (无云台控制, 可手动对准)\n";
    }

    auto manual_step_deg = [&](double dist_m) -> float {
        if (dist_m >= 1.9) return 0.15f;  // 2m: 细调
        if (dist_m >= 0.9) return 0.30f;  // 1m: 中等
        if (dist_m >= 0.4) return 0.60f;  // 0.5m: 稍快
        return 0.50f;
    };

    std::cout << "\n"
              << "===================================\n"
              << " 激光同轴校准工具\n"
              << "===================================\n"
              << " 相机:\n"
              << "   'e' = 曝光升高  't' = 曝光降低\n"
              << "   '+'/'-' = 调节检测阈值\n"
              << " 云台:\n"
              << "   W/S = pitch 上/下  A/D = yaw 左/右\n"
              << " 设距离 (视频窗口内按数字键):\n"
              << "   1=0.5m  2=1m  3=1.5m  4=2m  5=3m  6=5m\n"
              << " 校准:\n"
              << "   'g' = 多距离引导标定 (0.5m/1m/2m 各两遍)\n"
              << "   'c' = 手动保存样本  'f' = 手动拟合\n"
              << "   'r' = 保存结果  'X' = 清空样本\n"
              << "   空格 = 确认就位 / 提前结束扫描\n"
              << "   'q'/ESC = 退出\n"
              << "===================================\n\n";

    cv::namedWindow("Laser Calib", cv::WINDOW_NORMAL);
    std::vector<CalibSample> samples;
    CalibResult last_result;

    // 连续采集状态
    double current_Z = 0;           // 当前设定距离, 0=未设定
    double sum_u = 0, sum_v = 0;    // 累积像素坐标
    int collect_count = 0;          // 累积帧数
    double live_dx = 0, live_dy = 0; // 实时偏移估算

    // ── 自动校准: 费马螺线 (Fermat spiral, 均匀面积覆盖) ──
    // r = a * sqrt(θ), 每圈覆盖面积相等 (ΔA = 2π²a² = const)
    enum class AutoState { IDLE, MULTI_PROMPT, PREPARE, SPIRAL, MANUAL_CAPTURE, RETURNING, DONE };
    AutoState auto_state = AutoState::IDLE;
    double auto_base_dist = 0;          // 垂直距离到墙面 (m)
    double auto_theta = 0;              // 螺旋角度 (弧度)
    double auto_start_yaw = 0;          // 扫描起始yaw (度, IMU绝对角)
    double auto_start_pitch = 0;        // 扫描起始pitch (度, IMU绝对角)
    double auto_offset_yaw = 0;         // 螺旋yaw偏移 (度, 相对起点)
    double auto_offset_pitch = 0;       // 螺旋pitch偏移 (度, 相对起点)
    float manual_yaw = 0, manual_pitch = 0;  // IDLE时的手动目标角度
    bool manual_init = false;                 // 手动目标已从云台反馈初始化
    constexpr double kManualTxMaxDeltaDeg = 1.2;  // 手动控制每帧最大角度增量
    constexpr double kPrimarySpiralMaxDeg = 5.4;   // 第1遍最大扫描半径 (度), 去掉激进外圈
    constexpr double kRepeatSpiralMaxDeg = 4.0;    // 第2遍最大扫描半径 (度), 作为补扫
    constexpr double kPrimaryStepDeg = 0.30;       // 第1遍每帧弧长 (度), 全局降速
    constexpr double kRepeatStepDeg = 0.24;        // 第2遍每帧弧长 (度), 再轻一点
    constexpr double kReturnStepDeg = 0.5;         // 回正步长 (度)
    constexpr double kMaxDTheta = 0.3;        // 最大角度步进 (rad/frame)
    constexpr double kPrimarySpiralTurns = 6.0;  // 第1遍只扫稳定的前6圈
    constexpr double kRepeatSpiralTurns = 5.0;   // 第2遍只做更短的补扫
    constexpr double kPrepareTolDeg = 0.8;    // 开扫前回零容差 (度)
    constexpr int64_t kPrepareStableMs = 1200;  // 回零后稳定时长 (ms)
    struct SpiralProfile {
        double max_deg;
        double step_deg;
        double turns;
        double max_theta;
        double fermat_a;
    };
    auto make_spiral_profile = [&](double max_deg, double step_deg,
                                   double turns) {
        return SpiralProfile{
            max_deg,
            step_deg,
            turns,
            2 * M_PI * turns,
            max_deg / std::sqrt(2 * M_PI * turns),
        };
    };
    const SpiralProfile kPrimarySpiral =
        make_spiral_profile(kPrimarySpiralMaxDeg, kPrimaryStepDeg,
                            kPrimarySpiralTurns);
    const SpiralProfile kRepeatSpiral =
        make_spiral_profile(kRepeatSpiralMaxDeg, kRepeatStepDeg,
                            kRepeatSpiralTurns);
    auto select_spiral_profile = [&](int repeat) {
        return repeat > 0 ? kRepeatSpiral : kPrimarySpiral;
    };
    SpiralProfile auto_spiral = kPrimarySpiral;
    int64_t auto_prepare_stable_since_ms = 0;

    // ── 多距离引导标定 ──
    constexpr double kMultiDist[] = {0.5, 1.0, 2.0};
    constexpr int kMultiDistN = 3;
    constexpr int kMultiRep = 2;
    bool multi_mode = false;
    int multi_dist_idx = 0;
    int multi_repeat = 0;
    bool retry_current_scan = false;
    size_t auto_pass_sample_begin = 0;
    int auto_track_bad_frames = 0;
    constexpr double kAutoTrackErrDeg = 8.0;
    constexpr int kAutoTrackBadFramesLimit = 2;
    constexpr double kAutoTxMaxDeltaDeg = 3.0;
    constexpr int kManualPassFrames = 1000;
    int auto_tx_clamp_streak = 0;
    int manual_tx_clamp_streak = 0;
    int prompt_idle_frames = 0;
    bool prompt_manual_control = false;
    bool auto_pass_manual_capture = false;
    int manual_pass_frames = 0;
    int manual_capture_pause_frames = 0;
    const std::filesystem::path session_snapshot_dir = [&]() {
        std::filesystem::path p(session_path);
        if (p.has_parent_path()) {
            return p.parent_path() / "laser_calib_snapshots";
        }
        return std::filesystem::path("laser_calib_snapshots");
    }();

    auto progress_score = [&](int next_dist_idx, int next_repeat) {
        if (next_dist_idx >= kMultiDistN) {
            return kMultiDistN * kMultiRep;
        }
        return next_dist_idx * kMultiRep + next_repeat;
    };

    auto dist_tag = [](double dist_m) {
        std::string s = cv::format("%.1f", dist_m);
        std::replace(s.begin(), s.end(), '.', 'p');
        return s + "m";
    };

    auto snapshot_path_for_state = [&](int next_dist_idx, int next_repeat,
                                       size_t sample_count) {
        const int total = kMultiDistN * kMultiRep;
        std::string file_name;
        if (next_dist_idx < kMultiDistN) {
            const int scan_idx = next_dist_idx * kMultiRep + next_repeat + 1;
            file_name = cv::format("next_%02dof%02d_%s_pass%d_samples%05d.yaml",
                                   scan_idx, total,
                                   dist_tag(kMultiDist[next_dist_idx]).c_str(),
                                   next_repeat + 1,
                                   static_cast<int>(sample_count));
        } else {
            file_name = cv::format("done_samples%05d.yaml",
                                   static_cast<int>(sample_count));
        }
        return session_snapshot_dir / file_name;
    };

    auto save_history_snapshot = [&](int next_dist_idx, int next_repeat) {
        if (samples.empty()) {
            return;
        }
        std::filesystem::path snapshot_path =
            snapshot_path_for_state(next_dist_idx, next_repeat, samples.size());
        std::error_code ec;
        if (std::filesystem::exists(snapshot_path, ec)) {
            return;
        }
        if (saveSessionSnapshot(snapshot_path.string(), next_dist_idx,
                                next_repeat, samples, session_path)) {
            std::cout << cv::format("[快照] 已落盘: %s (%d样本)\n",
                                    snapshot_path.filename().string().c_str(),
                                    static_cast<int>(samples.size()))
                      << std::flush;
        }
    };

    auto load_best_history_snapshot =
        [&](SavedSession* out, std::filesystem::path* out_path) {
            if (!out) return false;
            std::error_code ec;
            if (!std::filesystem::exists(session_snapshot_dir, ec)) {
                return false;
            }

            bool found = false;
            SavedSession best{};
            std::filesystem::path best_path;
            int best_progress = -1;
            size_t best_samples = 0;
            std::filesystem::file_time_type best_mtime =
                std::filesystem::file_time_type::min();

            for (const auto& entry :
                 std::filesystem::directory_iterator(session_snapshot_dir, ec)) {
                if (ec || !entry.is_regular_file()) {
                    continue;
                }
                if (entry.path().extension() != ".yaml") {
                    continue;
                }
                SavedSession cur{};
                if (!loadSession(entry.path().string(), &cur) ||
                    !cur.valid || cur.samples.empty()) {
                    continue;
                }
                int cur_progress = progress_score(
                    std::clamp(cur.next_dist_idx, 0, kMultiDistN),
                    std::clamp(cur.next_repeat, 0, kMultiRep - 1));
                auto cur_mtime = entry.last_write_time(ec);
                if (ec) {
                    cur_mtime = std::filesystem::file_time_type::min();
                    ec.clear();
                }
                size_t cur_samples = cur.samples.size();
                if (!found ||
                    std::tie(cur_progress, cur_samples, cur_mtime) >
                        std::tie(best_progress, best_samples, best_mtime)) {
                    found = true;
                    best = cur;
                    best_path = entry.path();
                    best_progress = cur_progress;
                    best_samples = cur_samples;
                    best_mtime = cur_mtime;
                }
            }

            if (!found) {
                return false;
            }
            *out = best;
            if (out_path) {
                *out_path = best_path;
            }
            return true;
        };

    auto save_multi_session = [&](int next_dist_idx, int next_repeat) {
        if (!saveSession(session_path, next_dist_idx, next_repeat, samples)) {
            return;
        }
        save_history_snapshot(next_dist_idx, next_repeat);
        const int total = kMultiDistN * kMultiRep;
        if (next_dist_idx < kMultiDistN) {
            const int scan_idx = next_dist_idx * kMultiRep + next_repeat + 1;
            std::cout << cv::format("[自动保存] 已保存 %d 个样本, 下次可从 [%d/%d] %.1fm 第%d遍继续\n",
                                    (int)samples.size(),
                                    scan_idx, total,
                                    kMultiDist[next_dist_idx],
                                    next_repeat + 1) << std::flush;
        } else {
            std::cout << cv::format("[自动保存] 已保存全部 %d 个样本, 下次可直接继续保存结果\n",
                                    (int)samples.size()) << std::flush;
        }
    };

    if (fresh_session) {
        clearSession(session_path);
    }

    SavedSession resumed{};
    if (resume_session) {
        SavedSession primary{};
        SavedSession history{};
        std::filesystem::path history_path;
        bool loaded_primary = loadSession(session_path, &primary) && primary.valid;
        bool loaded_history = load_best_history_snapshot(&history, &history_path);

        if (loaded_primary) {
            resumed = primary;
        } else if (loaded_history) {
            resumed = history;
            std::cout << cv::format("[恢复] 检测到更完整的历史快照: %s\n",
                                    history_path.filename().string().c_str());
        }

        if ((loaded_primary || loaded_history) && resumed.valid) {
            samples = resumed.samples;
            pruneDistanceOutliers(&samples);
            multi_dist_idx = std::clamp(resumed.next_dist_idx, 0, kMultiDistN);
            multi_repeat = std::clamp(resumed.next_repeat, 0, kMultiRep - 1);
            retry_current_scan = false;
            auto_pass_sample_begin = samples.size();
            std::cout << cv::format("[恢复] 已加载上次进度: %d 个样本\n", (int)samples.size());
            if (multi_dist_idx >= kMultiDistN) {
                if (samples.size() >= 3) {
                    last_result = fitCalibration(samples, fx, fy, cx, cy);
                    if (last_result.valid) {
                        std::cout << cv::format("\n[多距离标定] 全部完成! %d 个样本, 按 'r' 保存\n",
                                                (int)samples.size()) << std::flush;
                        printResult(last_result);
                        std::cout << "按 'r' 保存到 " << control_config << "\n";
                    }
                }
            } else {
                multi_mode = true;
                auto_state = AutoState::MULTI_PROMPT;
                const int total = kMultiDistN * kMultiRep;
                const int scan_idx = multi_dist_idx * kMultiRep + multi_repeat + 1;
                std::cout << cv::format("[%d/%d] 请移到 %.1fm 处, 按空格开始\n",
                                        scan_idx, total,
                                        kMultiDist[multi_dist_idx]) << std::flush;
            }
        } else {
            std::cerr << "[WARN] 未找到可恢复进度, 将从头开始\n";
        }
    }

    bool r_saved = false;
    while (true) {
        // ── 读取串口反馈 (同 track.cpp, 5ms超时) ──
        if (has_gimbal) {
            int n = serial.read(rx_buf.data(),
                                static_cast<int>(rx_buf.size()), 5);
            if (n > 0) {
                common::GimbalState st;
                if (parser.push(rx_buf.data(),
                                static_cast<size_t>(n), &st)) {
                    int64_t now_ms = common::nowMs();
                    auto invalid_state = [](float v) {
                        return !std::isfinite(v) || std::fpclassify(v) == FP_SUBNORMAL;
                    };
                    if (invalid_state(st.pitch) || invalid_state(st.yaw)) {
                        std::cerr << "[WARN] 丢弃异常回传: pitch=" << st.pitch
                                  << " yaw=" << st.yaw << "\n";
                        continue;
                    }
                    bool allow_resync = !has_prev_gimbal_state ||
                                        (last_rx_ms > 0 && now_ms - last_rx_ms > kFeedbackFreshTimeoutMs) ||
                                        jump_reject_streak >= kJumpRejectResyncCount;
                    if (has_prev_gimbal_state && !allow_resync) {
                        double dp = std::abs(angle_diff_deg(st.pitch, prev_gimbal_state.pitch));
                        double dy = std::abs(angle_diff_deg(st.yaw, prev_gimbal_state.yaw));
                        if (dp > kStateJumpRejectDeg || dy > kStateJumpRejectDeg) {
                            jump_reject_streak++;
                            if (jump_reject_streak == 1 || jump_reject_streak % 20 == 0) {
                                std::cerr << cv::format(
                                    "[WARN] 丢弃跳变回传: dp=%.1f° dy=%.1f° pitch=%.1f yaw=%.1f prev=(%.1f,%.1f)\n",
                                    dp, dy, st.pitch, st.yaw,
                                    prev_gimbal_state.pitch, prev_gimbal_state.yaw);
                            }
                            continue;
                        }
                    }
                    if (allow_resync && has_prev_gimbal_state) {
                        std::cerr << cv::format(
                            "[WARN] 云台回传重同步: pitch=%.1f yaw=%.1f stale_ms=%lld reject_streak=%d\n",
                            st.pitch, st.yaw,
                            static_cast<long long>(last_rx_ms > 0 ? (now_ms - last_rx_ms) : 0),
                            jump_reject_streak);
                    }
                    gimbal_state = st;
                    prev_gimbal_state = st;
                    has_prev_gimbal_state = true;
                    has_gimbal_state = true;
                    last_rx_ms = now_ms;
                    jump_reject_streak = 0;
                }
            }
        }
        // 首次收到云台反馈时, 初始化手动目标为当前角度
        if (!manual_init && has_gimbal_state) {
            manual_yaw = gimbal_state.yaw;
            manual_pitch = gimbal_state.pitch;
            manual_init = true;
        }

        cv::Mat frame;
        if (use_galaxy) {
            galaxy_camera::Frame gf;
            if (!galaxy.read(&gf, 50)) continue;
            if (gf.bgr.empty()) continue;
            frame = gf.bgr.clone();
        } else {
            if (!cap.read(frame) || frame.empty()) break;
        }

        cv::Mat vis = frame.clone();

        if (has_gimbal && last_rx_ms > 0) {
            int64_t now_ms = common::nowMs();
            gimbal_feedback_fresh = has_gimbal_state &&
                                    (now_ms - last_rx_ms <= kFeedbackFreshTimeoutMs);
            if (!gimbal_feedback_fresh) {
                has_prev_gimbal_state = false;
            }
            if (now_ms - last_rx_ms > 1200 && now_ms - last_link_warn_ms > 1500) {
                std::cerr << "[WARN] 超过1.2s未收到云台回传, 请检查串口线/下位机上电状态\n";
                last_link_warn_ms = now_ms;
            }
        } else {
            gimbal_feedback_fresh = false;
        }

        // ── 光心十字准星 (精密瞄准风格) ──
        {
            int icx = (int)cx, icy = (int)cy;
            cv::Scalar cross_color(255, 160, 0, 180);
            // 中心小圆
            cv::circle(vis, cv::Point(icx, icy), 3, cross_color, 1, cv::LINE_AA);
            // 十字线 (中间留空)
            cv::line(vis, cv::Point(icx - 50, icy), cv::Point(icx - 8, icy), cross_color, 1, cv::LINE_AA);
            cv::line(vis, cv::Point(icx + 8, icy), cv::Point(icx + 50, icy), cross_color, 1, cv::LINE_AA);
            cv::line(vis, cv::Point(icx, icy - 50), cv::Point(icx, icy - 8), cross_color, 1, cv::LINE_AA);
            cv::line(vis, cv::Point(icx, icy + 8), cv::Point(icx, icy + 50), cross_color, 1, cv::LINE_AA);
            // 刻度线
            for (int d : {15, 30, 45}) {
                cv::line(vis, cv::Point(icx - d, icy - 3), cv::Point(icx - d, icy + 3), cross_color, 1, cv::LINE_AA);
                cv::line(vis, cv::Point(icx + d, icy - 3), cv::Point(icx + d, icy + 3), cross_color, 1, cv::LINE_AA);
                cv::line(vis, cv::Point(icx - 3, icy - d), cv::Point(icx + 3, icy - d), cross_color, 1, cv::LINE_AA);
                cv::line(vis, cv::Point(icx - 3, icy + d), cv::Point(icx + 3, icy + d), cross_color, 1, cv::LINE_AA);
            }
        }

        // 检测激光点
        double dot_area = 0;
        cv::Point2f dot = detectLaserDot(frame, vis, dot_threshold, &dot_area);

        // ── 连续采集: 每帧更新偏移 ──
        if (current_Z > 0.01 && dot.x >= 0) {
            sum_u += dot.x;
            sum_v += dot.y;
            collect_count++;
            double avg_u = sum_u / collect_count;
            double avg_v = sum_v / collect_count;
            live_dx = (avg_u - cx) * current_Z / fx;
            live_dy = (avg_v - cy) * current_Z / fy;
        }

        // PREPARE: 开扫前先回到正前方并稳定一小段时间
        if (auto_state == AutoState::PREPARE && has_gimbal) {
            auto_start_yaw = 0.0;
            auto_start_pitch = 0.0;
            auto_offset_yaw = 0.0;
            auto_offset_pitch = 0.0;
            manual_yaw = 0.0f;
            manual_pitch = 0.0f;
            if (gimbal_feedback_fresh && has_gimbal_state) {
                const int64_t now_ms = common::nowMs();
                const double yaw_zero_err =
                    std::abs(angle_diff_deg(gimbal_state.yaw, 0.0f));
                const double pitch_zero_err =
                    std::abs(angle_diff_deg(gimbal_state.pitch, 0.0f));
                if (yaw_zero_err <= kPrepareTolDeg &&
                    pitch_zero_err <= kPrepareTolDeg) {
                    if (auto_prepare_stable_since_ms <= 0) {
                        auto_prepare_stable_since_ms = now_ms;
                    }
                    if (now_ms - auto_prepare_stable_since_ms >=
                        kPrepareStableMs) {
                        auto_theta = 0;
                        auto_start_yaw = gimbal_state.yaw;
                        auto_start_pitch = gimbal_state.pitch;
                        auto_offset_yaw = 0.0;
                        auto_offset_pitch = 0.0;
                        auto_track_bad_frames = 0;
                        auto_state = AutoState::SPIRAL;
                        const int total = kMultiDistN * kMultiRep;
                        const int scan_idx =
                            multi_dist_idx * kMultiRep + multi_repeat;
                        std::cout << cv::format(
                            "[%d/%d] %.1fm 第%d遍开始, 起始角: yaw=%.1f° pitch=%.1f°  扫描R=%.1f° step=%.2f°\n",
                            scan_idx + 1, total, auto_base_dist,
                            multi_repeat + 1, auto_start_yaw, auto_start_pitch,
                            auto_spiral.max_deg, auto_spiral.step_deg)
                                  << std::flush;
                    }
                } else {
                    auto_prepare_stable_since_ms = 0;
                }
            } else {
                auto_prepare_stable_since_ms = 0;
            }
        }
        if (auto_state == AutoState::MANUAL_CAPTURE) {
            if (manual_capture_pause_frames > 0) {
                manual_capture_pause_frames--;
            } else if (dot.x >= 0 && auto_base_dist > 0.01) {
                samples.push_back({auto_base_dist, dot, dot_area});
                manual_pass_frames++;
                if (manual_pass_frames % 100 == 0) {
                    std::cout << cv::format("[手动采集] %d / %d 帧  %d样本\n",
                                            manual_pass_frames,
                                            kManualPassFrames,
                                            (int)samples.size()) << std::flush;
                }
                if (manual_pass_frames >= kManualPassFrames) {
                    auto_state = AutoState::DONE;
                }
            }
        }
        // ── 自动螺旋扫描 (绝对目标模式, 同system_demo) ──
        if (auto_state == AutoState::SPIRAL && has_gimbal) {
            double r_deg = auto_spiral.fermat_a * std::sqrt(auto_theta);

            if (auto_theta >= auto_spiral.max_theta ||
                r_deg > auto_spiral.max_deg) {
                auto_state = AutoState::DONE;
            } else {
                double target_yaw_now = auto_start_yaw + auto_offset_yaw;
                double target_pitch_now = auto_start_pitch + auto_offset_pitch;
                if (gimbal_feedback_fresh) {
                    double yaw_err = std::abs(angle_diff_deg(gimbal_state.yaw,
                                                             static_cast<float>(target_yaw_now)));
                    double pitch_err = std::abs(angle_diff_deg(gimbal_state.pitch,
                                                               static_cast<float>(target_pitch_now)));
                    if (yaw_err > kAutoTrackErrDeg || pitch_err > kAutoTrackErrDeg) {
                        auto_track_bad_frames++;
                    } else {
                        auto_track_bad_frames = 0;
                    }
                    if (auto_track_bad_frames >= kAutoTrackBadFramesLimit) {
                        size_t kept = auto_pass_sample_begin;
                        size_t removed = samples.size() - kept;
                        samples.resize(kept);
                        save_multi_session(multi_dist_idx, multi_repeat);
                        auto_track_bad_frames = 0;
                        retry_current_scan = true;
                        auto_theta = 0;
                        auto_start_yaw = 0.0;
                        auto_start_pitch = 0.0;
                        auto_offset_yaw = 0.0;
                        auto_offset_pitch = 0.0;
                        manual_yaw = gimbal_state.yaw;
                        manual_pitch = gimbal_state.pitch;
                        prompt_idle_frames = 12;
                        prompt_manual_control = false;
                        auto_state = AutoState::MULTI_PROMPT;
                        const int total = kMultiDistN * kMultiRep;
                        int scan_idx = multi_dist_idx * kMultiRep + multi_repeat;
                        std::cerr << cv::format(
                            "[WARN] 扫描失稳, 已删除当前这遍样本 %d 个并保留前一遍; "
                            "已停止自动控制, 请手动调整并按空格重试: "
                            "target=(%.1f,%.1f) state=(%.1f,%.1f) err=(%.1f,%.1f)\n",
                            static_cast<int>(removed),
                            target_yaw_now, target_pitch_now,
                            gimbal_state.yaw, gimbal_state.pitch,
                            yaw_err, pitch_err);
                        std::cout << cv::format("[%d/%d] 请移到 %.1fm 处, 按空格开始\n",
                                                scan_idx + 1, total,
                                                kMultiDist[multi_dist_idx]) << std::flush;
                        continue;
                    }
                }
                // 每帧采集数据
                if (dot.x >= 0 && auto_base_dist > 0.01) {
                    double yaw_rad = auto_offset_yaw * M_PI / 180.0;
                    double pitch_rad = auto_offset_pitch * M_PI / 180.0;
                    double Z = auto_base_dist / (std::cos(yaw_rad) * std::cos(pitch_rad));
                    samples.push_back({Z, dot, dot_area});
                }

                // 每帧推进一步螺旋
                {
                    double r_safe = std::max(r_deg, 1.0);
                    double d_theta =
                        std::min(auto_spiral.step_deg / r_safe, kMaxDTheta);
                    auto_theta += d_theta;

                    double new_r = auto_spiral.fermat_a * std::sqrt(auto_theta);
                    auto_offset_yaw = new_r * std::cos(auto_theta);
                    auto_offset_pitch = new_r * std::sin(auto_theta);

                    // 每圈打印一次进度
                    double rev = auto_theta / (2 * M_PI);
                    if (std::fmod(rev, 1.0) < d_theta / (2 * M_PI) + 0.02) {
                        std::cout << cv::format("[螺旋] 第%.0f圈  r=%.1f°  %d样本\n",
                                                std::ceil(rev), new_r, (int)samples.size());
                    }
                    // 每30帧打印调试信息
                    static int spiral_dbg = 0;
                    if (++spiral_dbg % 30 == 0) {
                        std::cout << cv::format("[DBG] theta=%.1f off=(%.2f,%.2f) "
                                                "target=(%.1f,%.1f) state=(%.1f,%.1f)\n",
                            auto_theta,
                            auto_offset_yaw, auto_offset_pitch,
                            auto_start_yaw + auto_offset_yaw,
                            auto_start_pitch + auto_offset_pitch,
                            gimbal_state.yaw, gimbal_state.pitch);
                    }
                }
            }
        }
        // DONE: 拟合, 然后切回手动等待下一遍
        if (auto_state == AutoState::DONE) {
            if (auto_pass_manual_capture) {
                std::cout << cv::format("\n[手动采集] 完成! %d 帧  总计 %d 个样本\n",
                                        manual_pass_frames,
                                        (int)samples.size());
            } else {
                std::cout << cv::format("\n[费马螺线] 完成! r=%.1f°  总计 %d 个样本\n",
                                        auto_spiral.fermat_a * std::sqrt(auto_theta),
                                        (int)samples.size());
            }
            if ((int)samples.size() >= 3) {
                pruneDistanceOutliers(&samples);
            }
            // 按距离统计样本分布
            {
                std::map<int, int> dist_count;  // Z(cm) → count
                for (auto& s : samples)
                    dist_count[static_cast<int>(s.Z * 100 + 0.5)]++;
                std::cout << "[样本分布]";
                for (auto& [zcm, cnt] : dist_count)
                    std::cout << cv::format(" %.2fm:%d", zcm / 100.0, cnt);
                std::cout << "\n";
            }
            if ((int)samples.size() >= 3) {
                last_result = fitCalibration(samples, fx, fy, cx, cy);
                if (last_result.valid) {
                    printResult(last_result);
                    std::cout << "按 'r' 保存到 " << control_config << "\n";
                }
            } else {
                std::cerr << "[螺旋] 样本不足 (" << samples.size() << "), 请检查激光/阈值\n";
            }
            if (multi_mode) {
                int next_dist_idx = multi_dist_idx;
                int next_repeat = multi_repeat + 1;
                if (next_repeat >= kMultiRep) {
                    next_repeat = 0;
                    next_dist_idx++;
                }
                save_multi_session(next_dist_idx, next_repeat);
                manual_yaw = has_gimbal_state ? gimbal_state.yaw : manual_yaw;
                manual_pitch = has_gimbal_state ? gimbal_state.pitch : manual_pitch;
                prompt_idle_frames = 12;
                prompt_manual_control = false;
                multi_repeat++;
                if (multi_repeat >= kMultiRep) {
                    multi_repeat = 0;
                    multi_dist_idx++;
                }
                int scan_idx = multi_dist_idx * kMultiRep + multi_repeat;
                if (multi_dist_idx >= kMultiDistN) {
                    multi_mode = false;
                    auto_state = AutoState::IDLE;
                    std::cout << cv::format("\n[多距离标定] 全部完成! %d 个样本, 按 'r' 保存\n",
                                            (int)samples.size()) << std::flush;
                } else {
                    auto_theta = 0.0;
                    auto_offset_yaw = 0.0;
                    auto_offset_pitch = 0.0;
                    auto_pass_manual_capture = false;
                    manual_pass_frames = 0;
                    auto_state = AutoState::MULTI_PROMPT;
                    std::cout << cv::format("[%d/%d] 请移到 %.1fm 处, 按空格开始\n",
                                            scan_idx + 1, kMultiDistN * kMultiRep,
                                            kMultiDist[multi_dist_idx]) << std::flush;
                }
            } else {
                auto_pass_manual_capture = false;
                manual_pass_frames = 0;
                auto_state = AutoState::IDLE;
            }
        }

        // ── 每帧发送云台命令 (同 track.cpp, 一帧一次) ──
        // yaw 方向与反馈一致, 直接发送绝对目标角即可.
        // pitch 为了匹配当前手动/GUI 操作约定, 仍需对增量取反.
        if (has_gimbal && gimbal_feedback_fresh) {
            float target_yaw, target_pitch;
            bool auto_target = false;
            if (auto_state == AutoState::PREPARE ||
                auto_state == AutoState::SPIRAL ||
                auto_state == AutoState::RETURNING) {
                target_yaw = static_cast<float>(auto_start_yaw + auto_offset_yaw);
                target_pitch = static_cast<float>(auto_start_pitch + auto_offset_pitch);
                auto_target = true;
            } else {
                target_yaw = manual_yaw;
                target_pitch = manual_pitch;
            }
            common::GimbalCommand cmd{};
            if (prompt_idle_frames > 0) {
                cmd.mode = static_cast<uint8_t>(common::GimbalCommandMode::Idle);
                prompt_idle_frames--;
                auto_tx_clamp_streak = 0;
            } else if (auto_state == AutoState::MULTI_PROMPT &&
                       !prompt_manual_control) {
                cmd.mode = static_cast<uint8_t>(common::GimbalCommandMode::Idle);
                auto_tx_clamp_streak = 0;
            } else if (auto_target) {
                cmd.mode = static_cast<uint8_t>(common::GimbalCommandMode::Control);
                // 协议下发的是 (cmd - state) 增量。扫描时对增量限幅，
                // 避免反馈发散后继续把十几二十度的纠偏量直接塞给电控。
                double raw_yaw_delta =
                    angle_diff_deg(target_yaw, gimbal_state.yaw);
                double raw_pitch_delta =
                    angle_diff_deg(gimbal_state.pitch, target_pitch);
                double yaw_delta =
                    std::clamp(raw_yaw_delta, -kAutoTxMaxDeltaDeg,
                               kAutoTxMaxDeltaDeg);
                double pitch_delta =
                    std::clamp(raw_pitch_delta, -kAutoTxMaxDeltaDeg,
                               kAutoTxMaxDeltaDeg);
                bool clamped = std::abs(raw_yaw_delta - yaw_delta) > 1e-3 ||
                               std::abs(raw_pitch_delta - pitch_delta) > 1e-3;
                if (clamped) {
                    auto_tx_clamp_streak++;
                    if (auto_tx_clamp_streak == 1 ||
                        auto_tx_clamp_streak % 20 == 0) {
                        std::cerr << cv::format(
                            "[WARN] 限幅发送: target=(%.1f,%.1f) state=(%.1f,%.1f) raw_delta=(%.1f,%.1f) tx_delta=(%.1f,%.1f)\n",
                            target_yaw, target_pitch,
                            gimbal_state.yaw, gimbal_state.pitch,
                            raw_yaw_delta, raw_pitch_delta,
                            yaw_delta, pitch_delta);
                    }
                } else {
                    auto_tx_clamp_streak = 0;
                }
                cmd.yaw = static_cast<float>(gimbal_state.yaw + yaw_delta);
                cmd.pitch =
                    static_cast<float>(gimbal_state.pitch + pitch_delta);
            } else {
                cmd.mode = static_cast<uint8_t>(common::GimbalCommandMode::Control);
                auto_tx_clamp_streak = 0;
                double raw_yaw_delta =
                    angle_diff_deg(target_yaw, gimbal_state.yaw);
                double raw_pitch_delta =
                    angle_diff_deg(gimbal_state.pitch, target_pitch);
                double yaw_delta =
                    std::clamp(raw_yaw_delta, -kManualTxMaxDeltaDeg,
                               kManualTxMaxDeltaDeg);
                double pitch_delta =
                    std::clamp(raw_pitch_delta, -kManualTxMaxDeltaDeg,
                               kManualTxMaxDeltaDeg);
                bool clamped = std::abs(raw_yaw_delta - yaw_delta) > 1e-3 ||
                               std::abs(raw_pitch_delta - pitch_delta) > 1e-3;
                if (clamped) {
                    manual_tx_clamp_streak++;
                    if (manual_tx_clamp_streak == 1 ||
                        manual_tx_clamp_streak % 20 == 0) {
                        std::cerr << cv::format(
                            "[WARN] 手动限幅发送: target=(%.1f,%.1f) state=(%.1f,%.1f) raw_delta=(%.1f,%.1f) tx_delta=(%.1f,%.1f)\n",
                            target_yaw, target_pitch,
                            gimbal_state.yaw, gimbal_state.pitch,
                            raw_yaw_delta, raw_pitch_delta,
                            yaw_delta, pitch_delta);
                    }
                } else {
                    manual_tx_clamp_streak = 0;
                }
                cmd.yaw = static_cast<float>(gimbal_state.yaw + yaw_delta);
                cmd.pitch =
                    static_cast<float>(gimbal_state.pitch + pitch_delta);
            }
            uint8_t tx[gimbal_serial::kTxFrameSize]{};
            gimbal_serial::packGimbalCommand(cmd, gimbal_state, tx);
            serial.write(tx, gimbal_serial::kTxFrameSize);
        }

        // ── 判断当前步骤 ──
        int step;
        if (last_result.valid && current_Z <= 0.01)
            step = 5;
        else if (current_Z > 0.01)
            step = 3;
        else if ((int)samples.size() >= 3)
            step = 4;
        else if (dot.x >= 0)
            step = 2;
        else
            step = 1;

        // ── UI 辅助 lambda ──
        int panel_w = 500, panel_pad = 16;
        int panel_x = vis.cols - panel_w - 8;
        auto drawBar = [&](int x, int y, int w, int h, double progress, cv::Scalar fg, cv::Scalar bg) {
            cv::rectangle(vis, cv::Rect(x, y, w, h), bg, -1);
            int fill = std::clamp((int)(w * progress), 0, w);
            if (fill > 0) cv::rectangle(vis, cv::Rect(x, y, fill, h), fg, -1);
            cv::rectangle(vis, cv::Rect(x, y, w, h), cv::Scalar(80, 80, 80), 1);
        };
        auto drawSep = [&](int y) {
            cv::line(vis, cv::Point(panel_x + panel_pad, y),
                     cv::Point(panel_x + panel_w - panel_pad, y), cv::Scalar(60, 60, 60), 1);
        };
        auto drawDot = [&](int x, int y, cv::Scalar color) {
            cv::circle(vis, cv::Point(x, y), 5, color, -1, cv::LINE_AA);
        };
        auto stepIcon = [&](int s, int x, int y) {
            if (s < step) {
                // 已完成: 绿色勾
                drawDot(x, y, cv::Scalar(50, 180, 50));
                cv::line(vis, cv::Point(x-3, y), cv::Point(x-1, y+3), cv::Scalar(255,255,255), 2, cv::LINE_AA);
                cv::line(vis, cv::Point(x-1, y+3), cv::Point(x+4, y-3), cv::Scalar(255,255,255), 2, cv::LINE_AA);
            } else if (s == step) {
                // 当前: 蓝色圆环 + 脉冲
                cv::circle(vis, cv::Point(x, y), 6, cv::Scalar(255, 180, 0), 2, cv::LINE_AA);
                cv::circle(vis, cv::Point(x, y), 3, cv::Scalar(255, 180, 0), -1, cv::LINE_AA);
            } else {
                // 未到: 灰色空圆
                cv::circle(vis, cv::Point(x, y), 5, cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
            }
        };

        // ── 计算面板高度 ──
        int panel_h = 420;  // 基础
        if (auto_state != AutoState::IDLE) panel_h += 70;
        if (step == 3) panel_h += 50;
        if (step == 5 || last_result.valid) panel_h += 40;
        if (!samples.empty()) panel_h += (int)std::min((int)samples.size(), 6) * 22 + 30;
        panel_h = std::min(panel_h, vis.rows - 16);

        // ── 面板背景 (圆角模拟 + 磨砂效果) ──
        {
            cv::Rect panelRect(panel_x, 6, panel_w, panel_h);
            panelRect &= cv::Rect(0, 0, vis.cols, vis.rows);
            cv::Mat roi = vis(panelRect);
            cv::Mat blurred;
            cv::GaussianBlur(roi, blurred, cv::Size(15, 15), 5);
            cv::addWeighted(blurred, 0.7, cv::Mat(roi.size(), roi.type(), cv::Scalar(15, 15, 20)), 0.3, 0, roi);
        }
        // 边框
        cv::rectangle(vis, cv::Rect(panel_x, 6, panel_w, panel_h),
                      cv::Scalar(70, 70, 80), 1, cv::LINE_AA);
        // 顶部高亮线
        cv::line(vis, cv::Point(panel_x + 1, 7), cv::Point(panel_x + panel_w - 1, 7),
                 cv::Scalar(100, 140, 200), 1);

        int py = 28;
        // ── 标题栏 ──
        ft2->putText(vis, "激光同轴校准", cv::Point(panel_x + panel_pad, py), 26,
                     cv::Scalar(240, 240, 255), -1, cv::LINE_AA, true);
        // 右上角显示模式标签
        {
            std::string mode_str = (auto_state != AutoState::IDLE) ? "自动" : "手动";
            cv::Scalar mode_col = (auto_state != AutoState::IDLE) ? cv::Scalar(0, 220, 220) : cv::Scalar(150, 150, 150);
            int mode_x = panel_x + panel_w - panel_pad - 55;
            cv::rectangle(vis, cv::Rect(mode_x - 4, py - 14, 62, 20), mode_col, -1);
            ft2->putText(vis, mode_str, cv::Point(mode_x + 6, py - 1), 14,
                         cv::Scalar(0, 0, 0), -1, cv::LINE_AA, true);
        }
        py += 30;

        // ── 自动模式状态卡片 ──
        if (auto_state == AutoState::PREPARE) {
            double settle_progress = 0.0;
            if (auto_prepare_stable_since_ms > 0) {
                settle_progress = std::min(
                    1.0, (common::nowMs() - auto_prepare_stable_since_ms) /
                             static_cast<double>(kPrepareStableMs));
            }
            cv::Rect card(panel_x + panel_pad - 4, py - 4,
                          panel_w - 2 * panel_pad + 8, 40);
            card &= cv::Rect(0, 0, vis.cols, vis.rows);
            cv::rectangle(vis, card, cv::Scalar(40, 50, 60), -1);
            cv::rectangle(vis, card, cv::Scalar(0, 180, 220), 1);
            ft2->putText(vis,
                         cv::format("准备中... 回零稳定 %.1fs", kPrepareStableMs / 1000.0),
                         cv::Point(panel_x + panel_pad + 4, py + 14), 18,
                         cv::Scalar(100, 220, 255), -1, cv::LINE_AA, true);
            drawBar(panel_x + panel_pad + 4, py + 24,
                    panel_w - 2 * panel_pad - 12, 6, settle_progress,
                    cv::Scalar(0, 180, 220), cv::Scalar(30, 30, 30));
            py += 44;
        }
        if (auto_state == AutoState::SPIRAL) {
            double r_now = auto_spiral.fermat_a * std::sqrt(auto_theta);
            cv::Rect card(panel_x + panel_pad - 4, py - 4,
                          panel_w - 2 * panel_pad + 8, 40);
            card &= cv::Rect(0, 0, vis.cols, vis.rows);
            cv::rectangle(vis, card, cv::Scalar(40, 50, 60), -1);
            cv::rectangle(vis, card, cv::Scalar(0, 180, 180), 1);
            ft2->putText(vis, cv::format("螺旋扫描  r=%.1f°  %d样本", r_now, (int)samples.size()),
                         cv::Point(panel_x + panel_pad + 4, py + 14), 18,
                         cv::Scalar(100, 255, 100), -1, cv::LINE_AA, true);
            drawBar(panel_x + panel_pad + 4, py + 24,
                    panel_w - 2 * panel_pad - 12, 6,
                    r_now / auto_spiral.max_deg,
                    cv::Scalar(0, 200, 200), cv::Scalar(30, 30, 30));
            py += 44;
        }
        if (auto_state == AutoState::RETURNING) {
            double dist = std::sqrt(auto_offset_yaw * auto_offset_yaw + auto_offset_pitch * auto_offset_pitch);
            cv::Rect card(panel_x + panel_pad - 4, py - 4,
                          panel_w - 2 * panel_pad + 8, 28);
            card &= cv::Rect(0, 0, vis.cols, vis.rows);
            cv::rectangle(vis, card, cv::Scalar(40, 50, 60), -1);
            cv::rectangle(vis, card, cv::Scalar(0, 150, 220), 1);
            ft2->putText(vis, cv::format("回正中... 剩余%.1f°", dist),
                         cv::Point(panel_x + panel_pad + 4, py + 14), 18,
                         cv::Scalar(100, 200, 255), -1, cv::LINE_AA, true);
            py += 32;
        }

        drawSep(py); py += 8;

        // ── 步骤列表 ──
        auto stepColor = [&](int s) -> cv::Scalar {
            if (s == step) return cv::Scalar(220, 220, 255);
            if (s < step)  return cv::Scalar(120, 120, 120);
            return cv::Scalar(60, 60, 60);
        };

        auto drawStep = [&](int s, const std::string& text, const std::string& hint = "") {
            stepIcon(s, panel_x + panel_pad + 6, py + 2);
            ft2->putText(vis, text, cv::Point(panel_x + panel_pad + 20, py + 6), 18, stepColor(s), -1, cv::LINE_AA, true);
            py += 24;
            if (!hint.empty() && s == step) {
                ft2->putText(vis, hint, cv::Point(panel_x + panel_pad + 20, py + 2), 14,
                             cv::Scalar(140, 140, 200), -1, cv::LINE_AA, true);
                py += 18;
            }
        };

        drawStep(1, "找到激光点", "WASD=云台  e=曝光  +/-=阈值");
        drawStep(2, "设距离 [1-6]", current_Z > 0.01 ? cv::format("当前 %.1fm", current_Z) : "按数字键选距离");
        drawStep(3, "采集数据 [c=保存]",
                 (step == 3) ? cv::format("%.2fm  %d帧  %+.1fmm / %+.1fmm", current_Z, collect_count,
                                          live_dx * 1000, live_dy * 1000) : "");
        drawStep(4, cv::format("拟合计算 [f]  (%d样本)", (int)samples.size()), "");
        drawStep(5, "保存结果 [r]", "");

        // ── 采集中: 实时进度条 (手动模式) ──
        if (step == 3 && collect_count > 0 && auto_state == AutoState::IDLE) {
            int bar_y = py;
            drawBar(panel_x + panel_pad, bar_y, panel_w - 2 * panel_pad, 10,
                    std::min(1.0, (double)collect_count / 100.0),
                    cv::Scalar(80, 200, 80), cv::Scalar(30, 30, 30));
            ft2->putText(vis, cv::format("%d 帧", collect_count),
                         cv::Point(panel_x + panel_w - panel_pad - 60, bar_y + 8), 12,
                         cv::Scalar(180, 180, 180), -1, cv::LINE_AA, true);
            py += 18;
        }

        // ── 螺旋扫描进度 ──
        if (auto_state == AutoState::SPIRAL || auto_state == AutoState::MANUAL_CAPTURE) {
            py += 4;
            drawSep(py); py += 8;
            // 样本数和偏移角度
            std::string progress_text;
            if (auto_state == AutoState::MANUAL_CAPTURE) {
                progress_text = cv::format("%d样本  手动采集 %d/%d帧",
                                           (int)samples.size(),
                                           manual_pass_frames,
                                           kManualPassFrames);
            } else {
                progress_text = cv::format("%d样本  偏移 %+.1f°, %+.1f°",
                                           (int)samples.size(),
                                           auto_offset_yaw, auto_offset_pitch);
            }
            ft2->putText(vis, progress_text,
                         cv::Point(panel_x + panel_pad, py + 4), 14,
                         cv::Scalar(160, 200, 200), -1, cv::LINE_AA, true);
            py += 20;
            if (auto_base_dist > 0.01 && auto_state == AutoState::SPIRAL) {
                double yaw_rad = auto_offset_yaw * M_PI / 180.0;
                double pitch_rad = auto_offset_pitch * M_PI / 180.0;
                double Z_now = auto_base_dist / (std::cos(yaw_rad) * std::cos(pitch_rad));
                ft2->putText(vis, cv::format("Z=%.3fm (基准%.2fm)", Z_now, auto_base_dist),
                             cv::Point(panel_x + panel_pad, py + 4), 14,
                             cv::Scalar(160, 200, 200), -1, cv::LINE_AA, true);
                py += 20;
            }
        }

        // ── 拟合结果卡片 ──
        if (last_result.valid) {
            py += 4;
            drawSep(py); py += 8;
            cv::Rect resCard(panel_x + panel_pad - 4, py - 2,
                             panel_w - 2 * panel_pad + 8, 48);
            resCard &= cv::Rect(0, 0, vis.cols, vis.rows);
            cv::rectangle(vis, resCard, cv::Scalar(30, 45, 30), -1);
            cv::rectangle(vis, resCard, cv::Scalar(60, 180, 60), 1);
            ft2->putText(vis, cv::format("dx=%+.1fmm  dy=%+.1fmm", last_result.dx * 1000, last_result.dy * 1000),
                         cv::Point(panel_x + panel_pad + 4, py + 14), 18,
                         cv::Scalar(100, 255, 100), -1, cv::LINE_AA, true);
            ft2->putText(vis, cv::format("误差: u=%.1fpx  v=%.1fpx", last_result.fit_error_u, last_result.fit_error_v),
                         cv::Point(panel_x + panel_pad + 4, py + 34), 14,
                         cv::Scalar(180, 220, 180), -1, cv::LINE_AA, true);
            py += 54;
        }

        // ── 样本列表 ──
        if (!samples.empty()) {
            py += 4;
            drawSep(py); py += 8;
            ft2->putText(vis, cv::format("样本 (%d)", (int)samples.size()),
                         cv::Point(panel_x + panel_pad, py + 4), 16,
                         cv::Scalar(180, 180, 180), -1, cv::LINE_AA, true);
            py += 22;
            for (int i = 0; i < (int)samples.size() && i < 6; i++) {
                drawDot(panel_x + panel_pad + 6, py + 2, cv::Scalar(0, 165, 255));
                ft2->putText(vis, cv::format("#%d  %.2fm  A=%.0f", i + 1, samples[i].Z, samples[i].dot_area),
                             cv::Point(panel_x + panel_pad + 18, py + 6), 15,
                             cv::Scalar(80, 190, 255), -1, cv::LINE_AA, true);
                py += 20;
            }
            if ((int)samples.size() > 6) {
                ft2->putText(vis, cv::format("...%d more", (int)samples.size() - 6),
                             cv::Point(panel_x + panel_pad + 18, py + 4), 13,
                             cv::Scalar(100, 100, 100), -1, cv::LINE_AA, true);
                py += 18;
            }
        }

        // ── 底部快捷键 ──
        py = 6 + panel_h - 24;
        ft2->putText(vis, "g=自动  z/c/x/f/r  WASD  e  +/-  q=退出",
                     cv::Point(panel_x + panel_pad, py), 13,
                     cv::Scalar(90, 90, 100), -1, cv::LINE_AA, true);

        // ══════════════════════════════════════════════════════════
        // ── 左上角: 状态条 (半透明背景) ──
        {
            int bar_h = (has_gimbal) ? 72 : 38;
            cv::Rect bar_rect(0, 0, std::min(500, panel_x - 16), bar_h);
            bar_rect &= cv::Rect(0, 0, vis.cols, vis.rows);
            cv::Mat bar_roi = vis(bar_rect);
            bar_roi *= 0.4;
        }
        if (dot.x >= 0) {
            // 激光点状态: 绿色圆点 + 坐标 + 面积
            cv::circle(vis, cv::Point(14, 18), 6, cv::Scalar(0, 220, 0), -1, cv::LINE_AA);
            ft2->putText(vis, cv::format("激光 (%.0f, %.0f)  偏移 %+.0f, %+.0f px  面积 %.0f px",
                         dot.x, dot.y, dot.x - cx, dot.y - cy, dot_area),
                         cv::Point(26, 24), 18, cv::Scalar(100, 255, 100), -1, cv::LINE_AA, true);
        } else {
            cv::circle(vis, cv::Point(14, 18), 6, cv::Scalar(0, 0, 200), -1, cv::LINE_AA);
            ft2->putText(vis, "未检测到激光点",
                         cv::Point(26, 24), 18, cv::Scalar(120, 120, 255), -1, cv::LINE_AA, true);
        }
        // 设备状态行
        {
            int sx = 10, sy = 44;
            // 曝光
            std::string exp_str = cv::format("曝光 %.0fus", current_exposure_us);
            cv::Scalar exp_col = (current_exposure_us < 500) ? cv::Scalar(100, 200, 255) : cv::Scalar(150, 150, 150);
            ft2->putText(vis, exp_str, cv::Point(sx, sy), 14, exp_col, -1, cv::LINE_AA, true);
            // 阈值
            ft2->putText(vis, cv::format("阈值 %d", dot_threshold), cv::Point(sx + 130, sy), 14,
                         cv::Scalar(150, 150, 150), -1, cv::LINE_AA, true);
            if (has_gimbal) {
                ft2->putText(vis, cv::format("云台 yaw=%.1f pitch=%.1f",
                             gimbal_state.yaw, gimbal_state.pitch),
                             cv::Point(sx, sy + 22), 14,
                             cv::Scalar(150, 180, 200), -1, cv::LINE_AA, true);
            }
        }

        // ── 画已保存样本在图像上 ──
        for (int i = 0; i < (int)samples.size(); i++) {
            cv::circle(vis, samples[i].dot, 8, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
            cv::circle(vis, samples[i].dot, 2, cv::Scalar(0, 165, 255), -1, cv::LINE_AA);
            ft2->putText(vis, cv::format("#%d %.1fm", i + 1, samples[i].Z),
                         samples[i].dot + cv::Point2f(12, 4), 13,
                         cv::Scalar(80, 190, 255), -1, cv::LINE_AA, true);
        }

        // ── 光点到光心的连线 (辅助对准) ──
        if (dot.x >= 0) {
            cv::line(vis, cv::Point((int)cx, (int)cy), cv::Point((int)dot.x, (int)dot.y),
                     cv::Scalar(0, 200, 0, 100), 1, cv::LINE_AA);
        }

        // ── 左下角: 操作指南 (始终可见) ──
        {
            const int hx = 8, hy_base = vis.rows - 8;
            const int line_h = 20;
            // 根据当前步骤高亮下一步操作
            struct HelpLine { std::string key; std::string desc; bool highlight; };
            std::vector<HelpLine> help;
            help.push_back({"WASD", "控制云台", step == 1});
            help.push_back({"e/t",  "曝光 升高/降低", step == 1});
            help.push_back({"+/-",  "调检测阈值(绿圈)", step == 1 && dot.x < 0});
            help.push_back({"1-6",  "设距离(0.5~5m)", step <= 2});
            help.push_back({"g",    "螺旋扫描(自动)", current_Z > 0.01 && auto_state == AutoState::IDLE});
            help.push_back({"c",    "保存样本(手动)", step == 3});
            help.push_back({"f",    "拟合计算", (int)samples.size() >= 2});
            help.push_back({"r",    "保存到 control.yaml", last_result.valid});
            help.push_back({"空格", "停止采集→拟合",
                            auto_state == AutoState::SPIRAL ||
                            auto_state == AutoState::MANUAL_CAPTURE});
            help.push_back({"q",    "退出", false});

            int box_h = (int)help.size() * line_h + 30;
            int box_w = 310;
            int box_y = hy_base - box_h;
            cv::Rect helpRect(hx, box_y, box_w, box_h);
            helpRect &= cv::Rect(0, 0, vis.cols, vis.rows);
            cv::Mat helpRoi = vis(helpRect);
            helpRoi *= 0.3;
            cv::rectangle(vis, helpRect, cv::Scalar(80, 80, 80), 1);

            int ty = box_y + 20;
            ft2->putText(vis, "操作指南", cv::Point(hx + 8, ty), 16,
                         cv::Scalar(220, 220, 240), -1, cv::LINE_AA, true);
            ty += line_h + 4;

            for (const auto& h : help) {
                cv::Scalar keyCol = h.highlight ? cv::Scalar(0, 255, 255) : cv::Scalar(160, 160, 160);
                cv::Scalar descCol = h.highlight ? cv::Scalar(200, 255, 200) : cv::Scalar(120, 120, 120);
                ft2->putText(vis, h.key, cv::Point(hx + 10, ty), 15, keyCol, -1, cv::LINE_AA, true);
                ft2->putText(vis, h.desc, cv::Point(hx + 55, ty), 15, descCol, -1, cv::LINE_AA, true);
                if (h.highlight) {
                    // 箭头标记当前步骤
                    cv::arrowedLine(vis, cv::Point(hx + 3, ty - 4), cv::Point(hx + 7, ty - 4),
                                    cv::Scalar(0, 255, 255), 2, cv::LINE_AA, 0, 0.4);
                }
                ty += line_h;
            }
        }

        // 多距离提示: 在画面中央显示醒目提示
        if ((auto_state == AutoState::MULTI_PROMPT ||
             auto_state == AutoState::PREPARE) && multi_mode) {
            int total = kMultiDistN * kMultiRep;
            int scan_idx = multi_dist_idx * kMultiRep + multi_repeat + 1;
            auto msg1 = cv::format("[%d/%d]  %.1fm", scan_idx, total, kMultiDist[multi_dist_idx]);
            std::string msg2 =
                (auto_state == AutoState::PREPARE)
                    ? cv::format("Settling at 0,0 for %.1fs", kPrepareStableMs / 1000.0)
                    : std::string("Press SPACE when ready");
            int font = cv::FONT_HERSHEY_SIMPLEX;
            // 半透明背景
            cv::Mat overlay = vis.clone();
            cv::rectangle(overlay, cv::Rect(vis.cols/2-220, vis.rows/2-60, 440, 120),
                          cv::Scalar(0,0,0), -1);
            cv::addWeighted(overlay, 0.7, vis, 0.3, 0, vis);
            cv::putText(vis, msg1, cv::Point(vis.cols/2-180, vis.rows/2-10),
                        font, 1.5, cv::Scalar(0,255,255), 3, cv::LINE_AA);
            cv::putText(vis, msg2, cv::Point(vis.cols/2-180, vis.rows/2+40),
                        font, 0.8, cv::Scalar(200,200,200), 2, cv::LINE_AA);
        }

        cv::imshow("Laser Calib", vis);
        int key = cv::waitKey(1) & 0xFF;
        // 也从 stdin 读命令 (供 PyQt5 GUI 通过管道发送)
        if (key == 0xFF) {
            struct pollfd pfd = {STDIN_FILENO, POLLIN, 0};
            if (poll(&pfd, 1, 0) > 0) {
                char c;
                if (::read(STDIN_FILENO, &c, 1) == 1)
                    key = static_cast<int>(c) & 0xFF;
            }
        }

        if (key == 'q' || key == 27) break;

        // ── 曝光调节: e=升高(亮) t=降低(暗) ──
        if (key == 'e' && use_galaxy) {
            current_exposure_us = std::min(50000.0, current_exposure_us * 1.5);
            galaxy.setExposure(current_exposure_us);
            std::cout << cv::format("曝光 → %.0f us\n", current_exposure_us);
        }
        if (key == 't' && use_galaxy) {
            current_exposure_us = std::max(50.0, current_exposure_us / 1.5);
            galaxy.setExposure(current_exposure_us);
            std::cout << cv::format("曝光 → %.0f us\n", current_exposure_us);
        }

        // ── 检测阈值 ──
        if (key == '+' || key == '=') {
            dot_threshold = std::min(255, dot_threshold + 5);
            std::cout << "阈值 → " << dot_threshold << "\n";
        }
        if (key == '-') {
            dot_threshold = std::max(20, dot_threshold - 5);
            std::cout << "阈值 → " << dot_threshold << "\n";
        }

        // ── 云台手动控制 (WASD) ── 更新持久目标, 由每帧统一发送
        if (has_gimbal &&
            (auto_state == AutoState::IDLE ||
             auto_state == AutoState::MULTI_PROMPT ||
             auto_state == AutoState::MANUAL_CAPTURE) &&
            manual_init) {
            float step_deg = manual_step_deg(current_Z > 0.01 ? current_Z : auto_base_dist);
            if (key == '0') {
                manual_yaw = 0.0f;
                manual_pitch = 0.0f;
            }
            if (key == 'w') manual_pitch += step_deg;
            if (key == 's') manual_pitch -= step_deg;
            if (key == 'a') manual_yaw += step_deg;
            if (key == 'd') manual_yaw -= step_deg;
            if (key == '0' || key == 'w' || key == 's' || key == 'a' || key == 'd') {
                prompt_idle_frames = 0;
                if (auto_state == AutoState::MULTI_PROMPT) {
                    prompt_manual_control = true;
                }
                if (auto_state == AutoState::MANUAL_CAPTURE) {
                    manual_capture_pause_frames = 8;
                }
                if (key == '0') {
                    std::cout << cv::format("[手动] 对正前方 target=(%.1f,%.1f) state=(%.1f,%.1f)\n",
                                            manual_yaw, manual_pitch,
                                            has_gimbal_state ? gimbal_state.yaw : 0.0f,
                                            has_gimbal_state ? gimbal_state.pitch : 0.0f)
                              << std::flush;
                } else {
                    std::cout << cv::format("[手动] step=%.2f target=(%.1f,%.1f) state=(%.1f,%.1f)\n",
                                            step_deg,
                                            manual_yaw, manual_pitch,
                                            has_gimbal_state ? gimbal_state.yaw : 0.0f,
                                            has_gimbal_state ? gimbal_state.pitch : 0.0f)
                              << std::flush;
                }
            }
        }

        // ── 数字键设距离: 1=0.5m 2=1m 3=1.5m 4=2m 5=3m 6=5m ──
        {
            constexpr double kDistPresets[] = {0, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0};
            int digit = key - '0';
            if (digit >= 1 && digit <= 6) {
                auto_state = AutoState::IDLE;
                current_Z = kDistPresets[digit];
                sum_u = sum_v = 0; collect_count = 0;
                live_dx = live_dy = 0;
                std::cout << cv::format("距离=%.1fm  (按g扫描 / WASD手动+c保存)\n", current_Z);
            }
        }

        // ── 多距离引导标定 (g=启动) ──
        if (key == 'g' && has_gimbal) {
            if (last_rx_ms <= 0) {
                std::cerr << "[ERR] 尚未收到云台回传, 请检查串口连接\n";
                continue;
            }
            clearSession(session_path);
            samples.clear();
            last_result = CalibResult{};
            multi_mode = true;
            multi_dist_idx = 0;
            multi_repeat = 0;
            retry_current_scan = false;
            auto_state = AutoState::MULTI_PROMPT;
            prompt_idle_frames = 0;
            prompt_manual_control = false;
            r_saved = false;
            int total = kMultiDistN * kMultiRep;
            std::cout << cv::format("\n[多距离标定] %d 个距离 × %d 遍 = %d 次扫描\n",
                                    kMultiDistN, kMultiRep, total);
            std::cout << "  距离:";
            for (int i = 0; i < kMultiDistN; i++)
                std::cout << cv::format(" %.1fm", kMultiDist[i]);
            std::cout << "\n";
            std::cout << cv::format("[1/%d] 请移到 %.1fm 处, 按空格开始\n",
                                    total, kMultiDist[0]) << std::flush;
            save_multi_session(0, 0);
        }
        // MULTI_PROMPT: 用户按空格确认已就位, 开始扫描
        if (key == ' ' && auto_state == AutoState::MULTI_PROMPT) {
            double D = kMultiDist[multi_dist_idx];
            auto_base_dist = D;
            auto_theta = 0;
            auto_start_yaw = has_gimbal_state ? gimbal_state.yaw : manual_yaw;
            auto_start_pitch = has_gimbal_state ? gimbal_state.pitch : manual_pitch;
            auto_offset_yaw = 0;
            auto_offset_pitch = 0;
            auto_spiral = select_spiral_profile(multi_repeat);
            auto_prepare_stable_since_ms = 0;
            retry_current_scan = false;
            auto_pass_sample_begin = samples.size();
            auto_track_bad_frames = 0;
            prompt_idle_frames = 0;
            prompt_manual_control = false;
            current_Z = D;
            int total = kMultiDistN * kMultiRep;
            int scan_idx = multi_dist_idx * kMultiRep + multi_repeat;
            auto_pass_manual_capture = (multi_dist_idx >= 1);
            manual_pass_frames = 0;
            manual_capture_pause_frames = 0;
            if (auto_pass_manual_capture) {
                manual_yaw = static_cast<float>(auto_start_yaw);
                manual_pitch = static_cast<float>(auto_start_pitch);
                auto_state = AutoState::MANUAL_CAPTURE;
                std::cout << cv::format(
                    "[%d/%d] %.1fm 第%d遍手动采集中, 可WASD微调, 稳定后采满%d帧  起始角: yaw=%.1f° pitch=%.1f°\n",
                    scan_idx + 1, total, D, multi_repeat + 1,
                    kManualPassFrames, auto_start_yaw, auto_start_pitch)
                          << std::flush;
            } else {
                auto_state = AutoState::SPIRAL;
                std::cout << cv::format(
                    "[%d/%d] %.1fm 第%d遍开始, 起始角: yaw=%.1f° pitch=%.1f°  扫描R=%.1f° step=%.2f°\n",
                    scan_idx + 1, total, D, multi_repeat + 1,
                    auto_start_yaw, auto_start_pitch, auto_spiral.max_deg,
                    auto_spiral.step_deg) << std::flush;
            }
        } else if (key == ' ' && (auto_state == AutoState::SPIRAL ||
                                  auto_state == AutoState::MANUAL_CAPTURE ||
                                  auto_state == AutoState::RETURNING)) {
            if (auto_state == AutoState::SPIRAL ||
                auto_state == AutoState::MANUAL_CAPTURE) {
                if (auto_state == AutoState::MANUAL_CAPTURE) {
                    std::cout << cv::format("[手动采集] 提前结束, %d 帧, 开始拟合...\n",
                                            manual_pass_frames) << std::flush;
                } else {
                    std::cout << "[螺旋] 手动停止, 开始拟合...\n";
                }
                auto_state = AutoState::DONE;
            } else {
                auto_offset_yaw = 0; auto_offset_pitch = 0;
                auto_state = AutoState::IDLE;
                manual_yaw = gimbal_state.yaw;
                manual_pitch = gimbal_state.pitch;
                std::cout << "[螺旋] 跳过回正\n";
            }
        }

        // ── 保存当前距离的平均值 ──
        if (key == 'c') {
            if (current_Z <= 0.01) {
                std::cerr << "先按数字键(1-6)设距离\n";
                continue;
            }
            if (collect_count < 5) {
                std::cerr << cv::format("帧数太少 (%d), 等激光点稳定几秒再按 'c'\n", collect_count);
                continue;
            }
            double avg_u = sum_u / collect_count;
            double avg_v = sum_v / collect_count;
            samples.push_back({current_Z, cv::Point2f(avg_u, avg_v), (double)collect_count});
            std::cout << cv::format("  + 样本 #%d: Z=%.2fm  avg=(%.1f, %.1f)  %d frames  dx=%+.1fmm dy=%+.1fmm\n",
                                    (int)samples.size(), current_Z, avg_u, avg_v, collect_count,
                                    live_dx * 1000, live_dy * 1000);
            // 重置, 等待下一个距离
            current_Z = 0;
            sum_u = sum_v = 0;
            collect_count = 0;
        }

        // ── 删除样本 ──
        if (key == 'x' && !samples.empty()) {
            std::cout << cv::format("  - 删除样本 #%d\n", (int)samples.size());
            samples.pop_back();
        }
        if (key == 'X') {
            std::cout << cv::format("[清空] 删除全部 %d 个样本\n", (int)samples.size());
            samples.clear();
            last_result = CalibResult{};
            clearSession(session_path);
        }

        // ── 拟合 ──
        if (key == 'f') {
            if (samples.size() < 2) {
                std::cerr << "至少需要 2 个样本 (建议 3+)\n";
                continue;
            }
            pruneDistanceOutliers(&samples);
            last_result = fitCalibration(samples, fx, fy, cx, cy);
            if (last_result.valid) {
                printResult(last_result);
                std::cout << "按 'r' 保存到 " << control_config << "\n";
            }
        }

        // ── 保存 ──
        if (key == 'r' && last_result.valid && !r_saved) {
            r_saved = true;
            std::ifstream ifs(control_config);
            if (!ifs.is_open()) {
                std::cerr << "无法打开 " << control_config << "\n";
                continue;
            }
            std::string content((std::istreambuf_iterator<char>(ifs)),
                                 std::istreambuf_iterator<char>());
            ifs.close();

            auto replaceYamlValue = [](std::string& text,
                                       const std::string& key, double value) {
                auto pos = text.find(key + ":");
                if (pos == std::string::npos) return;
                auto line_end = text.find('\n', pos);
                auto colon = text.find(':', pos);
                auto comment_pos = text.find('#', colon + 1);
                std::string comment;
                if (comment_pos != std::string::npos && comment_pos < line_end) {
                    comment = text.substr(comment_pos, line_end - comment_pos);
                }
                std::string new_line = key + ": " + cv::format("%.6f", value);
                if (!comment.empty()) new_line += "    " + comment;
                text.replace(pos, line_end - pos, new_line);
            };

            replaceYamlValue(content, "laser_offset_y", last_result.dy);
            replaceYamlValue(content, "laser_offset_x", last_result.dx);

            // 角度偏差直接保存为度 (controller 直接用, 不需要像素中间量)
            if (content.find("boresight_yaw_deg:") != std::string::npos) {
                replaceYamlValue(content, "boresight_yaw_deg", last_result.alpha_deg);
            } else {
                content += cv::format("\nboresight_yaw_deg: %.6f    # 度, 激光-相机 yaw 角度偏差, 标定自动生成\n", last_result.alpha_deg);
            }
            if (content.find("boresight_pitch_deg:") != std::string::npos) {
                replaceYamlValue(content, "boresight_pitch_deg", last_result.beta_deg);
            } else {
                content += cv::format("boresight_pitch_deg: %.6f    # 度, 激光-相机 pitch 角度偏差, 标定自动生成\n", last_result.beta_deg);
            }

            std::ofstream ofs(control_config);
            ofs << content;
            ofs.close();
            clearSession(session_path);

            std::cout << "Saved to " << control_config << "\n"
                      << "  laser_offset_x: " << last_result.dx << " m\n"
                      << "  laser_offset_y: " << last_result.dy << " m\n"
                      << "  boresight_yaw_deg: " << last_result.alpha_deg << " deg\n"
                      << "  boresight_pitch_deg: " << last_result.beta_deg << " deg\n";
        }
    }

    // 保存当前曝光值, 下次启动自动加载
    {
        std::string exp_file = camera_config.substr(0, camera_config.rfind('/') + 1) + ".boresight_exposure";
        std::ofstream ofs(exp_file);
        if (ofs.is_open()) {
            ofs << current_exposure_us;
            std::cout << cv::format("曝光 %.0f us 已保存\n", current_exposure_us);
        }
    }
    // 恢复正常曝光
    if (use_galaxy) {
        galaxy.setExposure(3000.0);
    }

    cv::destroyAllWindows();
    return 0;
}
