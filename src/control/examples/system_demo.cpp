#include "common/tee_buf.h"
#include "common/time_utils.h"
#include "control/config.h"
#include "control/controller.h"
#include "measurement.h"
#include "cascade_detector.h"
#include "gimbal_serial/protocol.h"
#include "gimbal_serial/serial_port.h"
#include "galaxy_camera/galaxy_camera.h"

#include <atomic>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <deque>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <thread>

namespace {
constexpr int kDisplayWidth = 1280;
constexpr int kDisplayHeight = 720;
constexpr int kPlotWidth = 640;
constexpr int kPlotHeight = 240;
constexpr int kPanelPadding = 20;
constexpr int kPanelWidth = kPlotWidth * 2 + kPanelPadding;
constexpr int kPanelHeight = kDisplayHeight + kPlotHeight * 2 + kPanelPadding * 2;
constexpr size_t kHistorySize = 200;
constexpr double kJumpThreshPx = 250.0;
constexpr double kStateJumpDeg = 5.0;
constexpr int64_t kDebugLogIntervalMs = 500;
constexpr int64_t kEpochMsThreshold = 1000000000000LL;
constexpr int64_t kStateFreshTimeoutMs = 200;

struct SharedMeasurement {
    std::mutex mu;
    common::TargetMeasurement meas;
    bool has_meas = false;
    int64_t capture_wall_ms = 0;      // 帧采集时的墙钟 (ms)
    int64_t detect_done_wall_ms = 0;  // 检测完成时的墙钟 (ms)
};

struct SharedControlState {
    std::mutex mu;
    common::GimbalCommand cmd;
    common::GimbalState state;
    bool has_cmd = false;
    bool has_state = false;
    bool state_fresh = false;
    bool tx_enabled = false;
    int64_t last_state_rx_ms = 0;
};

using common::TeeBuf;

std::string formatWallTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &now_time_t);
#else
    localtime_r(&now_time_t, &tm);
#endif
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
        << "." << std::setw(3) << std::setfill('0') << ms;
    return oss.str();
}

void printStartupTime() {
    const int64_t epoch_ms = common::nowSystemMs();
    const int64_t mono_ms = common::nowMs();
    std::cout << "startup_time=" << formatWallTimestamp()
              << " epoch_ms=" << epoch_ms
              << " mono_ms=" << mono_ms << "\n";
}

std::string toLowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

bool isPurpleLabel(const std::string& label) {
    return toLowerCopy(label) == "purple";
}

cv::Scalar colorForLabel(const std::string& label) {
    const std::string lowered = toLowerCopy(label);
    if (lowered == "blue") {
        return cv::Scalar(255, 0, 0);
    }
    if (lowered == "red") {
        return cv::Scalar(0, 0, 255);
    }
    if (lowered == "purple") {
        return cv::Scalar(255, 0, 255);
    }
    return cv::Scalar(0, 255, 0);
}

void drawPlot(const std::deque<double>& series_a,
              const std::deque<double>& series_b,
              const std::string& label_a,
              const std::string& label_b,
              cv::Mat* plot) {
    plot->setTo(cv::Scalar(20, 20, 20));
    if (series_a.empty() && series_b.empty()) {
        return;
    }
    double max_val = 1.0;
    for (double v : series_a) {
        max_val = std::max(max_val, std::abs(v));
    }
    for (double v : series_b) {
        max_val = std::max(max_val, std::abs(v));
    }
    int n = static_cast<int>(std::max(series_a.size(), series_b.size()));
    if (n < 2) {
        return;
    }
    auto draw_series = [&](const std::deque<double>& series, const cv::Scalar& color) {
        if (series.size() < 2) {
            return;
        }
        int width = plot->cols;
        int height = plot->rows;
        for (size_t i = 1; i < series.size(); ++i) {
            int x0 = static_cast<int>((i - 1) * (width - 1) / static_cast<double>(n - 1));
            int x1 = static_cast<int>(i * (width - 1) / static_cast<double>(n - 1));
            double v0 = series[i - 1];
            double v1 = series[i];
            int y0 = static_cast<int>(height / 2.0 - (v0 / max_val) * (height / 2.0 - 10));
            int y1 = static_cast<int>(height / 2.0 - (v1 / max_val) * (height / 2.0 - 10));
            cv::line(*plot, cv::Point(x0, y0), cv::Point(x1, y1), color, 2, cv::LINE_AA);
        }
    };
    draw_series(series_a, cv::Scalar(0, 255, 255));
    draw_series(series_b, cv::Scalar(0, 128, 255));
    cv::line(*plot, cv::Point(0, plot->rows / 2), cv::Point(plot->cols, plot->rows / 2),
             cv::Scalar(80, 80, 80), 1, cv::LINE_AA);
    cv::putText(*plot, label_a, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::putText(*plot, label_b, cv::Point(10, 45), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 128, 255), 2, cv::LINE_AA);
}

void drawSinglePlot(const std::deque<double>& series,
                    const std::string& label,
                    cv::Mat* plot) {
    plot->setTo(cv::Scalar(20, 20, 20));
    if (series.size() < 2) {
        return;
    }
    double max_val = 1.0;
    for (double v : series) {
        max_val = std::max(max_val, std::abs(v));
    }
    int n = static_cast<int>(series.size());
    for (size_t i = 1; i < series.size(); ++i) {
        int x0 = static_cast<int>((i - 1) * (plot->cols - 1) / static_cast<double>(n - 1));
        int x1 = static_cast<int>(i * (plot->cols - 1) / static_cast<double>(n - 1));
        double v0 = series[i - 1];
        double v1 = series[i];
        int y0 = static_cast<int>(plot->rows / 2.0 - (v0 / max_val) * (plot->rows / 2.0 - 10));
        int y1 = static_cast<int>(plot->rows / 2.0 - (v1 / max_val) * (plot->rows / 2.0 - 10));
        cv::line(*plot, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
    cv::line(*plot, cv::Point(0, plot->rows / 2), cv::Point(plot->cols, plot->rows / 2),
             cv::Scalar(80, 80, 80), 1, cv::LINE_AA);
    cv::putText(*plot, label, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

void pushHistory(std::deque<double>* series, double value) {
    series->push_back(value);
    if (series->size() > kHistorySize) {
        series->pop_front();
    }
}

std::atomic<bool> g_should_exit{false};

void SignalHandler(int /*signum*/) {
    g_should_exit.store(true);
}
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    std::string camera_config = "../../../config/camera.yaml";
    std::string cascade_config = "../../../config/cascade.yaml";
    std::string enemy_config = "../../../config/enemy.yaml";
    std::string control_config = "../../../config/control.yaml";
    bool is_match_mode = false;
    std::string log_path = "../../log/log.txt";
    std::string port = "/dev/ttyUSB0";
    int baud = 115200;
    int send_hz = 100;
    bool show = true;
    bool window_created = false;
    bool window_ready = false;
    int window_guard = 0;
    bool exit_on_close = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--camera-config" && i + 1 < argc) {
            camera_config = argv[++i];
        } else if (arg == "--cascade-config" && i + 1 < argc) {
            cascade_config = argv[++i];
        } else if (arg == "--enemy" && i + 1 < argc) {
            enemy_config = argv[++i];
            is_match_mode = true;
        } else if (arg == "--control-config" && i + 1 < argc) {
            control_config = argv[++i];
        } else if (arg == "--log-file" && i + 1 < argc) {
            log_path = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--baud" && i + 1 < argc) {
            baud = std::stoi(argv[++i]);
        } else if (arg == "--send-hz" && i + 1 < argc) {
            send_hz = std::stoi(argv[++i]);
        } else if (arg == "--no-show") {
            show = false;
        } else if (arg == "--exit-on-close") {
            exit_on_close = true;
        }
    }

    std::ofstream log_file(log_path, std::ios::app);
    std::streambuf* cout_buf = std::cout.rdbuf();
    std::streambuf* cerr_buf = std::cerr.rdbuf();
    TeeBuf tee_out(cout_buf, log_file.is_open() ? log_file.rdbuf() : cout_buf);
    TeeBuf tee_err(cerr_buf, log_file.is_open() ? log_file.rdbuf() : cerr_buf);
    std::cout.rdbuf(&tee_out);
    std::cerr.rdbuf(&tee_err);
    if (log_file.is_open()) {
        log_file << "\n\n=== " << formatWallTimestamp() << " ===\n";
        log_file.flush();
        std::cout << "Log file (append): " << log_path << "\n";
    } else {
        std::cerr << "Failed to open log file: " << log_path << "\n";
    }
    printStartupTime();

    std::cout << "camera_config=" << camera_config << "\n";
    std::cout << "cascade_config=" << cascade_config << "\n";
    std::cout << "control_config=" << control_config << "\n";
    if (is_match_mode) {
        std::cout << "★ 比赛模式 ★ enemy_config=" << enemy_config << "\n";
    }
    std::cout << std::flush;

    galaxy_camera::CameraConfig cam_cfg;
    if (!galaxy_camera::loadCameraConfig(camera_config, &cam_cfg)) {
        std::cerr << "Failed to load camera config\n";
        return 1;
    }
    std::cout << "cam_cfg.feature_load_enable=" << cam_cfg.feature_load_enable
              << " feature_save_enable=" << cam_cfg.feature_save_enable
              << " feature_save_on_close=" << cam_cfg.feature_save_on_close << "\n";
    std::cout << std::flush;

    detect::CascadeDetector detector;
    if (!detector.loadConfig(cascade_config)) {
        std::cerr << "Failed to load cascade config: " << cascade_config << "\n";
        return 1;
    }
    if (is_match_mode) {
        detector.loadConfig(enemy_config);
    }

    control::ControlConfig ctrl_cfg;
    common::CameraModel cam_model;
    common::Boresight boresight;
    if (!loadControlConfig(control_config, &ctrl_cfg, &cam_model, &boresight, camera_config)) {
        std::cerr << "Failed to load control config\n";
        return 1;
    }
    control::Controller controller(ctrl_cfg);

    galaxy_camera::GalaxyCamera camera;
    if (!camera.open(cam_cfg) || !camera.startGrabbing()) {
        std::cerr << "Failed to open camera\n";
        return 1;
    }
    std::cout << "Camera opened\n" << std::flush;

    gimbal_serial::SerialPort serial;
    if (!serial.open(port, baud)) {
        std::cerr << "Failed to open serial\n";
        return 1;
    }
    std::cout << "Serial opened: " << port << " baud=" << baud << "\n";
    std::cout << std::flush;

    if (show) {
        try {
            cv::namedWindow("control_panel", cv::WINDOW_NORMAL);
            cv::resizeWindow("control_panel", kPanelWidth, kPanelHeight);
            cv::moveWindow("control_panel", 50, 50);
            cv::Mat init_view(kDisplayHeight, kDisplayWidth, CV_8UC3, cv::Scalar(10, 10, 10));
            cv::putText(init_view, "Starting...", cv::Point(40, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            cv::Mat panel(kPanelHeight, kPanelWidth, CV_8UC3, cv::Scalar(10, 10, 10));
            init_view.copyTo(panel(cv::Rect(0, 0, kDisplayWidth, kDisplayHeight)));
            cv::imshow("control_panel", panel);
            cv::waitKey(1);
            window_created = true;
            std::cout << "GUI windows created\n";
            std::cout << std::flush;
        } catch (const cv::Exception& e) {
            std::cerr << "OpenCV GUI not available: " << e.what() << "\n";
            show = false;
        }
    }

    if (ctrl_cfg.startup_check_frames <= 0) {
        std::cout << "DBG init_send_deferred waiting_for_gimbal_feedback\n";
    }

    SharedMeasurement shared_meas;
    SharedControlState shared_ctrl;

    std::thread control_thread([&]() {
        gimbal_serial::FrameParser parser;
        std::vector<uint8_t> rx_buf(512);
        common::GimbalState feedback_state;
        feedback_state.pitch = static_cast<float>(ctrl_cfg.startup_home_pitch);
        feedback_state.yaw = static_cast<float>(ctrl_cfg.startup_home_yaw);
        feedback_state.timestamp = common::nowSystemMs();
        feedback_state.mode = static_cast<uint8_t>(common::GimbalFeedbackMode::Idle);
        common::GimbalState control_state = feedback_state;
        common::GimbalState prev_feedback_state;
        common::GimbalCommand last_cmd;
        common::GimbalCommand last_tx_cmd;
        bool has_feedback_state = false;
        bool has_prev_feedback_state = false;
        bool has_last_cmd = false;
        bool has_last_tx_cmd = false;
        bool feedback_fresh = false;
        int64_t last_feedback_rx_ms = 0;
        int64_t last_debug_ts = 0;
        int64_t last_wait_log_ts = 0;
        int64_t last_meas_ts_used = -1;

        const auto send_period =
            std::chrono::microseconds(send_hz > 0 ? static_cast<int>(1000000 / send_hz) : 20000);
        auto next_send = std::chrono::steady_clock::now();
        auto last_loop_ts = next_send;
        auto last_log = next_send;
        double loop_dt_ms_sum = 0.0;
        int loop_dt_count = 0;
        double meas_age_ms_sum = 0.0;
        int meas_age_count = 0;
        double serial_age_ms_sum = 0.0;
        int serial_age_count = 0;
        int send_count = 0;
        double cmd_step_pitch_sum = 0.0;
        double cmd_step_yaw_sum = 0.0;
        double cmd_step_pitch_max = 0.0;
        double cmd_step_yaw_max = 0.0;
        int cmd_step_count = 0;
        int rate_limit_hit_pitch = 0;
        int rate_limit_hit_yaw = 0;
        uint64_t hold_count = 0;
        // 延迟链统计
        double e2e_sum = 0.0;           // 端到端 (capture→ctrl) 总和
        double cap_to_det_sum = 0.0;    // capture→detect 总和
        double det_to_ctrl_sum = 0.0;   // detect→ctrl 总和
        int e2e_count = 0;
        double e2e_max = 0.0;
        auto last_send_ts = next_send;
        bool has_last_send_ts = false;

        while (!g_should_exit.load()) {
            auto loop_now = std::chrono::steady_clock::now();
            loop_dt_ms_sum += std::chrono::duration<double, std::milli>(loop_now - last_loop_ts).count();
            loop_dt_count++;
            last_loop_ts = loop_now;

            int n = serial.read(rx_buf.data(), static_cast<int>(rx_buf.size()), 2);
            if (n > 0) {
                gimbal_serial::GimbalState state;
                if (parser.push(rx_buf.data(), static_cast<size_t>(n), &state)) {
                    auto invalid_state = [](float v) {
                        return !std::isfinite(v) || std::fpclassify(v) == FP_SUBNORMAL;
                    };
                    if (invalid_state(state.pitch) || invalid_state(state.yaw)) {
                        std::cout << "DBG state_invalid pitch=" << state.pitch
                                  << " yaw=" << state.yaw
                                  << " ts=" << state.timestamp << "\n";
                        continue;
                    }
                    int64_t now_ms = common::nowSystemMs();
                    if (state.timestamp >= kEpochMsThreshold && now_ms >= kEpochMsThreshold) {
                        int64_t age_ms = now_ms - static_cast<int64_t>(state.timestamp);
                        serial_age_ms_sum += static_cast<double>(age_ms);
                        serial_age_count++;
                    }
                    if (has_prev_feedback_state) {
                        double dp = std::abs(state.pitch - prev_feedback_state.pitch);
                        double dy = std::abs(state.yaw - prev_feedback_state.yaw);
                        if (dp > kStateJumpDeg || dy > kStateJumpDeg) {
                            std::cout << "DBG state_jump dp=" << dp << " dy=" << dy
                                      << " pitch=" << state.pitch
                                      << " yaw=" << state.yaw
                                      << " ts=" << state.timestamp << "\n";
                        }
                    }
                    feedback_state = state;
                    feedback_state.timestamp = now_ms;
                    control_state = feedback_state;
                    prev_feedback_state = feedback_state;
                    has_prev_feedback_state = true;
                    has_feedback_state = true;
                    last_feedback_rx_ms = now_ms;
                    {
                        std::lock_guard<std::mutex> lock(shared_ctrl.mu);
                        shared_ctrl.state = control_state;
                        shared_ctrl.has_state = true;
                        shared_ctrl.last_state_rx_ms = last_feedback_rx_ms;
                    }
                }
            }

            auto now = std::chrono::steady_clock::now();
            while (now >= next_send) {
                auto send_now = std::chrono::steady_clock::now();
                double dt_s = 0.0;
                if (has_last_send_ts) {
                    dt_s = std::chrono::duration<double>(send_now - last_send_ts).count();
                }
                last_send_ts = send_now;
                has_last_send_ts = true;

                int64_t now_ms = common::nowMs();
                int64_t now_system_ms = common::nowSystemMs();
                const bool feedback_fresh_now = has_feedback_state && last_feedback_rx_ms > 0 &&
                    (now_system_ms - last_feedback_rx_ms <= kStateFreshTimeoutMs);
                if (feedback_fresh_now != feedback_fresh) {
                    feedback_fresh = feedback_fresh_now;
                    if (feedback_fresh) {
                        std::cout << "DBG gimbal_feedback_ready pitch=" << feedback_state.pitch
                                  << " yaw=" << feedback_state.yaw
                                  << " rx_age_ms=" << (now_system_ms - last_feedback_rx_ms) << "\n";
                        controller.resetRuntimeState();
                        has_last_cmd = false;
                        has_last_tx_cmd = false;
                        last_meas_ts_used = -1;
                    } else {
                        std::cout << "DBG gimbal_feedback_hold pitch=" << control_state.pitch
                                  << " yaw=" << control_state.yaw
                                  << " last_rx_ms=" << last_feedback_rx_ms << "\n";
                    }
                }

                if (feedback_fresh) {
                    control_state = feedback_state;
                } else {
                    if (!has_feedback_state) {
                        control_state.pitch = static_cast<float>(ctrl_cfg.startup_home_pitch);
                        control_state.yaw = static_cast<float>(ctrl_cfg.startup_home_yaw);
                        control_state.quaternion_wxyz = {1.0f, 0.0f, 0.0f, 0.0f};
                        control_state.mode = static_cast<uint8_t>(common::GimbalFeedbackMode::Idle);
                        control_state.bullet_speed = 0.0f;
                        control_state.bullet_count = 0;
                    } else {
                        control_state.pitch = feedback_state.pitch;
                        control_state.yaw = feedback_state.yaw;
                        control_state.quaternion_wxyz = feedback_state.quaternion_wxyz;
                        control_state.mode = static_cast<uint8_t>(common::GimbalFeedbackMode::Idle);
                        control_state.bullet_speed = feedback_state.bullet_speed;
                        control_state.bullet_count = feedback_state.bullet_count;
                    }
                    control_state.pitch_rate = 0.0f;
                    control_state.yaw_rate = 0.0f;
                    control_state.timestamp = now_system_ms;
                }

                common::TargetMeasurement meas_snapshot;
                bool has_meas = false;
                int64_t capture_wall = 0;
                int64_t detect_done_wall = 0;
                {
                    std::lock_guard<std::mutex> lock(shared_meas.mu);
                    if (shared_meas.has_meas) {
                        meas_snapshot = shared_meas.meas;
                        has_meas = true;
                        capture_wall = shared_meas.capture_wall_ms;
                        detect_done_wall = shared_meas.detect_done_wall_ms;
                    }
                }
                if (!has_meas) {
                    meas_snapshot.valid = false;
                    meas_snapshot.timestamp = now_ms;
                    meas_snapshot.uv = cv::Point2f(0.0f, 0.0f);
                    meas_snapshot.confidence = 0.0f;
                }

                common::TargetMeasurement use_meas = meas_snapshot;
                if (!has_meas || !meas_snapshot.valid || meas_snapshot.timestamp <= 0) {
                    use_meas.timestamp = now_ms;
                }

                if (meas_snapshot.timestamp > 0) {
                    meas_age_ms_sum += static_cast<double>(now_ms - meas_snapshot.timestamp);
                    meas_age_count++;
                }
                // 延迟链: capture_wall / detect_done_wall 在串口发送后统计

                common::GimbalCommand cmd;
                const bool has_new_meas = (use_meas.timestamp != last_meas_ts_used);
                if (has_new_meas || !has_last_cmd) {
                    cmd = controller.update(use_meas, cam_model, boresight, control_state);
                    last_meas_ts_used = use_meas.timestamp;  // 只在新测量帧到来时更新控制指令，其它时间保持上一条指令；解决相机帧率低造成的 闭环自激振荡
                    if (!use_meas.valid) {
                        int64_t dbg_ts = common::nowMs();
                        if (dbg_ts - last_debug_ts >= kDebugLogIntervalMs) {
                            std::cout << "DBG meas_invalid hold_state pitch=" << control_state.pitch
                                      << " yaw=" << control_state.yaw
                                      << " ts=" << dbg_ts << "\n";
                            last_debug_ts = dbg_ts;
                        }
                    }
                } else {
                    cmd = last_cmd;
                    cmd.timestamp = common::nowMs();
                }
                cmd.mode = static_cast<uint8_t>(common::GimbalCommandMode::Control);

                bool tx_enabled = false;
                uint8_t packet[gimbal_serial::kTxFrameSize]{};
                gimbal_serial::packGimbalCommand(cmd, control_state, packet);
                if (serial.write(packet, static_cast<int>(sizeof(packet))) >= 0) {
                    tx_enabled = true;
                    send_count++;
                    // ── 精确延迟链测量 (capture → detect → ctrl → serial_write) ──
                    if (capture_wall > 0 && detect_done_wall > 0 && meas_snapshot.valid) {
                        int64_t tx_wall = common::nowMs();
                        double cap_to_det = static_cast<double>(detect_done_wall - capture_wall);
                        double det_to_tx = static_cast<double>(tx_wall - detect_done_wall);
                        double e2e = static_cast<double>(tx_wall - capture_wall);
                        e2e_sum += e2e;
                        cap_to_det_sum += cap_to_det;
                        det_to_ctrl_sum += det_to_tx;
                        e2e_count++;
                        if (e2e > e2e_max) e2e_max = e2e;
                    }
                    if (has_last_tx_cmd && dt_s > 0.0) {
                        double max_step = ctrl_cfg.max_angle_rate * dt_s;
                        double dp = std::abs(cmd.pitch - last_tx_cmd.pitch);
                        double dy = std::abs(cmd.yaw - last_tx_cmd.yaw);
                        cmd_step_pitch_sum += dp;
                        cmd_step_yaw_sum += dy;
                        cmd_step_pitch_max = std::max(cmd_step_pitch_max, dp);
                        cmd_step_yaw_max = std::max(cmd_step_yaw_max, dy);
                        cmd_step_count++;
                        if (max_step > 0.0) {
                            if (dp >= 0.95 * max_step) {
                                rate_limit_hit_pitch++;
                            }
                            if (dy >= 0.95 * max_step) {
                                rate_limit_hit_yaw++;
                            }
                        }
                    }
                    last_tx_cmd = cmd;
                    has_last_tx_cmd = true;
                }
                if (!feedback_fresh) {
                    hold_count++;
                    if (use_meas.valid && now_ms - last_wait_log_ts >= kDebugLogIntervalMs) {
                        std::cout << "DBG gimbal_feedback_wait abs_cmd=(" << cmd.pitch
                                  << "," << cmd.yaw << ")"
                                  << " frozen_state=(" << control_state.pitch
                                  << "," << control_state.yaw << ")"
                                  << " label=" << (use_meas.label.empty() ? "none" : use_meas.label)
                                  << " uv=(" << use_meas.uv.x << "," << use_meas.uv.y << ")\n";
                        last_wait_log_ts = now_ms;
                    }
                }
                last_cmd = cmd;
                has_last_cmd = true;
                {
                    std::lock_guard<std::mutex> lock(shared_ctrl.mu);
                    shared_ctrl.cmd = cmd;
                    shared_ctrl.has_cmd = true;
                    shared_ctrl.state = control_state;
                    shared_ctrl.has_state = true;
                    shared_ctrl.state_fresh = feedback_fresh;
                    shared_ctrl.tx_enabled = tx_enabled;
                    shared_ctrl.last_state_rx_ms = last_feedback_rx_ms;
                }

                next_send += send_period;
                now = std::chrono::steady_clock::now();
            }

            auto log_now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(log_now - last_log).count() >= 1000) {
                double elapsed_ms = static_cast<double>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(log_now - last_log).count());
                double send_rate = elapsed_ms > 0.0 ? send_count * 1000.0 / elapsed_ms : 0.0;
                double avg_loop_dt_ms = loop_dt_count > 0 ? loop_dt_ms_sum / loop_dt_count : 0.0;
                double avg_meas_age_ms = meas_age_count > 0 ? meas_age_ms_sum / meas_age_count : 0.0;
                double avg_serial_age_ms = serial_age_count > 0 ? serial_age_ms_sum / serial_age_count : 0.0;
                double avg_cmd_step_pitch = cmd_step_count > 0 ? cmd_step_pitch_sum / cmd_step_count : 0.0;
                double avg_cmd_step_yaw = cmd_step_count > 0 ? cmd_step_yaw_sum / cmd_step_count : 0.0;
                double pitch_limit_ratio = cmd_step_count > 0
                    ? static_cast<double>(rate_limit_hit_pitch) / cmd_step_count
                    : 0.0;
                double yaw_limit_ratio = cmd_step_count > 0
                    ? static_cast<double>(rate_limit_hit_yaw) / cmd_step_count
                    : 0.0;
                std::cout << "STAT_CTRL send_hz=" << send_rate
                          << " link=" << (feedback_fresh ? "READY" : "HOLD")
                          << " loop_dt_ms=" << avg_loop_dt_ms
                          << " meas_age_ms=" << avg_meas_age_ms
                          << " cmd_step_deg(p,y)=(" << avg_cmd_step_pitch << "," << avg_cmd_step_yaw << ")"
                          << " cmd_step_max(p,y)=(" << cmd_step_pitch_max << "," << cmd_step_yaw_max << ")"
                          << " rate_hit(p,y)=(" << pitch_limit_ratio << "," << yaw_limit_ratio << ")"
                          << " hold=" << hold_count
                          << " serial_age_ms=";
                if (serial_age_count > 0) {
                    std::cout << avg_serial_age_ms;
                } else {
                    std::cout << "na";
                }
                std::cout << "\n";

                // ── STAT_DELAY: 真实延迟链分解 ──
                // capture → detect → ctrl → serial_write = e2e (纯PC软件)
                // serial_age = 电控时间戳到PC接收 ≈ 串口RTT (含电控处理)
                // 建议 system_delay_ms = e2e + serial_rtt/2 (单程)
                if (e2e_count > 0) {
                    double avg_e2e = e2e_sum / e2e_count;
                    double avg_cap_det = cap_to_det_sum / e2e_count;
                    double avg_det_tx = det_to_ctrl_sum / e2e_count;
                    double serial_rtt = (serial_age_count > 0)
                        ? serial_age_ms_sum / serial_age_count : 0.0;
                    double recommended = avg_e2e + serial_rtt / 2.0;
                    std::cout << "STAT_DELAY"
                              << " cap→det=" << avg_cap_det
                              << " det→tx=" << avg_det_tx
                              << " sw_e2e=" << avg_e2e
                              << " sw_max=" << e2e_max
                              << " serial_rtt=" << serial_rtt
                              << " n=" << e2e_count
                              << " ★建议 system_delay_ms=" << recommended
                              << "\n";
                }

                loop_dt_ms_sum = 0.0;
                loop_dt_count = 0;
                meas_age_ms_sum = 0.0;
                meas_age_count = 0;
                serial_age_ms_sum = 0.0;
                serial_age_count = 0;
                send_count = 0;
                cmd_step_pitch_sum = 0.0;
                cmd_step_yaw_sum = 0.0;
                cmd_step_pitch_max = 0.0;
                cmd_step_yaw_max = 0.0;
                cmd_step_count = 0;
                rate_limit_hit_pitch = 0;
                rate_limit_hit_yaw = 0;
                hold_count = 0;
                e2e_sum = 0.0;
                cap_to_det_sum = 0.0;
                det_to_ctrl_sum = 0.0;
                e2e_count = 0;
                e2e_max = 0.0;
                last_log = log_now;
            }
        }
    });

    std::deque<double> err_history;
    std::deque<double> pitch_history;
    std::deque<double> yaw_history;
    std::deque<double> pitch_rate_history;
    std::deque<double> yaw_rate_history;
    std::deque<double> pitch_fb_history;
    std::deque<double> yaw_fb_history;
    uint64_t frame_count = 0;
    uint64_t meas_count = 0;
    double infer_ms_sum = 0.0;
    int infer_count = 0;
    double frame_age_ms_sum = 0.0;
    int frame_age_count = 0;
    double frame_to_infer_ms_sum = 0.0;
    int frame_to_infer_count = 0;
    auto last_log = std::chrono::steady_clock::now();
    bool last_meas_valid = false;
    cv::Point2f last_meas_uv(0.0f, 0.0f);
    double last_meas_conf = 0.0;
    std::string last_meas_label = "none";
    int64_t vel_last_ts = 0;             // 延迟补偿: 上次有效测量时间戳
    cv::Point2f vel_last_uv(0.0f, 0.0f); // 延迟补偿: 上次有效测量位置
    cv::Point2f smooth_vel(0.0f, 0.0f);  // 延迟补偿: EMA平滑速度 (px/s)
    int lost_streak = 0;
    int64_t last_det_log_ts = 0;

    std::cout << "Enter detect/control loops\n";
    std::cout << std::flush;
    while (true) {
        if (g_should_exit.load()) {
            break;
        }
        try {
            galaxy_camera::Frame frame;
            bool got_frame = camera.read(&frame, cam_cfg.grab_timeout_ms);
            if (got_frame && frame.bgr.empty()) {
                got_frame = false;
            }
            if (!got_frame) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            auto frame_ready_ts = std::chrono::steady_clock::now();
            int64_t ts = common::nowMs();
            int64_t host_now_ms = common::nowSystemMs();
            std::vector<detect::Detection> dets;
            auto infer_start = std::chrono::steady_clock::now();
            if (got_frame) {
                if (frame.host_timestamp > 0) {
                    int64_t host_ts_ms = frame.host_timestamp;
                    if (host_ts_ms > kEpochMsThreshold * 1000LL) {
                        host_ts_ms /= 1000;
                    }
                    if (host_now_ms >= kEpochMsThreshold && host_ts_ms >= kEpochMsThreshold) {
                        double age_ms = static_cast<double>(host_now_ms - host_ts_ms);
                        frame_age_ms_sum += age_ms;
                        frame_age_count++;
                    }
                }
                auto result = detector.detectCascade(frame.bgr, ts);
                // 将级联检测结果转为 Detection 列表 (兼容下游)
                for (const auto& lrx : result.laser_rxs) {
                    detect::Detection d;
                    d.valid = true;
                    d.class_id = 0;
                    d.label = "laser_rx";
                    d.bbox = lrx.bbox;
                    d.center = lrx.center;
                    d.radius = std::max(lrx.bbox.width, lrx.bbox.height) * 0.5f;
                    d.confidence = lrx.confidence;
                    dets.push_back(d);
                }
                // 直接检测失败时, 使用偏移预测结果 (第3次锁定后模块停止发光)
                if (dets.empty() && result.has_predicted_target) {
                    detect::Detection d;
                    d.valid = true;
                    d.class_id = 0;
                    d.label = "laser_rx_predicted";
                    d.center = result.predicted_center;
                    d.radius = 10.0f;
                    d.bbox = cv::Rect(
                        static_cast<int>(result.predicted_center.x - 10),
                        static_cast<int>(result.predicted_center.y - 10),
                        20, 20);
                    d.confidence = result.predicted_confidence;
                    dets.push_back(d);
                }
                auto infer_end = std::chrono::steady_clock::now();
                double infer_ms = std::chrono::duration<double, std::milli>(infer_end - infer_start).count();
                infer_ms_sum += infer_ms;
                infer_count++;
                frame_to_infer_ms_sum += std::chrono::duration<double, std::milli>(infer_end - frame_ready_ts).count();
                frame_to_infer_count++;
            } else {
                auto infer_end = infer_start;
                frame_to_infer_ms_sum += std::chrono::duration<double, std::milli>(infer_end - frame_ready_ts).count();
                frame_to_infer_count++;
            }
            common::TargetMeasurement meas;
            meas.timestamp = ts;
            if (!dets.empty()) {
                meas = detect::toMeasurement(dets.front(), ts);
            }
            // ── 延迟补偿: 估计像素速度 (px/s) ──
            if (meas.valid && vel_last_ts > 0) {
                double dt_ms = static_cast<double>(ts - vel_last_ts);
                if (dt_ms > 0.5 && dt_ms < 200.0) {
                    double dt_s = dt_ms / 1000.0;
                    float vx = static_cast<float>((meas.uv.x - vel_last_uv.x) / dt_s);
                    float vy = static_cast<float>((meas.uv.y - vel_last_uv.y) / dt_s);
                    const float alpha = 0.4f;  // EMA 平滑系数
                    smooth_vel.x = alpha * vx + (1.0f - alpha) * smooth_vel.x;
                    smooth_vel.y = alpha * vy + (1.0f - alpha) * smooth_vel.y;
                }
            }
            if (meas.valid) {
                meas.velocity = smooth_vel;
                vel_last_ts = ts;
                vel_last_uv = meas.uv;
            } else {
                smooth_vel = cv::Point2f(0.0f, 0.0f);
            }
            int64_t detect_done_wall = common::nowMs();
            if (got_frame) {
                frame_count++;
            }
            if (meas.valid) {
                meas_count++;
            }

            if (!got_frame) {
                std::cout << "DBG frame_miss ts=" << ts << "\n";
            }

            if (meas.valid) {
                lost_streak = 0;
                int64_t det_log_ts = ts;
                if (det_log_ts - last_det_log_ts >= 1000) {
                    if (!dets.empty()) {
                        const auto& det = dets.front();
                        double du = det.center.x - boresight.u_L;
                        double dv = det.center.y - boresight.v_L;
                        std::cout << "DBG det_status ts=" << ts
                                  << " uv=(" << det.center.x << "," << det.center.y << ")"
                                  << " du=" << du << " dv=" << dv
                                  << " bbox=(" << det.bbox.x << "," << det.bbox.y
                                  << "," << det.bbox.width << "," << det.bbox.height << ")"
                                  << " conf=" << det.confidence
                                  << " label=" << det.label
                                  << " det_count=" << dets.size()
                                  << " frame=(";
                        if (got_frame) {
                            std::cout << frame.width << "," << frame.height;
                        } else {
                            std::cout << "na,na";
                        }
                        std::cout << ")"
                                  << " boresight=(" << boresight.u_L << "," << boresight.v_L << ")"
                                  << "\n";
                    }
                    last_det_log_ts = det_log_ts;
                }
                if (!last_meas_valid) {
                    std::cout << "DBG meas_acquire ts=" << ts
                              << " uv=(" << meas.uv.x << "," << meas.uv.y << ")"
                              << " conf=" << meas.confidence
                              << " label=" << meas.label << "\n";
                } else {
                    double du = meas.uv.x - last_meas_uv.x;
                    double dv = meas.uv.y - last_meas_uv.y;
                    double dist = std::sqrt(du * du + dv * dv);
                    if (dist > kJumpThreshPx) {
                        std::cout << "DBG meas_jump ts=" << ts
                                  << " dist_px=" << dist
                                  << " uv=(" << meas.uv.x << "," << meas.uv.y << ")"
                                  << " last=(" << last_meas_uv.x << "," << last_meas_uv.y << ")\n";
                        if (!dets.empty()) {
                            const auto& det = dets.front();
                            std::cout << "DBG det0 center=(" << det.center.x << "," << det.center.y << ")"
                                      << " bbox=(" << det.bbox.x << "," << det.bbox.y
                                      << "," << det.bbox.width << "," << det.bbox.height << ")"
                                      << " conf=" << det.confidence
                                      << " label=" << det.label
                                      << " count=" << dets.size()
                                      << "\n";
                        }
                    }
                }
            } else {
                lost_streak++;
                if (last_meas_valid) {
                    std::cout << "DBG meas_lost ts=" << ts
                              << " lost_streak=" << lost_streak << "\n";
                }
            }

            last_meas_valid = meas.valid;
            if (meas.valid) {
                last_meas_uv = meas.uv;
                last_meas_conf = meas.confidence;
                last_meas_label = meas.label.empty() ? "unknown" : meas.label;
            } else {
                last_meas_label = "none";
            }

            if (got_frame) {
                std::lock_guard<std::mutex> lock(shared_meas.mu);
                shared_meas.meas = meas;
                shared_meas.has_meas = true;
                shared_meas.capture_wall_ms = ts;
                shared_meas.detect_done_wall_ms = detect_done_wall;
            }

            common::GimbalCommand cmd_snapshot;
            common::GimbalState state_snapshot;
            bool has_cmd = false;
            bool has_state = false;
            bool state_fresh = false;
            bool tx_enabled = false;
            int64_t last_state_rx_ms = 0;
            {
                std::lock_guard<std::mutex> lock(shared_ctrl.mu);
                if (shared_ctrl.has_cmd) {
                    cmd_snapshot = shared_ctrl.cmd;
                    has_cmd = true;
                }
                if (shared_ctrl.has_state) {
                    state_snapshot = shared_ctrl.state;
                    has_state = true;
                }
                state_fresh = shared_ctrl.state_fresh;
                tx_enabled = shared_ctrl.tx_enabled;
                last_state_rx_ms = shared_ctrl.last_state_rx_ms;
            }

            if (has_cmd) {
                double err_px = 0.0;
                if (meas.valid) {
                    double du = meas.uv.x - boresight.u_L;
                    double dv = meas.uv.y - boresight.v_L;
                    err_px = std::sqrt(du * du + dv * dv);
                }
                pushHistory(&err_history, err_px);
                pushHistory(&pitch_history, cmd_snapshot.pitch);
                pushHistory(&yaw_history, cmd_snapshot.yaw);
                pushHistory(&pitch_rate_history, cmd_snapshot.pitch_rate);
                pushHistory(&yaw_rate_history, cmd_snapshot.yaw_rate);
            }
            if (has_state) {
                pushHistory(&pitch_fb_history, state_snapshot.pitch);
                pushHistory(&yaw_fb_history, state_snapshot.yaw);
            }

            auto log_now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(log_now - last_log).count() >= 1000) {
                double fps = frame_count * 1000.0 /
                    static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(log_now - last_log).count());
                double avg_infer_ms = infer_count > 0 ? infer_ms_sum / infer_count : 0.0;
                double avg_frame_age_ms = frame_age_count > 0 ? frame_age_ms_sum / frame_age_count : 0.0;
                double avg_frame_to_infer_ms =
                    frame_to_infer_count > 0 ? frame_to_infer_ms_sum / frame_to_infer_count : 0.0;
                std::cout << "STAT fps=" << fps
                          << " infer_ms=" << avg_infer_ms
                          << " frame_to_infer_ms=" << avg_frame_to_infer_ms
                          << " frame_age_ms=";
                if (frame_age_count > 0) {
                    std::cout << avg_frame_age_ms;
                } else {
                    std::cout << "na";
                }
                std::cout
                          << " meas=" << meas_count;
                if (has_cmd) {
                    std::cout << " cmd(p,y)=(" << cmd_snapshot.pitch << "," << cmd_snapshot.yaw << ")";
                }
                if (has_state) {
                    std::cout << " fb(p,y)=(" << state_snapshot.pitch << "," << state_snapshot.yaw << ")";
                }
                std::cout
                          << " gimbal_link=" << (state_fresh ? "ready" : "hold")
                          << " tx=" << (tx_enabled ? "on" : "off")
                          << " uv=(" << meas.uv.x << "," << meas.uv.y << ")"
                          << " label=" << (meas.label.empty() ? "none" : meas.label)
                          << " valid=" << (meas.valid ? 1 : 0)
                          << "\n";
                frame_count = 0;
                meas_count = 0;
                infer_ms_sum = 0.0;
                infer_count = 0;
                frame_age_ms_sum = 0.0;
                frame_age_count = 0;
                frame_to_infer_ms_sum = 0.0;
                frame_to_infer_count = 0;
                last_log = log_now;
            }

            if (show && window_created) {
                cv::Mat view(kDisplayHeight, kDisplayWidth, CV_8UC3, cv::Scalar(10, 10, 10));
                double sx = 1.0;
                double sy = 1.0;
                int ox = 0;
                int oy = 0;
                if (got_frame) {
                    const double scale = std::min(
                        static_cast<double>(kDisplayWidth) / static_cast<double>(frame.bgr.cols),
                        static_cast<double>(kDisplayHeight) / static_cast<double>(frame.bgr.rows));
                    const int draw_w = std::max(1, static_cast<int>(std::round(frame.bgr.cols * scale)));
                    const int draw_h = std::max(1, static_cast<int>(std::round(frame.bgr.rows * scale)));
                    ox = (kDisplayWidth - draw_w) / 2;
                    oy = (kDisplayHeight - draw_h) / 2;
                    cv::Mat resized;
                    cv::resize(frame.bgr, resized, cv::Size(draw_w, draw_h));
                    resized.copyTo(view(cv::Rect(ox, oy, draw_w, draw_h)));
                    sx = scale;
                    sy = scale;
                } else {
                    cv::putText(view, "Waiting for frames...", cv::Point(40, 60),
                                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
                }

                cv::Point boresight_pt(kDisplayWidth / 2, kDisplayHeight / 2);
                if (got_frame) {
                    boresight_pt = cv::Point(static_cast<int>(std::round(ox + boresight.u_L * sx)),
                                             static_cast<int>(std::round(oy + boresight.v_L * sy)));
                }
                cv::drawMarker(view, boresight_pt, cv::Scalar(0, 0, 255),
                               cv::MARKER_CROSS, 30, 3, cv::LINE_AA);

                if (got_frame) {
                    for (const auto& det : dets) {
                        const cv::Scalar det_color = colorForLabel(det.label);
                        const cv::Rect scaled_box(
                            static_cast<int>(std::round(ox + det.bbox.x * sx)),
                            static_cast<int>(std::round(oy + det.bbox.y * sy)),
                            std::max(1, static_cast<int>(std::round(det.bbox.width * sx))),
                            std::max(1, static_cast<int>(std::round(det.bbox.height * sy))));
                        cv::rectangle(view, scaled_box, det_color, 2, cv::LINE_AA);
                        cv::putText(view,
                                    det.label + " " + cv::format("%.2f", det.confidence),
                                    cv::Point(scaled_box.x, std::max(25, scaled_box.y - 8)),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.7, det_color, 2, cv::LINE_AA);
                    }
                }

                if (meas.valid && got_frame) {
                    cv::Point target_pt(static_cast<int>(std::round(ox + meas.uv.x * sx)),
                                        static_cast<int>(std::round(oy + meas.uv.y * sy)));
                    cv::circle(view, target_pt, 12, colorForLabel(meas.label), 3, cv::LINE_AA);
                }

                const int font = cv::FONT_HERSHEY_SIMPLEX;
                if (has_cmd) {
                    cv::putText(view, "Pitch cmd: " + std::to_string(cmd_snapshot.pitch),
                                cv::Point(20, 40), font, 0.8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
                    cv::putText(view, "Yaw cmd: " + std::to_string(cmd_snapshot.yaw),
                                cv::Point(20, 70), font, 0.8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
                    cv::putText(view, "Tx mode: " + std::to_string(static_cast<int>(cmd_snapshot.mode)),
                                cv::Point(20, 100), font, 0.8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
                }
                cv::putText(view, std::string("Gimbal link: ") + (state_fresh ? "READY" : "HOLD"),
                            cv::Point(20, 130), font, 0.8,
                            state_fresh ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 215, 255), 2, cv::LINE_AA);
                cv::putText(view, std::string("Tx gate: ") + (tx_enabled ? "OPEN" : "CLOSED"),
                            cv::Point(20, 160), font, 0.8,
                            tx_enabled ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 215, 255), 2, cv::LINE_AA);
                if (last_state_rx_ms > 0) {
                    cv::putText(view,
                                "Last rx ms: " + std::to_string(common::nowSystemMs() - last_state_rx_ms),
                                cv::Point(20, 190), font, 0.8, cv::Scalar(180, 255, 255), 2, cv::LINE_AA);
                }
                if (has_state) {
                    cv::putText(view, "Pitch fb: " + std::to_string(state_snapshot.pitch),
                                cv::Point(20, 220), font, 0.8, cv::Scalar(180, 255, 255), 2, cv::LINE_AA);
                    cv::putText(view, "Yaw fb: " + std::to_string(state_snapshot.yaw),
                                cv::Point(20, 250), font, 0.8, cv::Scalar(180, 255, 255), 2, cv::LINE_AA);
                }
                cv::putText(view, "Label: " + last_meas_label,
                            cv::Point(20, 280), font, 0.8, colorForLabel(last_meas_label), 2, cv::LINE_AA);
                cv::putText(view, "Conf: " + std::to_string(last_meas_conf),
                            cv::Point(20, 310), font, 0.8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
                cv::putText(view, std::string("Laser hit: ") + (isPurpleLabel(last_meas_label) ? "YES" : "NO"),
                            cv::Point(20, 340), font, 0.8,
                            isPurpleLabel(last_meas_label) ? cv::Scalar(255, 0, 255)
                                                           : cv::Scalar(200, 200, 200),
                            2, cv::LINE_AA);

                try {
                    cv::Mat plot(kPlotHeight, kPlotWidth, CV_8UC3);
                    drawPlot(pitch_history, yaw_history, "pitch_cmd (deg)", "yaw_cmd (deg)", &plot);

                    cv::Mat plot_err(kPlotHeight, kPlotWidth, CV_8UC3);
                    drawSinglePlot(err_history, "err_px", &plot_err);

                    cv::Mat plot_rate(kPlotHeight, kPlotWidth, CV_8UC3);
                    drawPlot(pitch_rate_history, yaw_rate_history,
                             "pitch_rate_cmd (deg/s)", "yaw_rate_cmd (deg/s)", &plot_rate);

                    cv::Mat plot_fb(kPlotHeight, kPlotWidth, CV_8UC3);
                    drawPlot(pitch_fb_history, yaw_fb_history, "pitch_fb (deg)", "yaw_fb (deg)", &plot_fb);

                    cv::Mat panel(kPanelHeight, kPanelWidth, CV_8UC3, cv::Scalar(10, 10, 10));
                    view.copyTo(panel(cv::Rect(0, 0, kDisplayWidth, kDisplayHeight)));
                    plot.copyTo(panel(cv::Rect(0, kDisplayHeight + kPanelPadding, kPlotWidth, kPlotHeight)));
                    plot_err.copyTo(panel(cv::Rect(kPlotWidth + kPanelPadding,
                                                   kDisplayHeight + kPanelPadding,
                                                   kPlotWidth, kPlotHeight)));
                    plot_rate.copyTo(panel(cv::Rect(0,
                                                    kDisplayHeight + kPanelPadding * 2 + kPlotHeight,
                                                    kPlotWidth, kPlotHeight)));
                    plot_fb.copyTo(panel(cv::Rect(kPlotWidth + kPanelPadding,
                                                  kDisplayHeight + kPanelPadding * 2 + kPlotHeight,
                                                  kPlotWidth, kPlotHeight)));
                    cv::imshow("control_panel", panel);
                } catch (const cv::Exception& e) {
                    std::cerr << "OpenCV GUI error: " << e.what() << "\n";
                    show = false;
                }

                int key = cv::waitKey(1);
                window_ready = true;
                window_guard++;
                if (key == 'q' || key == 'Q') {
                    break;
                }
                if (exit_on_close && window_ready && window_guard > 5) {
                    double visible = cv::getWindowProperty("control_panel", cv::WND_PROP_VISIBLE);
                    if (visible >= 0.0 && visible < 1.0) {
                        break;
                    }
                }
            }
        } catch (const cv::Exception& e) {
            std::cerr << "OpenCV error: " << e.what() << "\n";
            show = false;
        } catch (const std::exception& e) {
            std::cerr << "Unhandled exception: " << e.what() << "\n";
            break;
        }
    }

    g_should_exit.store(true);
    if (control_thread.joinable()) {
        control_thread.join();
    }
    return 0;
}
