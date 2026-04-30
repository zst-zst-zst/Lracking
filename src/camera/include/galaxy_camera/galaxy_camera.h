#ifndef GALAXY_CAMERA_H
#define GALAXY_CAMERA_H

#include <GxIAPI.h>
#include <GxPixelFormat.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#if defined(GALAXY_CAMERA_WITH_OPENCV_CUDA)
#include <opencv2/core/cuda.hpp>
#endif
#if defined(GALAXY_CAMERA_WITH_CUDA)
#include <cuda_runtime.h>
#endif

namespace galaxy_camera {

struct RoiConfig {
    bool enable = false;
    int offset_x = 0;
    int offset_y = 0;
    int width = 0;
    int height = 0;
};

struct CameraConfig {
    std::string serial_number;
    int device_index = 0;
    bool use_first_device = true;

    bool trigger_mode = false;
    int trigger_source = 7;

    bool frame_rate_enable = true;
    double frame_rate = 120.0;

    bool auto_exposure = false;
    double exposure_time_us = 3000.0;
    double auto_exposure_time_lower_us = 0.0;
    double auto_exposure_time_upper_us = 0.0;

    bool auto_gain = true;
    double gain = 0.0;

    bool balance_white_auto = true;
    bool gamma_enable = false;
    int gamma_selector = 0;
    double gamma = 0.7;
    bool brightness_enable = false;
    double brightness = 0.0;
    bool digital_shift_enable = false;
    double digital_shift = 0.0;

    RoiConfig roi;

    bool output_bgr = true;
    bool output_raw = true;
    bool copy_raw = true;

    bool rotate_180 = false;

    int grab_timeout_ms = 1000;
    int image_node_num = 8;
    int grab_strategy = 0;

    bool undistort_enable = false;
    std::string calib_path;

    bool reconnect_enable = true;
    int reconnect_max_failures = 10;
    int reconnect_retry_delay_ms = 1000;
    int reconnect_max_retries = 0;

    bool feature_load_enable = false;
    std::string feature_load_path;
    bool feature_save_enable = false;
    std::string feature_save_path;
    bool feature_save_on_close = false;

    bool zero_copy_enable = false;
    int buffer_pool_size = 6;
    bool gpu_pipeline_enable = false;
    int gpu_device_id = 0;
    bool gpu_stream_enable = true;
    bool gpu_demosaic_enable = false;
    std::string gpu_demosaic_backend = "auto";
    bool force_swap_rb = false;
};

struct Frame {
    uint32_t width = 0;
    uint32_t height = 0;
    uint64_t frame_id = 0;
    uint64_t device_timestamp = 0;
    int64_t host_timestamp = 0;
    uint32_t pixel_type = 0;

    std::vector<uint8_t> raw_data;
    std::vector<uint8_t> bgr_data;
    uint8_t* bgr_ptr = nullptr;
    size_t bgr_size = 0;
    bool bgr_is_copy = true;
    const uint8_t* raw_ptr = nullptr;
    size_t raw_size = 0;
    bool raw_is_copy = true;

    cv::Mat bgr;

    uint8_t* gpu_bgr_ptr = nullptr;
    int gpu_stride_bytes = 0;
    bool gpu_valid = false;
    void* gpu_stream = nullptr;
};

bool loadCameraConfig(const std::string& path, CameraConfig* config);

class GalaxyCamera {
public:
    using FrameCallback = std::function<void(const Frame&)>;

    GalaxyCamera();
    ~GalaxyCamera();

    bool open(const CameraConfig& config);
    void close();

    bool startGrabbing();
    void stopGrabbing();

    bool read(Frame* out, int timeout_ms);

    void setFrameCallback(FrameCallback cb);

    bool isOpen() const;
    bool isGrabbing() const;

    bool setExposure(double us);
    bool setGain(double gain);
    bool setFrameRate(double fps);
    bool setTriggerMode(bool enable);
    bool setRoi(const RoiConfig& roi);

    const CameraConfig& config() const;

private:
    bool initSdkOnce();
    void releaseSdkOnce();

    bool openDevice();
    void closeDevice();
    bool applyConfig();
    bool prepareBuffers();
    bool reopenDevice();
    bool applyFeatureLoad();
    bool applyFeatureSave(const std::string& path);
    void registerExceptionCallback();
    void handleDisconnect();

    bool grabOnce(Frame* out, int timeout_ms);
    bool buildFrameFromBuffer(Frame* out, const GX_FRAME_DATA& gx_frame, bool allow_zero_copy);
    void grabLoop();
    void configureGpuPipeline();
    void configurePinnedPool();
    void unregisterPinnedPool();
    void releaseGpuResources();
    bool tryGpuDemosaic([[maybe_unused]] const GX_FRAME_DATA& gx_frame,
                        int pixel_type,
                        [[maybe_unused]] Frame& frame,
                        [[maybe_unused]] uint8_t* bgr_dst);

    static std::string getDeviceSerial(const GX_DEVICE_INFO& info);
    static void printDeviceInfo(const GX_DEVICE_INFO& info);
    static void GX_STDC ExceptionCallback(unsigned int msg_type, void* user);

    GX_DEV_HANDLE handle_ = nullptr;
    CameraConfig config_;
    std::vector<uint8_t> grab_buffer_;

    std::thread grab_thread_;
    std::atomic<bool> grabbing_{false};
    std::atomic<bool> exit_thread_{false};
    FrameCallback callback_;

    std::mutex frame_mutex_;
    Frame frame_cache_;

    size_t payload_size_ = 0;
    std::vector<std::vector<uint8_t>> bgr_pool_;
    size_t bgr_pool_index_ = 0;
    size_t bgr_pool_bytes_ = 0;
    bool bgr_pool_pinned_ = false;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat undistort_map1_;
    cv::Mat undistort_map2_;
    std::vector<uint8_t> undistort_buffer_;
    cv::Mat undistort_input_;

#if defined(GALAXY_CAMERA_WITH_OPENCV_CUDA)
    cv::cuda::GpuMat gpu_raw_;
    cv::cuda::GpuMat gpu_bgr_;
    cv::cuda::Stream gpu_stream_;
#endif
#if defined(GALAXY_CAMERA_WITH_CUDA)
    cudaStream_t cuda_stream_ = nullptr;
    uint8_t* npp_raw_dev_ = nullptr;
    uint8_t* npp_bgr_dev_ = nullptr;
    size_t npp_raw_bytes_ = 0;
    size_t npp_bgr_bytes_ = 0;
#endif

    std::mutex reconnect_mutex_;
    std::atomic<bool> reconnecting_{false};
    std::atomic<bool> exception_registered_{false};
    bool gpu_swap_rb_ = true;
    bool gpu_channel_order_decided_ = false;
};

}  // namespace galaxy_camera

#endif  // GALAXY_CAMERA_H
