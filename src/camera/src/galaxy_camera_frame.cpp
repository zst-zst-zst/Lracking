#include "galaxy_camera/galaxy_camera.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

namespace galaxy_camera {
namespace {
std::atomic<bool> g_zero_copy_pool_logged{false};
std::atomic<bool> g_zero_copy_frame_logged{false};
std::atomic<bool> g_gpu_upload_logged{false};

bool readRosMatrix(const cv::FileNode& node, cv::Mat* out) {
    if (node.empty() || !out) {
        return false;
    }
    int rows = 0;
    int cols = 0;
    node["rows"] >> rows;
    node["cols"] >> cols;
    std::vector<double> data;
    node["data"] >> data;
    if (rows <= 0 || cols <= 0 || static_cast<size_t>(rows * cols) != data.size()) {
        return false;
    }
    cv::Mat mat(rows, cols, CV_64F);
    std::memcpy(mat.data, data.data(), data.size() * sizeof(double));
    *out = mat;
    return true;
}

bool is_bayer8(int pixel_type) {
    return pixel_type == GX_PIXEL_FORMAT_BAYER_GR8 ||
           pixel_type == GX_PIXEL_FORMAT_BAYER_RG8 ||
           pixel_type == GX_PIXEL_FORMAT_BAYER_GB8 ||
           pixel_type == GX_PIXEL_FORMAT_BAYER_BG8;
}

int bayer_to_cv_bgr_code_from_pixel(int pixel_type) {
    switch (pixel_type) {
        case GX_PIXEL_FORMAT_BAYER_BG8:
            return cv::COLOR_BayerBG2BGR;
        case GX_PIXEL_FORMAT_BAYER_GB8:
            return cv::COLOR_BayerGB2BGR;
        case GX_PIXEL_FORMAT_BAYER_GR8:
            return cv::COLOR_BayerGR2BGR;
        case GX_PIXEL_FORMAT_BAYER_RG8:
            return cv::COLOR_BayerRG2BGR;
        default:
            return cv::COLOR_BayerGB2BGR;
    }
}

double meanAbsColorDiff(const cv::Mat& a, const cv::Mat& b) {
    if (a.empty() || b.empty() || a.size() != b.size() || a.type() != b.type()) {
        return std::numeric_limits<double>::infinity();
    }
    cv::Mat diff;
    cv::absdiff(a, b, diff);
    cv::Scalar m = cv::mean(diff);
    return (m[0] + m[1] + m[2]) / 3.0;
}

bool cameraMatrixLooksValidForFrame(const cv::Mat& camera_matrix,
                                    int frame_width,
                                    int frame_height,
                                    std::string* reason) {
    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        if (reason) {
            *reason = "camera_matrix is empty or not 3x3";
        }
        return false;
    }
    if (camera_matrix.type() != CV_64F) {
        if (reason) {
            *reason = "camera_matrix type is not CV_64F";
        }
        return false;
    }
    const double fx = camera_matrix.at<double>(0, 0);
    const double fy = camera_matrix.at<double>(1, 1);
    const double cx = camera_matrix.at<double>(0, 2);
    const double cy = camera_matrix.at<double>(1, 2);
    if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) || !std::isfinite(cy)) {
        if (reason) {
            *reason = "camera_matrix has non-finite values";
        }
        return false;
    }
    if (fx <= 0.0 || fy <= 0.0) {
        if (reason) {
            *reason = "camera_matrix focal length <= 0";
        }
        return false;
    }
    if (frame_width <= 0 || frame_height <= 0) {
        if (reason) {
            *reason = "invalid frame size";
        }
        return false;
    }
    if (cx < 0.0 || cx >= static_cast<double>(frame_width) ||
        cy < 0.0 || cy >= static_cast<double>(frame_height)) {
        if (reason) {
            std::ostringstream oss;
            oss << "principal point out of frame: cx=" << cx << " cy=" << cy
                << " frame=" << frame_width << "x" << frame_height;
            *reason = oss.str();
        }
        return false;
    }
    return true;
}

}  // namespace

bool GalaxyCamera::prepareBuffers() {
    GX_INT_VALUE payload{};
    if (GXGetIntValue(handle_, "PayloadSize", &payload) == GX_STATUS_SUCCESS) {
        payload_size_ = static_cast<size_t>(payload.nCurValue);
    }

    GX_INT_VALUE width{};
    GX_INT_VALUE height{};
    if (GXGetIntValue(handle_, "Width", &width) != GX_STATUS_SUCCESS ||
        GXGetIntValue(handle_, "Height", &height) != GX_STATUS_SUCCESS) {
        std::cerr << "Get Width/Height failed\n";
        return false;
    }

    frame_cache_.width = static_cast<uint32_t>(width.nCurValue);
    frame_cache_.height = static_cast<uint32_t>(height.nCurValue);
    if (payload_size_ == 0) {
        payload_size_ = static_cast<size_t>(frame_cache_.width) * frame_cache_.height * 3;
    }
    grab_buffer_.resize(payload_size_);

    if (config_.output_bgr) {
        frame_cache_.bgr_data.resize(static_cast<size_t>(frame_cache_.width) * frame_cache_.height * 3);
        frame_cache_.bgr = cv::Mat(frame_cache_.height, frame_cache_.width, CV_8UC3, frame_cache_.bgr_data.data());
    }
    if (config_.output_raw) {
        frame_cache_.raw_data.resize(payload_size_);
    }

    configureGpuPipeline();

    if (config_.output_bgr && config_.zero_copy_enable) {
        size_t bgr_size = static_cast<size_t>(frame_cache_.width) * frame_cache_.height * 3;
        int pool_size = std::max(1, config_.buffer_pool_size);
        if (bgr_pool_bytes_ != bgr_size || static_cast<int>(bgr_pool_.size()) != pool_size) {
            unregisterPinnedPool();
            bgr_pool_bytes_ = bgr_size;
            bgr_pool_.assign(static_cast<size_t>(pool_size), std::vector<uint8_t>(bgr_size));
            bgr_pool_index_ = 0;
            if (!g_zero_copy_pool_logged.exchange(true)) {
                std::cout << "Zero-copy pool enabled: pool_size=" << pool_size
                          << " bgr_bytes=" << bgr_size << "\n";
            }
        }
        configurePinnedPool();
    } else {
        unregisterPinnedPool();
        bgr_pool_.clear();
        bgr_pool_bytes_ = 0;
        bgr_pool_index_ = 0;
    }

    if (config_.undistort_enable && config_.output_bgr && config_.calib_path.empty()) {
        std::cerr << "Undistort enabled but calib_path is empty\n";
        return false;
    }

    if (config_.undistort_enable && config_.output_bgr && !config_.calib_path.empty()) {
        cv::FileStorage fs;
        try {
            fs.open(config_.calib_path, cv::FileStorage::READ);
        } catch (const cv::Exception&) {
            fs.release();
        }
        if (!fs.isOpened()) {
            std::ifstream in(config_.calib_path);
            if (!in.is_open()) {
                std::cerr << "Failed to open calib file: " << config_.calib_path << "\n";
                return false;
            }
            std::ostringstream oss;
            oss << in.rdbuf();
            std::string content = oss.str();
            if (content.rfind("%YAML:", 0) != 0) {
                content = "%YAML:1.0\n" + content;
            }
            fs.open(content, cv::FileStorage::READ | cv::FileStorage::MEMORY);
        }

        int calib_width = 0;
        int calib_height = 0;
        fs["image_width"] >> calib_width;
        fs["image_height"] >> calib_height;
        cv::FileNode cam_node = fs["camera_matrix"];
        if (!readRosMatrix(cam_node, &camera_matrix_)) {
            cam_node = fs["K"];
            readRosMatrix(cam_node, &camera_matrix_);
        }
        cv::FileNode dist_node = fs["distortion_coefficients"];
        if (!readRosMatrix(dist_node, &dist_coeffs_)) {
            dist_node = fs["D"];
            readRosMatrix(dist_node, &dist_coeffs_);
        }
        if (camera_matrix_.empty()) {
            fs["K"] >> camera_matrix_;
        }
        if (dist_coeffs_.empty()) {
            fs["D"] >> dist_coeffs_;
        }
        if (camera_matrix_.empty() || dist_coeffs_.empty()) {
            std::cerr << "Invalid calibration data in: " << config_.calib_path << "\n";
            return false;
        }
        if (calib_width > 0 && calib_height > 0 &&
            (calib_width != static_cast<int>(frame_cache_.width) ||
             calib_height != static_cast<int>(frame_cache_.height))) {
            std::cerr << "Calib size mismatch: file " << calib_width << "x" << calib_height
                      << " vs frame " << frame_cache_.width << "x" << frame_cache_.height << "\n";
        }
        std::string invalid_reason;
        if (!cameraMatrixLooksValidForFrame(camera_matrix_,
                                            static_cast<int>(frame_cache_.width),
                                            static_cast<int>(frame_cache_.height),
                                            &invalid_reason)) {
            std::cerr << "Undistort disabled: " << invalid_reason << "\n";
            undistort_map1_.release();
            undistort_map2_.release();
            undistort_buffer_.clear();
            undistort_input_.release();
        } else {
            cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_, cv::Mat(),
                camera_matrix_, cv::Size(frame_cache_.width, frame_cache_.height),
                CV_16SC2, undistort_map1_, undistort_map2_);
            undistort_buffer_.resize(static_cast<size_t>(frame_cache_.width) * frame_cache_.height * 3);
            undistort_input_ = cv::Mat(frame_cache_.height, frame_cache_.width, CV_8UC3);
        }
    }

    return true;
}

bool GalaxyCamera::grabOnce(Frame* out, int timeout_ms) {
    GX_FRAME_DATA gx_frame{};
    gx_frame.pImgBuf = grab_buffer_.data();
    gx_frame.nImgSize = static_cast<int32_t>(grab_buffer_.size());
    if (GXGetImage(handle_, &gx_frame, static_cast<uint32_t>(std::max(1, timeout_ms))) != GX_STATUS_SUCCESS) {
        return false;
    }
    if (gx_frame.nStatus != GX_FRAME_STATUS_SUCCESS) {
        return false;
    }
    return buildFrameFromBuffer(out, gx_frame, !config_.copy_raw);
}

void GalaxyCamera::grabLoop() {
    int failure_count = 0;
    int reconnect_attempts = 0;
    while (!exit_thread_ && grabbing_) {
        Frame frame;
        if (grabOnce(&frame, config_.grab_timeout_ms)) {
            failure_count = 0;
            reconnect_attempts = 0;
            if (callback_) {
                callback_(frame);
            }
            continue;
        }

        failure_count++;
        if (!config_.reconnect_enable ||
            failure_count < std::max(1, config_.reconnect_max_failures)) {
            continue;
        }

        if (config_.reconnect_max_retries > 0 && reconnect_attempts >= config_.reconnect_max_retries) {
            std::cerr << "Reconnect retry limit reached\n";
            failure_count = 0;
            continue;
        }

        reconnect_attempts++;
        std::cerr << "Camera disconnected, attempting reconnect...\n";
        stopGrabbing();
        if (reopenDevice() && startGrabbing()) {
            std::cerr << "Reconnect succeeded\n";
            failure_count = 0;
            reconnect_attempts = 0;
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(config_.reconnect_retry_delay_ms));
    }
}

bool GalaxyCamera::buildFrameFromBuffer(Frame* out, const GX_FRAME_DATA& gx_frame, bool allow_zero_copy) {
    if (!out || !gx_frame.pImgBuf) {
        return false;
    }

    Frame& frame = *out;
    frame.width = static_cast<uint32_t>(gx_frame.nWidth);
    frame.height = static_cast<uint32_t>(gx_frame.nHeight);
    frame.frame_id = gx_frame.nFrameID;
    frame.device_timestamp = gx_frame.nTimestamp;
    frame.host_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    frame.pixel_type = static_cast<uint32_t>(gx_frame.nPixelFormat);

    frame.raw_ptr = nullptr;
    frame.raw_size = 0;
    frame.raw_is_copy = true;
    frame.bgr_ptr = nullptr;
    frame.bgr_size = 0;
    frame.bgr_is_copy = true;
    frame.gpu_bgr_ptr = nullptr;
    frame.gpu_stride_bytes = 0;
    frame.gpu_valid = false;
    frame.gpu_stream = nullptr;

    const uint8_t* src = static_cast<const uint8_t*>(gx_frame.pImgBuf);
    const size_t src_size = static_cast<size_t>(gx_frame.nImgSize);
    const int pixel_type = static_cast<int>(frame.pixel_type);

    if (config_.output_raw) {
        if (!config_.copy_raw && allow_zero_copy) {
            frame.raw_ptr = src;
            frame.raw_size = src_size;
            frame.raw_is_copy = false;
        } else {
            frame.raw_data.resize(src_size);
            std::memcpy(frame.raw_data.data(), src, src_size);
            frame.raw_ptr = frame.raw_data.data();
            frame.raw_size = src_size;
            frame.raw_is_copy = true;
        }
    }

    if (!config_.output_bgr && is_bayer8(pixel_type)) {
        (void)tryGpuDemosaic(gx_frame, pixel_type, frame, nullptr);
    }

    if (config_.output_bgr) {
        const size_t bgr_size = static_cast<size_t>(frame.width) * frame.height * 3;
        uint8_t* bgr_dst = nullptr;
        if (config_.zero_copy_enable && !bgr_pool_.empty() && bgr_pool_bytes_ == bgr_size) {
            bgr_dst = bgr_pool_[bgr_pool_index_].data();
            bgr_pool_index_ = (bgr_pool_index_ + 1) % bgr_pool_.size();
            frame.bgr_ptr = bgr_dst;
            frame.bgr_size = bgr_size;
            frame.bgr_is_copy = false;
            frame.bgr_data.clear();
            if (!g_zero_copy_frame_logged.exchange(true)) {
                std::cout << "Zero-copy BGR frame active (bgr_is_copy=0)\n";
            }
        } else {
            frame.bgr_data.resize(bgr_size);
            bgr_dst = frame.bgr_data.data();
            frame.bgr_ptr = bgr_dst;
            frame.bgr_size = bgr_size;
            frame.bgr_is_copy = true;
        }

        frame.bgr = cv::Mat(frame.height, frame.width, CV_8UC3, bgr_dst);

        if (pixel_type == GX_PIXEL_FORMAT_BGR8) {
            if (src_size < bgr_size) {
                return false;
            }
            std::memcpy(bgr_dst, src, bgr_size);
        } else if (pixel_type == GX_PIXEL_FORMAT_MONO8) {
            const cv::Mat mono(frame.height, frame.width, CV_8UC1, const_cast<uint8_t*>(src));
            cv::cvtColor(mono, frame.bgr, cv::COLOR_GRAY2BGR);
        } else if (is_bayer8(pixel_type)) {
            const cv::Mat raw(frame.height, frame.width, CV_8UC1, const_cast<uint8_t*>(src));
            const int code = bayer_to_cv_bgr_code_from_pixel(pixel_type);
            bool gpu_ok = false;
            cv::Mat gpu_preview;
            if (!gpu_channel_order_decided_) {
                gpu_ok = tryGpuDemosaic(gx_frame, pixel_type, frame, bgr_dst);
                if (gpu_ok) {
                    gpu_preview = frame.bgr.clone();
                }
            } else {
                gpu_ok = tryGpuDemosaic(gx_frame, pixel_type, frame, nullptr);
            }

            // 统一输出 BGR，和 frame.bgr / imshow / 后续 OpenCV 处理保持一致。
            cv::cvtColor(raw, frame.bgr, code);
            if (config_.force_swap_rb) {
                cv::cvtColor(frame.bgr, frame.bgr, cv::COLOR_BGR2RGB);
            }

            if (gpu_ok && !gpu_channel_order_decided_) {
                cv::Mat cpu_ref;
                cv::cvtColor(raw, cpu_ref, code);
                cv::Mat swapped;
                cv::cvtColor(gpu_preview, swapped, cv::COLOR_BGR2RGB);
                const double err_cur = meanAbsColorDiff(gpu_preview, cpu_ref);
                const double err_swap = meanAbsColorDiff(swapped, cpu_ref);
                if (err_swap + 1.0 < err_cur) {
                    gpu_swap_rb_ = !gpu_swap_rb_;
#if defined(GALAXY_CAMERA_WITH_CUDA)
                    (void)tryGpuDemosaic(gx_frame, pixel_type, frame, nullptr);
#endif
                    std::cout << "GPU demosaic channel order auto-corrected: swap_rb="
                              << (gpu_swap_rb_ ? "1" : "0")
                              << " err_cur=" << err_cur
                              << " err_swap=" << err_swap << "\n";
                } else {
                    std::cout << "GPU demosaic channel order confirmed: swap_rb="
                              << (gpu_swap_rb_ ? "1" : "0")
                              << " err_cur=" << err_cur
                              << " err_swap=" << err_swap << "\n";
                }
                gpu_channel_order_decided_ = true;
            }
        } else {
            std::cerr << "Unsupported pixel format: " << frame.pixel_type << "\n";
            return false;
        }

        if (config_.rotate_180) {
            cv::flip(frame.bgr, frame.bgr, -1);
        }

        if (config_.undistort_enable && !undistort_map1_.empty() && !undistort_map2_.empty()) {
            std::memcpy(undistort_input_.data, bgr_dst, bgr_size);
            cv::Mat undistorted(frame.height, frame.width, CV_8UC3, undistort_buffer_.data());
            cv::remap(undistort_input_, undistorted, undistort_map1_, undistort_map2_, cv::INTER_LINEAR);
            std::memcpy(bgr_dst, undistort_buffer_.data(), bgr_size);
        }

#if defined(GALAXY_CAMERA_WITH_CUDA)
        // 若做了 CPU 侧后处理，异步回写 GPU，确保 TRT 读取的是同一最终图像。
        if (frame.gpu_valid && frame.gpu_bgr_ptr &&
            (config_.rotate_180 ||
             (config_.undistort_enable && !undistort_map1_.empty() && !undistort_map2_.empty()))) {
            cudaStream_t stream = static_cast<cudaStream_t>(frame.gpu_stream);
            cudaError_t status = cudaMemcpyAsync(frame.gpu_bgr_ptr, bgr_dst, bgr_size,
                                                 cudaMemcpyHostToDevice, stream);
            if (status != cudaSuccess) {
                frame.gpu_valid = false;
                frame.gpu_bgr_ptr = nullptr;
                frame.gpu_stride_bytes = 0;
                frame.gpu_stream = nullptr;
                std::cerr << "cudaMemcpyAsync(H2D postprocess) failed: "
                          << cudaGetErrorString(status) << "\n";
            }
        }

        // 当 GPU 去马赛克关闭或失败时，仍可把 CPU BGR 上传到 GPU，保持 TRT GPU 输入链路。
        if (!frame.gpu_valid && config_.gpu_pipeline_enable) {
            if (cudaSetDevice(config_.gpu_device_id) == cudaSuccess) {
                if (config_.gpu_stream_enable && !cuda_stream_) {
                    (void)cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking);
                }
                cudaStream_t stream = config_.gpu_stream_enable ? cuda_stream_ : nullptr;
                const int gpu_stride = static_cast<int>(frame.width) * 3;
                const size_t gpu_bytes = static_cast<size_t>(gpu_stride) * frame.height;
                if (npp_bgr_bytes_ < gpu_bytes) {
                    if (npp_bgr_dev_) {
                        cudaFree(npp_bgr_dev_);
                        npp_bgr_dev_ = nullptr;
                        npp_bgr_bytes_ = 0;
                    }
                    if (cudaMalloc(reinterpret_cast<void**>(&npp_bgr_dev_), gpu_bytes) == cudaSuccess) {
                        npp_bgr_bytes_ = gpu_bytes;
                    }
                }
                if (npp_bgr_dev_) {
                    cudaError_t status = cudaMemcpyAsync(npp_bgr_dev_, bgr_dst, gpu_bytes,
                                                         cudaMemcpyHostToDevice, stream);
                    if (status == cudaSuccess) {
                        frame.gpu_valid = true;
                        frame.gpu_bgr_ptr = npp_bgr_dev_;
                        frame.gpu_stride_bytes = gpu_stride;
                        frame.gpu_stream = stream;
                        if (!g_gpu_upload_logged.exchange(true)) {
                            std::cout << "GPU input path active via CPU->GPU upload\n";
                        }
                    }
                }
            }
        }
#endif
    }

    return true;
}

}  // namespace galaxy_camera
