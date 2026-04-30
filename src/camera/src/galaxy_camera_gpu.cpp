#include "galaxy_camera/galaxy_camera.h"
#include "galaxy_camera/gpu_demosaic_npp.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <iostream>
#include <string>

namespace galaxy_camera {
namespace {
std::atomic<bool> g_gpu_pipeline_logged{false};
std::atomic<bool> g_gpu_demosaic_logged{false};

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

#if defined(GALAXY_CAMERA_WITH_CUDA)
bool cuda_ok(cudaError_t status, const char* what) {
    if (status == cudaSuccess) {
        return true;
    }
    std::cerr << what << " failed: " << cudaGetErrorString(status)
              << " (" << static_cast<int>(status) << ")\n";
    return false;
}
#endif
}  // namespace

void GalaxyCamera::releaseGpuResources() {
#ifdef GALAXY_CAMERA_WITH_CUDA
    unregisterPinnedPool();
    if (cuda_stream_) {
        cudaStreamDestroy(cuda_stream_);
        cuda_stream_ = nullptr;
    }
    if (npp_raw_dev_) {
        cudaFree(npp_raw_dev_);
        npp_raw_dev_ = nullptr;
        npp_raw_bytes_ = 0;
    }
    if (npp_bgr_dev_) {
        cudaFree(npp_bgr_dev_);
        npp_bgr_dev_ = nullptr;
        npp_bgr_bytes_ = 0;
    }
#endif
}

void GalaxyCamera::configureGpuPipeline() {
#ifdef GALAXY_CAMERA_WITH_CUDA
    if (!config_.gpu_pipeline_enable) {
        if (cuda_stream_) {
            cudaStreamDestroy(cuda_stream_);
            cuda_stream_ = nullptr;
        }
        if (npp_raw_dev_) {
            cudaFree(npp_raw_dev_);
            npp_raw_dev_ = nullptr;
            npp_raw_bytes_ = 0;
        }
        if (npp_bgr_dev_) {
            cudaFree(npp_bgr_dev_);
            npp_bgr_dev_ = nullptr;
            npp_bgr_bytes_ = 0;
        }
        return;
    }

    if (!cuda_ok(cudaSetDevice(config_.gpu_device_id), "cudaSetDevice")) {
        return;
    }

    if (config_.gpu_stream_enable) {
        if (!cuda_stream_ &&
            !cuda_ok(cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking),
                     "cudaStreamCreateWithFlags")) {
            return;
        }
    } else if (cuda_stream_) {
        cudaStreamDestroy(cuda_stream_);
        cuda_stream_ = nullptr;
    }

    if (!g_gpu_pipeline_logged.exchange(true)) {
        std::cout << "GPU pipeline ready: device=" << config_.gpu_device_id
                  << " stream=" << (config_.gpu_stream_enable ? "on" : "off") << "\n";
    }
#endif
}

void GalaxyCamera::unregisterPinnedPool() {
#ifdef GALAXY_CAMERA_WITH_CUDA
    if (!bgr_pool_pinned_) {
        return;
    }
    for (auto& buf : bgr_pool_) {
        if (!buf.empty()) {
            cudaHostUnregister(buf.data());
        }
    }
    bgr_pool_pinned_ = false;
#endif
}

void GalaxyCamera::configurePinnedPool() {
#ifdef GALAXY_CAMERA_WITH_CUDA
    if (!config_.gpu_pipeline_enable || bgr_pool_pinned_) {
        return;
    }

    bool ok = true;
    for (auto& buf : bgr_pool_) {
        if (!buf.empty()) {
            if (cudaHostRegister(buf.data(), buf.size(), cudaHostRegisterPortable) != cudaSuccess) {
                ok = false;
                break;
            }
        }
    }
    if (ok) {
        bgr_pool_pinned_ = true;
    } else {
        unregisterPinnedPool();
        std::cerr << "Pinned pool registration failed, fallback to pageable memory\n";
    }
#endif
}

bool GalaxyCamera::tryGpuDemosaic(const GX_FRAME_DATA& gx_frame,
                                  int pixel_type,
                                  Frame& frame,
                                  uint8_t* bgr_dst) {
#if !defined(GALAXY_CAMERA_WITH_CUDA)
    (void)gx_frame;
    (void)pixel_type;
    (void)frame;
    (void)bgr_dst;
    return false;
#else
    if (!config_.gpu_pipeline_enable || !config_.gpu_demosaic_enable) {
        return false;
    }

    const int width = static_cast<int>(gx_frame.nWidth);
    const int height = static_cast<int>(gx_frame.nHeight);
    if (width <= 0 || height <= 0 || !gx_frame.pImgBuf) {
        return false;
    }

    std::string backend = toLower(config_.gpu_demosaic_backend);
    if (backend.empty()) {
        backend = "auto";
    }
#if defined(GALAXY_CAMERA_WITH_NPP)
    const bool use_npp = (backend == "auto" || backend == "npp");
#else
    const bool use_npp = false;
#endif
    if (!use_npp) {
        return false;
    }

#if !defined(GALAXY_CAMERA_WITH_NPP)
    return false;
#else
    if (!cuda_ok(cudaSetDevice(config_.gpu_device_id), "cudaSetDevice")) {
        return false;
    }
    if (config_.gpu_stream_enable && !cuda_stream_ &&
        !cuda_ok(cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking),
                 "cudaStreamCreateWithFlags")) {
        return false;
    }
    cudaStream_t stream = config_.gpu_stream_enable ? cuda_stream_ : nullptr;

    const size_t min_src_bytes = static_cast<size_t>(width) * static_cast<size_t>(height);
    const size_t frame_bytes = gx_frame.nImgSize > 0
                                   ? static_cast<size_t>(gx_frame.nImgSize)
                                   : min_src_bytes;
    if (frame_bytes < min_src_bytes) {
        return false;
    }

    size_t src_step = static_cast<size_t>(width);
    if (height > 0 && frame_bytes % static_cast<size_t>(height) == 0) {
        const size_t candidate = frame_bytes / static_cast<size_t>(height);
        if (candidate >= static_cast<size_t>(width)) {
            src_step = candidate;
        }
    }
    const size_t src_bytes = src_step * static_cast<size_t>(height);
    const int dst_step = width * 3;
    const size_t dst_bytes = static_cast<size_t>(dst_step) * static_cast<size_t>(height);

    if (npp_raw_bytes_ < src_bytes) {
        if (npp_raw_dev_) {
            cudaFree(npp_raw_dev_);
            npp_raw_dev_ = nullptr;
            npp_raw_bytes_ = 0;
        }
        if (!cuda_ok(cudaMalloc(reinterpret_cast<void**>(&npp_raw_dev_), src_bytes), "cudaMalloc(raw)")) {
            return false;
        }
        npp_raw_bytes_ = src_bytes;
    }
    if (npp_bgr_bytes_ < dst_bytes) {
        if (npp_bgr_dev_) {
            cudaFree(npp_bgr_dev_);
            npp_bgr_dev_ = nullptr;
            npp_bgr_bytes_ = 0;
        }
        if (!cuda_ok(cudaMalloc(reinterpret_cast<void**>(&npp_bgr_dev_), dst_bytes), "cudaMalloc(bgr)")) {
            return false;
        }
        npp_bgr_bytes_ = dst_bytes;
    }

    if (!cuda_ok(cudaMemcpyAsync(npp_raw_dev_, gx_frame.pImgBuf, src_bytes,
                                 cudaMemcpyHostToDevice, stream),
                 "cudaMemcpyAsync(H2D raw)")) {
        return false;
    }

    const int npp_bayer = nppBayerFromPixelType(static_cast<uint32_t>(pixel_type));
    std::string demosaic_err;
    if (!nppDemosaicBayer8(npp_raw_dev_,
                           static_cast<int>(src_step),
                           npp_bgr_dev_,
                           dst_step,
                           width,
                           height,
                           npp_bayer,
                           gpu_swap_rb_,
                           stream,
                           &demosaic_err)) {
        std::cerr << "nppDemosaicBayer8 failed: " << demosaic_err << "\n";
        return false;
    }

    frame.gpu_bgr_ptr = npp_bgr_dev_;
    frame.gpu_stride_bytes = dst_step;
    frame.gpu_valid = true;
    frame.gpu_stream = stream;

    if (bgr_dst) {
        if (!cuda_ok(cudaMemcpyAsync(bgr_dst, npp_bgr_dev_, dst_bytes,
                                     cudaMemcpyDeviceToHost, stream),
                     "cudaMemcpyAsync(D2H bgr)")) {
            frame.gpu_valid = false;
            return false;
        }
        if (!cuda_ok(cudaStreamSynchronize(stream), "cudaStreamSynchronize")) {
            frame.gpu_valid = false;
            return false;
        }
    }

    if (!g_gpu_demosaic_logged.exchange(true)) {
        std::cout << "GPU demosaic active: backend=npp"
                  << " width=" << width
                  << " height=" << height
                  << " stream=" << (config_.gpu_stream_enable ? "on" : "off")
                  << "\n";
    }
    return true;
#endif
#endif
}

}  // namespace galaxy_camera
