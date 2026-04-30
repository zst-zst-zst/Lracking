#ifndef GALAXY_CAMERA_GPU_DEMOSAIC_NPP_H
#define GALAXY_CAMERA_GPU_DEMOSAIC_NPP_H

#include <cstdint>
#include <string>

#if defined(GALAXY_CAMERA_WITH_NPP)
#include <cuda_runtime.h>
#include <npp.h>
#endif

namespace galaxy_camera {

#if defined(GALAXY_CAMERA_WITH_NPP)
int nppBayerFromPattern(const std::string& pattern);
int nppBayerFromPixelType(uint32_t pixel_type);
bool nppDemosaicBayer8(const uint8_t* d_src,
                       int src_step,
                       uint8_t* d_dst,
                       int dst_step,
                       int width,
                       int height,
                       int npp_bayer,
                       bool swap_rb,
                       cudaStream_t stream,
                       std::string* err);
#endif

}  // namespace galaxy_camera

#endif  // GALAXY_CAMERA_GPU_DEMOSAIC_NPP_H
