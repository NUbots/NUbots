#ifndef MODULE_VISION_CUDA_REPROJECTION_CUH
#define MODULE_VISION_CUDA_REPROJECTION_CUH

#include <cuda.h>
#include <samples/common/inc/helper_math.h>

extern "C" {
cudaError_t launchKernel(const unsigned char* input,
                         unsigned int image_format,
                         float radians_per_pixel,
                         uint2 input_dimensions,
                         uint2 output_dimensions,
                         float cam_focal_length_pixels,
                         unsigned char* output,
                         float* setup_ms,
                         float* kernel_ms,
                         float* total_ms);
}

#endif // MODULE_VISION_CUDA_REPROJECTION_CUH

