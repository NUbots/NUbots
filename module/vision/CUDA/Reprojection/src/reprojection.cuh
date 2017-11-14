#ifndef MODULE_VISION_CUDA_REPROJECTION_CUH
#define MODULE_VISION_CUDA_REPROJECTION_CUH

#include <cuda.h>
// #include <cuda_runtime.h>
// #include <cuda_runtime_api.h>
// #include <driver_types.h>
#include <samples/common/inc/helper_math.h>

extern "C" {
void launchKernel(const unsigned char* input,
                  unsigned int image_format,
                  float radians_per_pixel,
                  uint2 input_dimensions,
                  uint2 output_dimensions,
                  float cam_focal_length_pixels,
                  unsigned char* output);
}

#endif // MODULE_VISION_CUDA_REPROJECTION_CUH

