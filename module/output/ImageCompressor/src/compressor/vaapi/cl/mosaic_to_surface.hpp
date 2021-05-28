#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_MOSAIC_TO_SURFACE_HPP
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_MOSAIC_TO_SURFACE_HPP

#include <CL/cl.h>
#include <vector>

#include "../CompressionContext.hpp"

namespace module::output::compressor::vaapi::cl {

    void mosaic_to_surface(const CompressionContext::OpenCLContext& context,
                           const cl::command_queue& command_queue,
                           const cl::kernel& mosaic,
                           const cl::mem& image,
                           const std::vector<uint8_t>& data,
                           const uint32_t& width,
                           const uint32_t& height,
                           const cl_mem& surface);

}  // namespace module::output::compressor::vaapi::cl

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_MOSAIC_TO_SURFACE_HPP
