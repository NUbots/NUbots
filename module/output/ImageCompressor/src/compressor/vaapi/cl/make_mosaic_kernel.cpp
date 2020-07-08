#include "make_mosaic_kernel.h"

#include <fmt/format.h>

#include "../../mosaic.h"
#include "opencl_error_category.h"

namespace module::output::compressor::vaapi::cl {
std::pair<cl::program, cl::kernel> make_mosaic_kernel(CompressionContext::OpenCLContext context,
                                                      const uint32_t& width,
                                                      const uint32_t& height,
                                                      const uint32_t& format) {
    // Make the OpenCL source
    int mosaic_size    = mosaic::mosaic_size(format);
    std::string source = fmt::format(R"(
                    const sampler_t s  = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP | CLK_FILTER_NEAREST;
                    kernel void
                        mosaic_to_surface(read_only image2d_t input, write_only image2d_t surface) {{
                        const int2 src = (int2)(get_global_id(0), get_global_id(1));
                        const int2 dst = (src / {0}) + (src % {0}) * (int2)({1}, {2});
                        write_imagef(surface, dst, read_imagef(input, s, src));
                    }}
                    )",
                                     mosaic_size,
                                     width / mosaic_size,
                                     height / mosaic_size);

    cl_int error;
    const char* cstr = source.c_str();
    size_t csize     = source.size();
    cl::program program(::clCreateProgramWithSource(context.context, 1, &cstr, &csize, &error), ::clReleaseProgram);
    if (error != CL_SUCCESS) {
        throw std::system_error(error, cl::opencl_error_category(), "Error adding sources to projection program");
    }

    // Compile the program
    error = ::clBuildProgram(program, 0, nullptr, nullptr, nullptr, nullptr);
    if (error != CL_SUCCESS) {
        // Get program build log
        size_t used = 0;
        ::clGetProgramBuildInfo(program, context.device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &used);
        std::vector<char> log(used);
        ::clGetProgramBuildInfo(program, context.device, CL_PROGRAM_BUILD_LOG, log.size(), log.data(), &used);

        // Throw an error with the build log
        throw std::system_error(error,
                                cl::opencl_error_category(),
                                "Error building projection program\n" + std::string(log.begin(), log.begin() + used));
    }

    cl::kernel kernel(::clCreateKernel(program, "mosaic_to_surface", &error), ::clReleaseKernel);
    if (error != CL_SUCCESS) {
        throw std::system_error(error, cl::opencl_error_category(), "Error getting the mosaic_to_surface kernel");
    }

    return std::make_pair(program, kernel);
}

}  // namespace module::output::compressor::vaapi::cl
