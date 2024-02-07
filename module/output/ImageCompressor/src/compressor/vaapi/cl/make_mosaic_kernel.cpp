/*
 * MIT License
 *
 * Copyright (c) 2020 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "make_mosaic_kernel.hpp"

#include <fmt/format.h>

#include "opencl_error_category.hpp"

#include "utility/vision/mosaic.hpp"

namespace module::output::compressor::vaapi::cl {
    std::pair<cl::program, cl::kernel> make_mosaic_kernel(const CompressionContext::OpenCLContext& context,
                                                          const uint32_t& width,
                                                          const uint32_t& height,
                                                          const uint32_t& format) {
        // Make the OpenCL source
        int mosaic_size    = utility::vision::Mosaic::size(format);
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

        cl_int error     = 0;
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
            throw std::system_error(
                error,
                cl::opencl_error_category(),
                "Error building projection program\n" + std::string(log.begin(), log.begin() + ssize_t(used)));
        }

        cl::kernel kernel(::clCreateKernel(program, "mosaic_to_surface", &error), ::clReleaseKernel);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, cl::opencl_error_category(), "Error getting the mosaic_to_surface kernel");
        }

        return std::make_pair(program, kernel);
    }

}  // namespace module::output::compressor::vaapi::cl
