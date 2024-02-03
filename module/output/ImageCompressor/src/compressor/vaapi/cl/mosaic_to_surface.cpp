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
#include "mosaic_to_surface.hpp"

#include <array>
#include <fmt/format.h>

#include "opencl_error_category.hpp"

namespace module::output::compressor::vaapi::cl {

    void mosaic_to_surface(const CompressionContext::OpenCLContext& context,
                           const cl::command_queue& command_queue,
                           const cl::kernel& mosaic,
                           const cl::mem& image,
                           const std::vector<uint8_t>& data,
                           const uint32_t& width,
                           const uint32_t& height,
                           const cl_mem& surface) {

        cl_int error = 0;
        cl_event ev  = nullptr;

        // Initialise the description of the image so we can copy it to the device
        std::array<size_t, 3> origin = {{0, 0, 0}};
        std::array<size_t, 3> region = {{size_t(width), size_t(height), 1}};

        // Upload our image to device memory
        error = clEnqueueWriteImage(command_queue,
                                    image,
                                    0,
                                    origin.data(),
                                    region.data(),
                                    0,
                                    0,
                                    data.data(),
                                    0,
                                    nullptr,
                                    &ev);
        cl::event copied_ev(ev, ::clReleaseEvent);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, cl::opencl_error_category(), "Error mapping the image into device memory");
        }

        // Acquire the surface from VAAPI
        error = context.acquire_surface(command_queue, 1, &surface, 0, nullptr, &ev);
        cl::event acquired_ev(ev, ::clReleaseEvent);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, cl::opencl_error_category(), "Error acquiring the VAAPI surface");
        }

        // Run the opencl kernel
        cl_mem arg = image;
        // NOLINTNEXTLINE(bugprone-sizeof-expression) clSetKernelArg actually expects the size of a pointer here
        error = ::clSetKernelArg(mosaic, 0, sizeof(arg), &arg);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, cl::opencl_error_category(), "Error setting the first kernel argument");
        }
        arg = surface;
        // NOLINTNEXTLINE(bugprone-sizeof-expression) clSetKernelArg actually expects the size of a pointer here
        error = ::clSetKernelArg(mosaic, 1, sizeof(arg), &arg);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, cl::opencl_error_category(), "Error setting the second kernel argument");
        }
        std::array<size_t, 2> offset         = {0, 0};
        std::array<size_t, 2> global_size    = {width, height};
        std::array<cl_event, 2> process_wait = {copied_ev, acquired_ev};
        error                                = ::clEnqueueNDRangeKernel(command_queue,
                                         mosaic,
                                         offset.size(),
                                         offset.data(),
                                         global_size.data(),
                                         nullptr,
                                         process_wait.size(),
                                         process_wait.data(),
                                         &ev);
        cl::event filled_ev(ev, ::clReleaseEvent);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, cl::opencl_error_category(), "Error running the mosaic kernel");
        }

        // Release the surface back to vaapi
        cl_event release_wait = filled_ev;
        context.release_surface(command_queue, 1, &surface, 1, &release_wait, &ev);
        cl::event finished_ev(ev, ::clReleaseEvent);
        if (error != CL_SUCCESS) {
            throw std::system_error(error, cl::opencl_error_category(), "Error acquiring the VAAPI surface");
        }

        // Wait for everything to be done
        cl_event finished_wait = finished_ev;
        clWaitForEvents(1, &finished_wait);
    }

}  // namespace module::output::compressor::vaapi::cl
