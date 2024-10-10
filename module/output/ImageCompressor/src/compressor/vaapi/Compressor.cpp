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
#include "Compressor.hpp"

#include <cstring>
#include <fcntl.h>
#include <system_error>
#include <unistd.h>
#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_enc_jpeg.h>

#include "cl/make_mosaic_kernel.hpp"
#include "cl/mosaic_to_surface.hpp"
#include "cl/opencl_error_category.hpp"
#include "operation/create_surface.hpp"
#include "operation/huffman_table.hpp"
#include "operation/jpeg_header.hpp"
#include "operation/picture_parameter.hpp"
#include "operation/quantization_matrix.hpp"
#include "operation/slice_parameter.hpp"
#include "operation/upload_to_surface.hpp"
#include "vaapi_error_category.hpp"

#include "utility/vision/fourcc.hpp"
#include "utility/vision/mosaic.hpp"

namespace module::output::compressor::vaapi {

    Compressor::Compressor(const CompressionContext& cctx,
                           const uint32_t& width,
                           const uint32_t& height,
                           const uint32_t& format,
                           const int& quality)
        : cctx(cctx)
        , surface{operation::create_surface(cctx.va.dpy, width, height, format)}
        , width(width)
        , height(height)
        , format(format) {

        VAStatus va_status = 0;

        // If we are storing in a YUV400 it is a monochrome image
        bool monochrome = operation::va_render_target_type_from_format(format) == VA_RT_FORMAT_YUV400;

        // If we have a mosaic pattern we need to setup OpenCL to do the mosaic permutation
        if (utility::vision::Mosaic::size(format) > 1) {
            cl_int error = 0;

            // Create a command queue to run the mosaic kernels on
            command_queue = cl::command_queue(
                clCreateCommandQueue(cctx.cl.context, cctx.cl.device, CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE, &error),
                ::clReleaseCommandQueue);

            if (error != CL_SUCCESS) {
                throw std::system_error(error, cl::opencl_error_category(), "Error creating the command queue");
            }

            cl_surface = cl::mem(cctx.cl.mem_from_surface(cctx.cl.context, CL_MEM_WRITE_ONLY, &surface, 0, &error),
                                 ::clReleaseMemObject);
            if (error != CL_SUCCESS) {
                throw std::system_error(error,
                                        cl::opencl_error_category(),
                                        "Unable to create an OpenCL memory object for the surface");
            }

            // Compile the kernel that will make the transform
            std::tie(mosaic_program, mosaic_kernel) = cl::make_mosaic_kernel(cctx.cl, width, height, format);

            // Make some memory that will hold where the image goes
            cl_image_format fmt{CL_R, CL_UNORM_INT8};
            cl_image_desc desc{CL_MEM_OBJECT_IMAGE2D, size_t(width), size_t(height), 1, 1, 0, 0, 0, 0, nullptr};
            cl_image = cl::mem(::clCreateImage(cctx.cl.context, CL_MEM_READ_ONLY, &fmt, &desc, nullptr, &error),
                               ::clReleaseMemObject);
            if (error != CL_SUCCESS) {
                throw std::system_error(error, cl::opencl_error_category(), "Unable to make an input image for OpenCL");
            }
        }

        // Create Context for the encode pipe
        va_status = vaCreateContext(cctx.va.dpy,
                                    cctx.va.config,
                                    int(width),
                                    int(height),
                                    VA_PROGRESSIVE,
                                    &surface,
                                    1,
                                    &context);
        if (va_status != VA_STATUS_SUCCESS) {
            throw std::system_error(va_status, vaapi_error_category(), "Error while creating a context");
        }

        // Create buffer for Encoded data to be stored
        va_status = vaCreateBuffer(cctx.va.dpy,
                                   context,
                                   VAEncCodedBufferType,
                                   width * height * (monochrome ? 1 : 4),
                                   1,
                                   nullptr,
                                   &encoded);
        if (va_status != VA_STATUS_SUCCESS) {
            throw std::system_error(va_status, vaapi_error_category(), "Error creating buffer for encoded data");
        }

        VABufferID picture_parameter =
            operation::picture_parameter(cctx.va.dpy, context, encoded, width, height, quality, monochrome);

        // Initialise the buffers
        VABufferID quantization_matrix = operation::quantization_matrix(cctx.va.dpy, context, monochrome);
        VABufferID huffman_table       = operation::huffman_table(cctx.va.dpy, context, monochrome);
        VABufferID slice               = operation::slice_parameter(cctx.va.dpy, context, monochrome);
        auto raw_header = operation::jpeg_header(cctx.va.dpy, context, width, height, monochrome, quality);

        buffers = {{picture_parameter, quantization_matrix, huffman_table, slice, raw_header.first, raw_header.second}};
    }

    Compressor::~Compressor() {
        vaDestroyBuffer(cctx.va.dpy, encoded);
        for (auto& b : buffers) {
            vaDestroyBuffer(cctx.va.dpy, b);
        }
        vaDestroySurfaces(cctx.va.dpy, &surface, 1);
        vaDestroyContext(cctx.va.dpy, context);
    }

    std::vector<uint8_t> Compressor::compress(const std::vector<uint8_t>& data) {

        if (utility::vision::Mosaic::size(format) > 1) {
            // Use OpenCL to permute the mosaic into the surface
            cl::mosaic_to_surface(cctx.cl, command_queue, mosaic_kernel, cl_image, data, width, height, cl_surface);
        }
        else {
            // Upload the image to the surface the normal way
            operation::upload_to_surface(cctx.va.dpy, data, width, height, format, surface);
        }

        // Render the picture
        VAStatus va_status = vaBeginPicture(cctx.va.dpy, context, surface);
        if (va_status != VA_STATUS_SUCCESS) {
            throw std::system_error(va_status, vaapi_error_category(), "Error beginning picture rendering");
        }
        va_status = vaRenderPicture(cctx.va.dpy, context, buffers.data(), int(buffers.size()));
        if (va_status != VA_STATUS_SUCCESS) {
            throw std::system_error(va_status, vaapi_error_category(), "Error pushing buffers to the render context");
        }
        va_status = vaEndPicture(cctx.va.dpy, context);
        if (va_status != VA_STATUS_SUCCESS) {
            throw std::system_error(va_status, vaapi_error_category(), "Error ending the picture rendering");
        }

        // Wait until the surface has finished rendering
        va_status = vaSyncSurface(cctx.va.dpy, surface);
        if (va_status != VA_STATUS_SUCCESS) {
            throw std::system_error(va_status, vaapi_error_category(), "Error syncing the surface state");
        }

        // Map the encoded buffer into user space
        VACodedBufferSegment* encoded_status = nullptr;
        va_status = vaMapBuffer(cctx.va.dpy, encoded, reinterpret_cast<void**>(&encoded_status));
        if (va_status != VA_STATUS_SUCCESS) {
            throw std::system_error(va_status,
                                    vaapi_error_category(),
                                    "Error mapping the encoded buffer into user space");
        }

        // Copy the data into the output vector
        const auto* buffer = reinterpret_cast<const uint8_t*>(encoded_status->buf);
        std::vector<uint8_t> output(buffer, buffer + encoded_status->size);

        // Unmap the buffer back to device space
        va_status = vaUnmapBuffer(cctx.va.dpy, encoded);
        if (va_status != VA_STATUS_SUCCESS) {
            throw std::system_error(va_status,
                                    vaapi_error_category(),
                                    "Error unmapping the encoded buffer back to device space");
        }

        return output;
    }

}  // namespace module::output::compressor::vaapi
