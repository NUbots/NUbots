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
#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_COMPRESSOR_HPP
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_COMPRESSOR_HPP

#include <array>
#include <memory>
#include <va/va.h>

#include "../Compressor.hpp"
#include "CompressionContext.hpp"
#include "cl/wrapper.hpp"

namespace module::output::compressor::vaapi {

    struct Compressor : public compressor::Compressor {

        Compressor(const CompressionContext& cctx,
                   const uint32_t& width,
                   const uint32_t& height,
                   const uint32_t& format,
                   const int& quality);
        Compressor(const Compressor&)            = default;
        Compressor(Compressor&&)                 = default;
        Compressor& operator=(const Compressor&) = default;
        Compressor& operator=(Compressor&&)      = default;
        virtual ~Compressor();

        std::vector<uint8_t> compress(const std::vector<uint8_t>& data) override;

        // Contexts and buffers used for compressing images
        CompressionContext cctx;
        VASurfaceID surface{};
        VAContextID context{};
        VABufferID encoded{};
        std::array<VABufferID, 6> buffers{};

        // OpenCL things
        cl::command_queue command_queue;
        cl::program mosaic_program;
        cl::kernel mosaic_kernel;
        cl::mem cl_surface;
        cl::mem cl_image;

        // Parameters this compressor was created with
        uint32_t width;
        uint32_t height;
        uint32_t format;
    };

}  // namespace module::output::compressor::vaapi

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_COMPRESSOR_HPP
