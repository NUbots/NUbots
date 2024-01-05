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
#include "create_surface.hpp"

#include <fmt/format.h>
#include <iostream>

#include "../vaapi_error_category.hpp"

#include "utility/vision/fourcc.hpp"

namespace module::output::compressor::vaapi::operation {

    int va_render_target_type_from_format(uint32_t format) {
        // Work out what surface type we will need to hold the image
        switch (format) {
            case utility::vision::fourcc("BGGR"):
            case utility::vision::fourcc("RGGB"):
            case utility::vision::fourcc("GRBG"):
            case utility::vision::fourcc("GBRG"):
            case utility::vision::fourcc("PBG8"):
            case utility::vision::fourcc("PRG8"):
            case utility::vision::fourcc("PGR8"):
            case utility::vision::fourcc("PGB8"):
            case utility::vision::fourcc("GRAY"):
            case utility::vision::fourcc("GREY"):
            case utility::vision::fourcc("Y8  "):
            case utility::vision::fourcc("Y16 "): return VA_RT_FORMAT_YUV400;

            case utility::vision::fourcc("BGR3"):
            case utility::vision::fourcc("BGR8"):
            case utility::vision::fourcc("BGRA"):
            case utility::vision::fourcc("RGB3"):
            case utility::vision::fourcc("RGB8"):
            case utility::vision::fourcc("RGBA"): return VA_RT_FORMAT_RGB32;
            default:
                throw std::runtime_error(
                    fmt::format("VAAPI Compressor does not support format {}", utility::vision::fourcc(format)));
        }
    }

    VASurfaceAttrib va_fourcc_from_format(uint32_t format) {
        VASurfaceAttrib fourcc;
        fourcc.type       = VASurfaceAttribPixelFormat;
        fourcc.flags      = VA_SURFACE_ATTRIB_SETTABLE;
        fourcc.value.type = VAGenericValueTypeInteger;

        switch (format) {
            case utility::vision::fourcc("BGGR"):
            case utility::vision::fourcc("RGGB"):
            case utility::vision::fourcc("GRBG"):
            case utility::vision::fourcc("GBRG"):
            case utility::vision::fourcc("PBG8"):
            case utility::vision::fourcc("PRG8"):
            case utility::vision::fourcc("PGR8"):
            case utility::vision::fourcc("PGB8"):
            case utility::vision::fourcc("GRAY"):
            case utility::vision::fourcc("GREY"):
            case utility::vision::fourcc("Y8  "):
            case utility::vision::fourcc("Y16 "): fourcc.value.value.i = VA_FOURCC_Y800; break;

            case utility::vision::fourcc("BGR3"):
            case utility::vision::fourcc("BGR8"):
            case utility::vision::fourcc("BGRA"): fourcc.value.value.i = VA_FOURCC_BGRA; break;
            case utility::vision::fourcc("RGB3"):
            case utility::vision::fourcc("RGB8"):
            case utility::vision::fourcc("RGBA"): fourcc.value.value.i = VA_FOURCC_RGBA; break;
            default:
                throw std::runtime_error(
                    fmt::format("VAAPI Compressor does not support format {}", utility::vision::fourcc(format)));
        }

        return fourcc;
    }

    VASurfaceID create_surface(VADisplay dpy, const uint32_t& width, const uint32_t& height, const uint32_t& format) {

        // Create Surface for the input picture
        VASurfaceAttrib va_fourcc = va_fourcc_from_format(format);
        VASurfaceID surface_id    = 0;
        VAStatus va_status        = vaCreateSurfaces(dpy,
                                              va_render_target_type_from_format(format),
                                              width,
                                              height,
                                              &surface_id,
                                              1,
                                              &va_fourcc,
                                              1);
        if (va_status != VA_STATUS_SUCCESS) {
            throw std::system_error(va_status, vaapi_error_category(), "Error creating a surface for the picture");
        }

        return surface_id;
    }

}  // namespace module::output::compressor::vaapi::operation
