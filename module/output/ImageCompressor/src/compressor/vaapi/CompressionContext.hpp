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
#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_COMPRESSIONCONTEXT_HPP
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_COMPRESSIONCONTEXT_HPP

#include <CL/cl.h>
#include <CL/cl_va_api_media_sharing_intel.h>
#include <va/va.h>

#include "cl/wrapper.hpp"

namespace module::output::compressor::vaapi {

    struct CompressionContext {

        struct VAAPIContext {
            /// A shared pointer to the display object (gets terminated on destruction)
            VADisplay dpy;
            /// The configuration we made for compressing jpegs
            VAConfigID config;
        } va{};

        struct OpenCLContext {
            /// The OpenCL device we are executing on
            cl_device_id device{};
            /// The OpenCL context we will do operations on
            cl::context context;
            /// The OpenCL extension function that maps surfaces into opencl mem objects
            clCreateFromVA_APIMediaSurfaceINTEL_fn mem_from_surface{};
            /// The OpenCL extension function that acquires surfaces from VAAPI
            clEnqueueAcquireVA_APIMediaSurfacesINTEL_fn acquire_surface{};
            /// The OpenCL extension function that releases surfaces back to VAAPI
            clEnqueueReleaseVA_APIMediaSurfacesINTEL_fn release_surface{};
        } cl;

        /// The quality that this compressor is configured for
        int quality{};
    };

}  // namespace module::output::compressor::vaapi

#endif  //  MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_COMPRESSIONCONTEXT_HPP
