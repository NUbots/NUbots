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
#include "opencl_context_for_display.hpp"

#include <CL/cl.h>
#include <array>
#include <va/va.h>
#include <vector>

#include "opencl_error_category.hpp"

namespace module::output::compressor::vaapi::cl {

    CompressionContext::OpenCLContext opencl_context_for_display(VADisplay dpy) {

        // Get our platforms
        cl_uint platform_count = 0;
        ::clGetPlatformIDs(0, nullptr, &platform_count);
        std::vector<cl_platform_id> platform_ids(platform_count);
        ::clGetPlatformIDs(platform_ids.size(), platform_ids.data(), nullptr);

        for (const auto& platform_id : platform_ids) {
            // Get the extensions supported by this platform
            size_t len = 0;
            ::clGetPlatformInfo(platform_id, CL_PLATFORM_EXTENSIONS, 0, nullptr, &len);
            std::vector<char> extensions(len);
            // get string with extension names
            ::clGetPlatformInfo(platform_id, CL_PLATFORM_EXTENSIONS, len, extensions.data(), nullptr);

            // Check if this platform supports the vaapi extension
            if (std::string(extensions.data()).find("cl_intel_va_api_media_sharing") != std::string::npos) {

                // Get the extension function for this platform
                auto devices_from_display = reinterpret_cast<clGetDeviceIDsFromVA_APIMediaAdapterINTEL_fn>(
                    clGetExtensionFunctionAddressForPlatform(platform_id, "clGetDeviceIDsFromVA_APIMediaAdapterINTEL"));
                if (devices_from_display != nullptr) {
                    cl_uint n_devices = 0;
                    cl_int err        = devices_from_display(platform_id,
                                                      CL_VA_API_DISPLAY_INTEL,
                                                      dpy,
                                                      CL_PREFERRED_DEVICES_FOR_VA_API_INTEL,
                                                      0,
                                                      nullptr,
                                                      &n_devices);

                    // If there was a device on this platform get it
                    if (err == CL_SUCCESS && n_devices > 0) {
                        // Get the devices
                        std::vector<cl_device_id> device_ids(n_devices);
                        err = devices_from_display(platform_id,
                                                   CL_VA_API_DISPLAY_INTEL,
                                                   dpy,
                                                   CL_PREFERRED_DEVICES_FOR_VA_API_INTEL,
                                                   n_devices,
                                                   device_ids.data(),
                                                   nullptr);

                        // the properties list that is used to create context
                        std::array<cl_context_properties, 5> props = {CL_CONTEXT_VA_API_DISPLAY_INTEL,
                                                                      reinterpret_cast<cl_context_properties>(dpy),
                                                                      CL_CONTEXT_INTEROP_USER_SYNC,
                                                                      CL_TRUE,
                                                                      0};

                        // Grab the other functions that we will need
                        auto mem_from_surface = reinterpret_cast<clCreateFromVA_APIMediaSurfaceINTEL_fn>(
                            clGetExtensionFunctionAddressForPlatform(platform_id,
                                                                     "clCreateFromVA_APIMediaSurfaceINTEL"));
                        auto acquire_surface = reinterpret_cast<clEnqueueAcquireVA_APIMediaSurfacesINTEL_fn>(
                            clGetExtensionFunctionAddressForPlatform(platform_id,
                                                                     "clEnqueueAcquireVA_APIMediaSurfacesINTEL"));
                        auto release_surface = reinterpret_cast<clEnqueueReleaseVA_APIMediaSurfacesINTEL_fn>(
                            clGetExtensionFunctionAddressForPlatform(platform_id,
                                                                     "clEnqueueReleaseVA_APIMediaSurfacesINTEL"));

                        if (mem_from_surface == nullptr || acquire_surface == nullptr || release_surface == nullptr) {
                            throw std::runtime_error(
                                "Unable to get the function pointer for converting surfaces to mem objects");
                        }

                        // create context for the first device only
                        cl::context context =
                            cl::context(clCreateContext(props.data(), 1, device_ids.data(), nullptr, nullptr, &err),
                                        ::clReleaseContext);
                        if (err != CL_SUCCESS) {
                            throw std::system_error(err,
                                                    cl::opencl_error_category(),
                                                    "Error while trying to create an OpenCL context");
                        }

                        return CompressionContext::OpenCLContext{
                            device_ids.front(),
                            context,
                            mem_from_surface,
                            acquire_surface,
                            release_surface,
                        };
                    }
                }
            }
        }

        // If we got here, we failed
        throw std::runtime_error("Unable to find an OpenCL platform that is associated with this vaapi device");
    }

}  // namespace module::output::compressor::vaapi::cl
