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
#include "slice_parameter.hpp"

#include "../vaapi_error_category.hpp"

namespace module::output::compressor::vaapi::operation {

    VABufferID slice_parameter(VADisplay dpy, VAContextID context, const bool& monochrome) {
        VAEncSliceParameterBufferJPEG params;
        params.restart_interval = 0;

        params.num_components = monochrome ? 1 : 3;

        params.components[0].component_selector = 1;
        params.components[0].dc_table_selector  = 0;
        params.components[0].ac_table_selector  = 0;

        if (!monochrome) {
            params.components[1].component_selector = 2;
            params.components[1].dc_table_selector  = 1;
            params.components[1].ac_table_selector  = 1;

            params.components[2].component_selector = 3;
            params.components[2].dc_table_selector  = 1;
            params.components[2].ac_table_selector  = 1;
        }

        // Upload to device
        VABufferID bufferid = 0;
        VAStatus va_status  = vaCreateBuffer(dpy,
                                            context,
                                            VAEncSliceParameterBufferType,
                                            sizeof(VAEncSliceParameterBufferJPEG),
                                            1,
                                            &params,
                                            &bufferid);
        if (va_status != VA_STATUS_SUCCESS) {
            throw std::system_error(va_status, vaapi_error_category(), "Error creating slice parameters buffer");
        }

        return bufferid;
    }

}  // namespace module::output::compressor::vaapi::operation
