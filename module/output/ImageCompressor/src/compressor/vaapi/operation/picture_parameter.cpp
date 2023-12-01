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
#include "picture_parameter.hpp"

#include "../vaapi_error_category.hpp"

namespace module::output::compressor::vaapi::operation {

    VABufferID picture_parameter(VADisplay dpy,
                                 VAContextID context,
                                 VABufferID encoded,
                                 const uint32_t& width,
                                 const uint32_t& height,
                                 const uint32_t& quality,
                                 const bool& monochrome) {

        // Initialize the picture parameter buffer
        VAEncPictureParameterBufferJPEG params;
        params.coded_buf                   = encoded;
        params.picture_width               = width;
        params.picture_height              = height;
        params.quality                     = quality;
        params.pic_flags.bits.profile      = 0;  // Baseline jpeg profile
        params.pic_flags.bits.progressive  = 0;  // Sequential encoding
        params.pic_flags.bits.huffman      = 1;  // Uses Huffman coding
        params.pic_flags.bits.interleaved  = 0;  // Non interleaved
        params.pic_flags.bits.differential = 0;  // non-Differential Encoding
        params.sample_bit_depth            = 8;  // only 8 bit sample depth is currently supported
        params.num_scan                    = 1;

        // Luma component
        params.num_components              = 1;
        params.component_id[0]             = 0;
        params.quantiser_table_selector[0] = 0;

        // RGB
        if (!monochrome) {
            // Luma + 2 chroma
            params.num_components = 3;
            // U and V use the same quantisation table
            params.component_id[1]             = 1;
            params.quantiser_table_selector[1] = 1;
            params.component_id[2]             = 2;
            params.quantiser_table_selector[2] = 1;
        }

        // Upload to device
        VABufferID bufferid = 0;
        VAStatus va_status  = vaCreateBuffer(dpy,
                                            context,
                                            VAEncPictureParameterBufferType,
                                            sizeof(VAEncPictureParameterBufferJPEG),
                                            1,
                                            &params,
                                            &bufferid);
        if (va_status != VA_STATUS_SUCCESS) {
            throw std::system_error(va_status, vaapi_error_category(), "Error creating the picture parameter buffer");
        }

        return bufferid;
    }

}  // namespace module::output::compressor::vaapi::operation
