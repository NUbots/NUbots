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
#include "quantization_matrix.hpp"

#include "../vaapi_error_category.hpp"
#include "jpeg_constants.hpp"

namespace module::output::compressor::vaapi::operation {

    VABufferID quantization_matrix(VADisplay dpy, VAContextID context, const bool& monochrome) {

        // Load the QMatrix
        VAQMatrixBufferJPEG params;
        params.load_lum_quantiser_matrix = 1;

        // LibVA expects the QM in zigzag order
        for (size_t i = 0; i < jpeg_luma_quant.size(); i++) {
            params.lum_quantiser_matrix[i] = jpeg_luma_quant[jpeg_zigzag[i]];
        }

        if (monochrome) {
            params.load_chroma_quantiser_matrix = 0;
        }
        else {
            params.load_chroma_quantiser_matrix = 1;
            for (size_t i = 0; i < jpeg_chroma_quant.size(); i++) {
                params.chroma_quantiser_matrix[i] = jpeg_chroma_quant[jpeg_zigzag[i]];
            }
        }

        // Upload to device
        VABufferID bufferid = 0;
        VAStatus va_status =
            vaCreateBuffer(dpy, context, VAQMatrixBufferType, sizeof(VAQMatrixBufferJPEG), 1, &params, &bufferid);
        if (va_status != VA_STATUS_SUCCESS) {
            throw std::system_error(va_status, vaapi_error_category(), "Error creating quantization matrix buffer");
        }

        return bufferid;
    }

}  // namespace module::output::compressor::vaapi::operation
