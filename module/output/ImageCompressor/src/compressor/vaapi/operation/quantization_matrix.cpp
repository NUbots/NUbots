#include "quantization_matrix.h"

#include "../vaapi_error_category.hpp"
#include "jpeg_constants.h"

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
    VABufferID bufferid;
    VAStatus va_status =
        vaCreateBuffer(dpy, context, VAQMatrixBufferType, sizeof(VAQMatrixBufferJPEG), 1, &params, &bufferid);
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(va_status, vaapi_error_category(), "Error creating quantization matrix buffer");
    }

    return bufferid;
}

}  // namespace module::output::compressor::vaapi::operation
