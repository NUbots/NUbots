#include "picture_parameter.h"

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
    VABufferID bufferid;
    VAStatus va_status = vaCreateBuffer(
        dpy, context, VAEncPictureParameterBufferType, sizeof(VAEncPictureParameterBufferJPEG), 1, &params, &bufferid);
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(va_status, vaapi_error_category(), "Error creating the picture parameter buffer");
    }

    return bufferid;
}

}  // namespace module::output::compressor::vaapi::operation
