#include "huffman_table.h"

#include <cstring>

#include "../vaapi_error_category.hpp"
#include "jpeg_constants.h"

namespace module::output::compressor::vaapi::operation {

VABufferID huffman_table(VADisplay dpy, VAContextID context, const bool& monochrome) {

    VAHuffmanTableBufferJPEGBaseline params;

    params.load_huffman_table[0] = 1;                   // Load Luma Hufftable
    params.load_huffman_table[1] = monochrome ? 0 : 1;  // Load Chroma Hufftable if colour

    // Load Luma hufftable values
    // Load DC codes
    std::memcpy(params.huffman_table[0].num_dc_codes, jpeg_hufftable_luma_dc.data() + 1, 16);
    // Load DC Values
    std::memcpy(params.huffman_table[0].dc_values, jpeg_hufftable_luma_dc.data() + 17, 12);
    // Load AC codes
    std::memcpy(params.huffman_table[0].num_ac_codes, jpeg_hufftable_luma_ac.data() + 1, 16);
    // Load AC Values
    std::memcpy(params.huffman_table[0].ac_values, jpeg_hufftable_luma_ac.data() + 17, 162);
    std::memset(params.huffman_table[0].pad, 0, 2);

    // Load Chroma hufftable values if needed
    if (!monochrome) {
        // Load DC codes
        std::memcpy(params.huffman_table[1].num_dc_codes, jpeg_hufftable_chroma_dc.data() + 1, 16);
        // Load DC Values
        std::memcpy(params.huffman_table[1].dc_values, jpeg_hufftable_chroma_dc.data() + 17, 12);
        // Load AC codes
        std::memcpy(params.huffman_table[1].num_ac_codes, jpeg_hufftable_chroma_ac.data() + 1, 16);
        // Load AC Values
        std::memcpy(params.huffman_table[1].ac_values, jpeg_hufftable_chroma_ac.data() + 17, 162);
        std::memset(params.huffman_table[1].pad, 0, 2);
    }

    VABufferID bufferid;
    VAStatus va_status = vaCreateBuffer(
        dpy, context, VAHuffmanTableBufferType, sizeof(VAHuffmanTableBufferJPEGBaseline), 1, &params, &bufferid);
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(va_status, vaapi_error_category(), "Error creating huffman table buffer");
    }

    return bufferid;
}

}  // namespace module::output::compressor::vaapi::operation
