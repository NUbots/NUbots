#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_HUFFMAN_TABLE_HPP
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_HUFFMAN_TABLE_HPP

#include <va/va.h>

namespace module::output::compressor::vaapi::operation {

    VABufferID huffman_table(VADisplay dpy, VAContextID context, const bool& monochrome);

}  // namespace module::output::compressor::vaapi::operation

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_HUFFMAN_TABLE_HPP
