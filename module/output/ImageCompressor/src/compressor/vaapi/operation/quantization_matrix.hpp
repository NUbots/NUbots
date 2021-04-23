#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_QUANTIZATION_MATRIX_HPP
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_QUANTIZATION_MATRIX_HPP

#include <va/va.h>

namespace module::output::compressor::vaapi::operation {

    VABufferID quantization_matrix(VADisplay dpy, VAContextID context, const bool& monochrome);

}  // namespace module::output::compressor::vaapi::operation

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_QUANTIZATION_MATRIX_HPP
