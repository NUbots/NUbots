#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_PICTURE_PARAMETER_H
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_PICTURE_PARAMETER_H

#include <va/va.h>

namespace module::output::compressor::vaapi::operation {

VABufferID picture_parameter(VADisplay dpy,
                             VAContextID context,
                             VABufferID encoded,
                             const uint32_t& width,
                             const uint32_t& height,
                             const uint32_t& quality,
                             const bool& monochrome);

}  // namespace module::output::compressor::vaapi::operation

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_PICTURE_PARAMETER_H
