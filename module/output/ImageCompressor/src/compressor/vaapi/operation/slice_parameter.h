#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_SLICE_PARAMETER_H
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_SLICE_PARAMETER_H

#include <va/va.h>


namespace module::output::compressor::vaapi::operation {

VABufferID slice_parameter(VADisplay dpy, VAContextID context, const bool& monochrome);

}  // namespace module::output::compressor::vaapi::operation

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_SLICE_PARAMETER_H
