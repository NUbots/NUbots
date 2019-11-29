#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_CL_OPENCL_CONTEXT_FOR_DISPLAY_H
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_CL_OPENCL_CONTEXT_FOR_DISPLAY_H

#include <va/va.h>

#include "../CompressionContext.h"

namespace module::output::compressor::vaapi::cl {

CompressionContext::OpenCLContext opencl_context_for_display(VADisplay dpy);

}  // namespace module::output::compressor::vaapi::cl

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_CL_OPENCL_CONTEXT_FOR_DISPLAY_H
