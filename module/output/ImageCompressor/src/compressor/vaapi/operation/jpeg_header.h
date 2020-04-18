#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_JPEG_HEADER_H
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_JPEG_HEADER_H

#include <va/va.h>

#include <utility>

namespace module::output::compressor::vaapi::operation {

std::pair<VABufferID, VABufferID> jpeg_header(VADisplay dpy,
                                              VAContextID context,
                                              uint32_t width,
                                              uint32_t height,
                                              const bool& monochrome,
                                              int quality);

}  // namespace module::output::compressor::vaapi::operation

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_JPEG_HEADER_H
