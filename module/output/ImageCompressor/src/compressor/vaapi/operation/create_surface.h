#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_CREATE_SURFACE_H
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_CREATE_SURFACE_H

#include <va/va.h>

namespace module::output::compressor::vaapi::operation {

int va_render_target_type_from_format(uint32_t format);
VASurfaceID create_surface(VADisplay dpy, const uint32_t& width, const uint32_t& height, const uint32_t& format);

}  // namespace module::output::compressor::vaapi::operation

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_CREATE_SURFACE_H
