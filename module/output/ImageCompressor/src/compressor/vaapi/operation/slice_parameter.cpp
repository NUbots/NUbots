#include "slice_parameter.h"

#include "../vaapi_error_category.hpp"

namespace module::output::compressor::vaapi::operation {

VABufferID slice_parameter(VADisplay dpy, VAContextID context, const bool& monochrome) {
    VAEncSliceParameterBufferJPEG params;
    params.restart_interval = 0;

    params.num_components = monochrome ? 1 : 3;

    params.components[0].component_selector = 1;
    params.components[0].dc_table_selector  = 0;
    params.components[0].ac_table_selector  = 0;

    if (!monochrome) {
        params.components[1].component_selector = 2;
        params.components[1].dc_table_selector  = 1;
        params.components[1].ac_table_selector  = 1;

        params.components[2].component_selector = 3;
        params.components[2].dc_table_selector  = 1;
        params.components[2].ac_table_selector  = 1;
    }

    // Upload to device
    VABufferID bufferid;
    VAStatus va_status = vaCreateBuffer(
        dpy, context, VAEncSliceParameterBufferType, sizeof(VAEncSliceParameterBufferJPEG), 1, &params, &bufferid);
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(va_status, vaapi_error_category(), "Error creating slice parameters buffer");
    }

    return bufferid;
}

}  // namespace module::output::compressor::vaapi::operation
