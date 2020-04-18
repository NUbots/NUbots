/*
 * Copyright (C) 2017-2018 Trent Houliston <trent@houliston.me>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_CL_OPENCL_ERROR_CATEGORY_H
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_CL_OPENCL_ERROR_CATEGORY_H

#include <CL/cl.h>
#include <CL/cl_va_api_media_sharing_intel.h>

#include <system_error>

namespace module::output::compressor::vaapi::cl {
enum class opencl_error_code {
    SUCCESS                                     = CL_SUCCESS,
    DEVICE_NOT_FOUND                            = CL_DEVICE_NOT_FOUND,
    DEVICE_NOT_AVAILABLE                        = CL_DEVICE_NOT_AVAILABLE,
    COMPILER_NOT_AVAILABLE                      = CL_COMPILER_NOT_AVAILABLE,
    MEM_OBJECT_ALLOCATION_FAILURE               = CL_MEM_OBJECT_ALLOCATION_FAILURE,
    OUT_OF_RESOURCES                            = CL_OUT_OF_RESOURCES,
    OUT_OF_HOST_MEMORY                          = CL_OUT_OF_HOST_MEMORY,
    PROFILING_INFO_NOT_AVAILABLE                = CL_PROFILING_INFO_NOT_AVAILABLE,
    MEM_COPY_OVERLAP                            = CL_MEM_COPY_OVERLAP,
    IMAGE_FORMAT_MISMATCH                       = CL_IMAGE_FORMAT_MISMATCH,
    IMAGE_FORMAT_NOT_SUPPORTED                  = CL_IMAGE_FORMAT_NOT_SUPPORTED,
    BUILD_PROGRAM_FAILURE                       = CL_BUILD_PROGRAM_FAILURE,
    MAP_FAILURE                                 = CL_MAP_FAILURE,
    MISALIGNED_SUB_BUFFER_OFFSET                = CL_MISALIGNED_SUB_BUFFER_OFFSET,
    EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST   = CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST,
    COMPILE_PROGRAM_FAILURE                     = CL_COMPILE_PROGRAM_FAILURE,
    LINKER_NOT_AVAILABLE                        = CL_LINKER_NOT_AVAILABLE,
    LINK_PROGRAM_FAILURE                        = CL_LINK_PROGRAM_FAILURE,
    DEVICE_PARTITION_FAILED                     = CL_DEVICE_PARTITION_FAILED,
    KERNEL_ARG_INFO_NOT_AVAILABLE               = CL_KERNEL_ARG_INFO_NOT_AVAILABLE,
    INVALID_VALUE                               = CL_INVALID_VALUE,
    INVALID_DEVICE_TYPE                         = CL_INVALID_DEVICE_TYPE,
    INVALID_PLATFORM                            = CL_INVALID_PLATFORM,
    INVALID_DEVICE                              = CL_INVALID_DEVICE,
    INVALID_CONTEXT                             = CL_INVALID_CONTEXT,
    INVALID_QUEUE_PROPERTIES                    = CL_INVALID_QUEUE_PROPERTIES,
    INVALID_COMMAND_QUEUE                       = CL_INVALID_COMMAND_QUEUE,
    INVALID_HOST_PTR                            = CL_INVALID_HOST_PTR,
    INVALID_MEM_OBJECT                          = CL_INVALID_MEM_OBJECT,
    INVALID_IMAGE_FORMAT_DESCRIPTOR             = CL_INVALID_IMAGE_FORMAT_DESCRIPTOR,
    INVALID_IMAGE_SIZE                          = CL_INVALID_IMAGE_SIZE,
    INVALID_SAMPLER                             = CL_INVALID_SAMPLER,
    INVALID_BINARY                              = CL_INVALID_BINARY,
    INVALID_BUILD_OPTIONS                       = CL_INVALID_BUILD_OPTIONS,
    INVALID_PROGRAM                             = CL_INVALID_PROGRAM,
    INVALID_PROGRAM_EXECUTABLE                  = CL_INVALID_PROGRAM_EXECUTABLE,
    INVALID_KERNEL_NAME                         = CL_INVALID_KERNEL_NAME,
    INVALID_KERNEL_DEFINITION                   = CL_INVALID_KERNEL_DEFINITION,
    INVALID_KERNEL                              = CL_INVALID_KERNEL,
    INVALID_ARG_INDEX                           = CL_INVALID_ARG_INDEX,
    INVALID_ARG_VALUE                           = CL_INVALID_ARG_VALUE,
    INVALID_ARG_SIZE                            = CL_INVALID_ARG_SIZE,
    INVALID_KERNEL_ARGS                         = CL_INVALID_KERNEL_ARGS,
    INVALID_WORK_DIMENSION                      = CL_INVALID_WORK_DIMENSION,
    INVALID_WORK_GROUP_SIZE                     = CL_INVALID_WORK_GROUP_SIZE,
    INVALID_WORK_ITEM_SIZE                      = CL_INVALID_WORK_ITEM_SIZE,
    INVALID_GLOBAL_OFFSET                       = CL_INVALID_GLOBAL_OFFSET,
    INVALID_EVENT_WAIT_LIST                     = CL_INVALID_EVENT_WAIT_LIST,
    INVALID_EVENT                               = CL_INVALID_EVENT,
    INVALID_OPERATION                           = CL_INVALID_OPERATION,
    INVALID_GL_OBJECT                           = CL_INVALID_GL_OBJECT,
    INVALID_BUFFER_SIZE                         = CL_INVALID_BUFFER_SIZE,
    INVALID_MIP_LEVEL                           = CL_INVALID_MIP_LEVEL,
    INVALID_GLOBAL_WORK_SIZE                    = CL_INVALID_GLOBAL_WORK_SIZE,
    INVALID_PROPERTY                            = CL_INVALID_PROPERTY,
    INVALID_IMAGE_DESCRIPTOR                    = CL_INVALID_IMAGE_DESCRIPTOR,
    INVALID_COMPILER_OPTIONS                    = CL_INVALID_COMPILER_OPTIONS,
    INVALID_LINKER_OPTIONS                      = CL_INVALID_LINKER_OPTIONS,
    INVALID_DEVICE_PARTITION_COUNT              = CL_INVALID_DEVICE_PARTITION_COUNT,
    INVALID_VA_API_MEDIA_ADAPTER_INTEL          = CL_INVALID_VA_API_MEDIA_ADAPTER_INTEL,
    INVALID_VA_API_MEDIA_SURFACE_INTEL          = CL_INVALID_VA_API_MEDIA_SURFACE_INTEL,
    VA_API_MEDIA_SURFACE_ALREADY_ACQUIRED_INTEL = CL_VA_API_MEDIA_SURFACE_ALREADY_ACQUIRED_INTEL,
    VA_API_MEDIA_SURFACE_NOT_ACQUIRED_INTEL     = CL_VA_API_MEDIA_SURFACE_NOT_ACQUIRED_INTEL,
    UNKNOWN
};

}  // namespace module::output::compressor::vaapi::cl

namespace std {
template <>
struct is_error_condition_enum<module::output::compressor::vaapi::cl::opencl_error_code> : public true_type {};
}  // namespace std

namespace module::output::compressor::vaapi::cl {

class opencl_error_category_t : public std::error_category {
public:
    virtual const char* name() const noexcept;

    virtual std::error_condition default_error_condition(int code) const noexcept;

    virtual bool equivalent(const std::error_code& code, int condition) const noexcept;

    virtual std::string message(int code) const noexcept;
};

const std::error_category& opencl_error_category();

std::error_condition make_error_condition(opencl_error_code e);

}  // namespace module::output::compressor::vaapi::cl

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_CL_OPENCL_ERROR_CATEGORY_H
