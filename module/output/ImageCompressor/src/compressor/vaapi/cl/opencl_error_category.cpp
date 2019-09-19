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
#include "opencl_error_category.h"

namespace module::output::compressor::vaapi::cl {

const std::error_category& opencl_error_category() {
    static opencl_error_category_t instance;
    return instance;
}

std::error_condition make_error_condition(opencl_error_code e) {
    return std::error_condition(static_cast<int>(e), opencl_error_category());
}

const char* opencl_error_category_t::name() const noexcept {
    return "opencl_error_category";
}

std::error_condition opencl_error_category_t::default_error_condition(int code) const noexcept {
    using cle = opencl_error_code;
    switch (code) {
        case CL_SUCCESS: return std::error_condition(cle::SUCCESS);
        case CL_DEVICE_NOT_FOUND: return std::error_condition(cle::DEVICE_NOT_FOUND);
        case CL_DEVICE_NOT_AVAILABLE: return std::error_condition(cle::DEVICE_NOT_AVAILABLE);
        case CL_COMPILER_NOT_AVAILABLE: return std::error_condition(cle::COMPILER_NOT_AVAILABLE);
        case CL_MEM_OBJECT_ALLOCATION_FAILURE: return std::error_condition(cle::MEM_OBJECT_ALLOCATION_FAILURE);
        case CL_OUT_OF_RESOURCES: return std::error_condition(cle::OUT_OF_RESOURCES);
        case CL_OUT_OF_HOST_MEMORY: return std::error_condition(cle::OUT_OF_HOST_MEMORY);
        case CL_PROFILING_INFO_NOT_AVAILABLE: return std::error_condition(cle::PROFILING_INFO_NOT_AVAILABLE);
        case CL_MEM_COPY_OVERLAP: return std::error_condition(cle::MEM_COPY_OVERLAP);
        case CL_IMAGE_FORMAT_MISMATCH: return std::error_condition(cle::IMAGE_FORMAT_MISMATCH);
        case CL_IMAGE_FORMAT_NOT_SUPPORTED: return std::error_condition(cle::IMAGE_FORMAT_NOT_SUPPORTED);
        case CL_BUILD_PROGRAM_FAILURE: return std::error_condition(cle::BUILD_PROGRAM_FAILURE);
        case CL_MAP_FAILURE: return std::error_condition(cle::MAP_FAILURE);
        case CL_MISALIGNED_SUB_BUFFER_OFFSET: return std::error_condition(cle::MISALIGNED_SUB_BUFFER_OFFSET);
        case CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST:
            return std::error_condition(cle::EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST);
        case CL_COMPILE_PROGRAM_FAILURE: return std::error_condition(cle::COMPILE_PROGRAM_FAILURE);
        case CL_LINKER_NOT_AVAILABLE: return std::error_condition(cle::LINKER_NOT_AVAILABLE);
        case CL_LINK_PROGRAM_FAILURE: return std::error_condition(cle::LINK_PROGRAM_FAILURE);
        case CL_DEVICE_PARTITION_FAILED: return std::error_condition(cle::DEVICE_PARTITION_FAILED);
        case CL_KERNEL_ARG_INFO_NOT_AVAILABLE: return std::error_condition(cle::KERNEL_ARG_INFO_NOT_AVAILABLE);
        case CL_INVALID_VALUE: return std::error_condition(cle::INVALID_VALUE);
        case CL_INVALID_DEVICE_TYPE: return std::error_condition(cle::INVALID_DEVICE_TYPE);
        case CL_INVALID_PLATFORM: return std::error_condition(cle::INVALID_PLATFORM);
        case CL_INVALID_DEVICE: return std::error_condition(cle::INVALID_DEVICE);
        case CL_INVALID_CONTEXT: return std::error_condition(cle::INVALID_CONTEXT);
        case CL_INVALID_QUEUE_PROPERTIES: return std::error_condition(cle::INVALID_QUEUE_PROPERTIES);
        case CL_INVALID_COMMAND_QUEUE: return std::error_condition(cle::INVALID_COMMAND_QUEUE);
        case CL_INVALID_HOST_PTR: return std::error_condition(cle::INVALID_HOST_PTR);
        case CL_INVALID_MEM_OBJECT: return std::error_condition(cle::INVALID_MEM_OBJECT);
        case CL_INVALID_IMAGE_FORMAT_DESCRIPTOR: return std::error_condition(cle::INVALID_IMAGE_FORMAT_DESCRIPTOR);
        case CL_INVALID_IMAGE_SIZE: return std::error_condition(cle::INVALID_IMAGE_SIZE);
        case CL_INVALID_SAMPLER: return std::error_condition(cle::INVALID_SAMPLER);
        case CL_INVALID_BINARY: return std::error_condition(cle::INVALID_BINARY);
        case CL_INVALID_BUILD_OPTIONS: return std::error_condition(cle::INVALID_BUILD_OPTIONS);
        case CL_INVALID_PROGRAM: return std::error_condition(cle::INVALID_PROGRAM);
        case CL_INVALID_PROGRAM_EXECUTABLE: return std::error_condition(cle::INVALID_PROGRAM_EXECUTABLE);
        case CL_INVALID_KERNEL_NAME: return std::error_condition(cle::INVALID_KERNEL_NAME);
        case CL_INVALID_KERNEL_DEFINITION: return std::error_condition(cle::INVALID_KERNEL_DEFINITION);
        case CL_INVALID_KERNEL: return std::error_condition(cle::INVALID_KERNEL);
        case CL_INVALID_ARG_INDEX: return std::error_condition(cle::INVALID_ARG_INDEX);
        case CL_INVALID_ARG_VALUE: return std::error_condition(cle::INVALID_ARG_VALUE);
        case CL_INVALID_ARG_SIZE: return std::error_condition(cle::INVALID_ARG_SIZE);
        case CL_INVALID_KERNEL_ARGS: return std::error_condition(cle::INVALID_KERNEL_ARGS);
        case CL_INVALID_WORK_DIMENSION: return std::error_condition(cle::INVALID_WORK_DIMENSION);
        case CL_INVALID_WORK_GROUP_SIZE: return std::error_condition(cle::INVALID_WORK_GROUP_SIZE);
        case CL_INVALID_WORK_ITEM_SIZE: return std::error_condition(cle::INVALID_WORK_ITEM_SIZE);
        case CL_INVALID_GLOBAL_OFFSET: return std::error_condition(cle::INVALID_GLOBAL_OFFSET);
        case CL_INVALID_EVENT_WAIT_LIST: return std::error_condition(cle::INVALID_EVENT_WAIT_LIST);
        case CL_INVALID_EVENT: return std::error_condition(cle::INVALID_EVENT);
        case CL_INVALID_OPERATION: return std::error_condition(cle::INVALID_OPERATION);
        case CL_INVALID_GL_OBJECT: return std::error_condition(cle::INVALID_GL_OBJECT);
        case CL_INVALID_BUFFER_SIZE: return std::error_condition(cle::INVALID_BUFFER_SIZE);
        case CL_INVALID_MIP_LEVEL: return std::error_condition(cle::INVALID_MIP_LEVEL);
        case CL_INVALID_GLOBAL_WORK_SIZE: return std::error_condition(cle::INVALID_GLOBAL_WORK_SIZE);
        case CL_INVALID_PROPERTY: return std::error_condition(cle::INVALID_PROPERTY);
        case CL_INVALID_IMAGE_DESCRIPTOR: return std::error_condition(cle::INVALID_IMAGE_DESCRIPTOR);
        case CL_INVALID_COMPILER_OPTIONS: return std::error_condition(cle::INVALID_COMPILER_OPTIONS);
        case CL_INVALID_LINKER_OPTIONS: return std::error_condition(cle::INVALID_LINKER_OPTIONS);
        case CL_INVALID_DEVICE_PARTITION_COUNT: return std::error_condition(cle::INVALID_DEVICE_PARTITION_COUNT);
        case CL_INVALID_VA_API_MEDIA_ADAPTER_INTEL:
            return std::error_condition(cle::INVALID_VA_API_MEDIA_ADAPTER_INTEL);
        case CL_INVALID_VA_API_MEDIA_SURFACE_INTEL:
            return std::error_condition(cle::INVALID_VA_API_MEDIA_SURFACE_INTEL);
        case CL_VA_API_MEDIA_SURFACE_ALREADY_ACQUIRED_INTEL:
            return std::error_condition(cle::VA_API_MEDIA_SURFACE_ALREADY_ACQUIRED_INTEL);
        case CL_VA_API_MEDIA_SURFACE_NOT_ACQUIRED_INTEL:
            return std::error_condition(cle::VA_API_MEDIA_SURFACE_NOT_ACQUIRED_INTEL);
        default: return std::error_condition(cle::UNKNOWN);
    }
}

bool opencl_error_category_t::equivalent(const std::error_code& code, int condition) const noexcept {
    return *this == code.category() && static_cast<int>(default_error_condition(code.value()).value()) == condition;
}

std::string opencl_error_category_t::message(int code) const noexcept {
    switch (code) {
        case CL_SUCCESS: return "Success";
        case CL_DEVICE_NOT_FOUND: return "Device not found";
        case CL_DEVICE_NOT_AVAILABLE: return "Device not available";
        case CL_COMPILER_NOT_AVAILABLE: return "Compiler not available";
        case CL_MEM_OBJECT_ALLOCATION_FAILURE: return "Mem object allocation failure";
        case CL_OUT_OF_RESOURCES: return "Out of resources";
        case CL_OUT_OF_HOST_MEMORY: return "Out of host memory";
        case CL_PROFILING_INFO_NOT_AVAILABLE: return "Profiling info not available";
        case CL_MEM_COPY_OVERLAP: return "Mem copy overlap";
        case CL_IMAGE_FORMAT_MISMATCH: return "Image format mismatch";
        case CL_IMAGE_FORMAT_NOT_SUPPORTED: return "Image format not supported";
        case CL_BUILD_PROGRAM_FAILURE: return "Build program failure";
        case CL_MAP_FAILURE: return "Map failure";
        case CL_MISALIGNED_SUB_BUFFER_OFFSET: return "Misaligned sub buffer offset";
        case CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST: return "Exec status error for events in wait list";
        case CL_COMPILE_PROGRAM_FAILURE: return "Compile program failure";
        case CL_LINKER_NOT_AVAILABLE: return "Linker not available";
        case CL_LINK_PROGRAM_FAILURE: return "Link program failure";
        case CL_DEVICE_PARTITION_FAILED: return "Device partition failed";
        case CL_KERNEL_ARG_INFO_NOT_AVAILABLE: return "Kernel arg info not available";
        case CL_INVALID_VALUE: return "Invalid value";
        case CL_INVALID_DEVICE_TYPE: return "Invalid device type";
        case CL_INVALID_PLATFORM: return "Invalid platform";
        case CL_INVALID_DEVICE: return "Invalid device";
        case CL_INVALID_CONTEXT: return "Invalid context";
        case CL_INVALID_QUEUE_PROPERTIES: return "Invalid queue properties";
        case CL_INVALID_COMMAND_QUEUE: return "Invalid command queue";
        case CL_INVALID_HOST_PTR: return "Invalid host ptr";
        case CL_INVALID_MEM_OBJECT: return "Invalid mem object";
        case CL_INVALID_IMAGE_FORMAT_DESCRIPTOR: return "Invalid image format descriptor";
        case CL_INVALID_IMAGE_SIZE: return "Invalid image size";
        case CL_INVALID_SAMPLER: return "Invalid sampler";
        case CL_INVALID_BINARY: return "Invalid binary";
        case CL_INVALID_BUILD_OPTIONS: return "Invalid build options";
        case CL_INVALID_PROGRAM: return "Invalid program";
        case CL_INVALID_PROGRAM_EXECUTABLE: return "Invalid program executable";
        case CL_INVALID_KERNEL_NAME: return "Invalid kernel name";
        case CL_INVALID_KERNEL_DEFINITION: return "Invalid kernel definition";
        case CL_INVALID_KERNEL: return "Invalid kernel";
        case CL_INVALID_ARG_INDEX: return "Invalid arg index";
        case CL_INVALID_ARG_VALUE: return "Invalid arg value";
        case CL_INVALID_ARG_SIZE: return "Invalid arg size";
        case CL_INVALID_KERNEL_ARGS: return "Invalid kernel args";
        case CL_INVALID_WORK_DIMENSION: return "Invalid work dimension";
        case CL_INVALID_WORK_GROUP_SIZE: return "Invalid work group size";
        case CL_INVALID_WORK_ITEM_SIZE: return "Invalid work item size";
        case CL_INVALID_GLOBAL_OFFSET: return "Invalid global offset";
        case CL_INVALID_EVENT_WAIT_LIST: return "Invalid event wait list";
        case CL_INVALID_EVENT: return "Invalid event";
        case CL_INVALID_OPERATION: return "Invalid operation";
        case CL_INVALID_GL_OBJECT: return "Invalid gl object";
        case CL_INVALID_BUFFER_SIZE: return "Invalid buffer size";
        case CL_INVALID_MIP_LEVEL: return "Invalid mip level";
        case CL_INVALID_GLOBAL_WORK_SIZE: return "Invalid global work size";
        case CL_INVALID_PROPERTY: return "Invalid property";
        case CL_INVALID_IMAGE_DESCRIPTOR: return "Invalid image descriptor";
        case CL_INVALID_COMPILER_OPTIONS: return "Invalid compiler options";
        case CL_INVALID_LINKER_OPTIONS: return "Invalid linker options";
        case CL_INVALID_DEVICE_PARTITION_COUNT: return "Invalid device partition count";
        case CL_INVALID_VA_API_MEDIA_ADAPTER_INTEL: return "Invalid VAAPI media adapter";
        case CL_INVALID_VA_API_MEDIA_SURFACE_INTEL: return "Invalid VAAPI media surface";
        case CL_VA_API_MEDIA_SURFACE_ALREADY_ACQUIRED_INTEL: return "VAAPI media surface already acquired";
        case CL_VA_API_MEDIA_SURFACE_NOT_ACQUIRED_INTEL: return "VAAPI media surface not acquired";
        default: return "Unknown error";
    }
}

}  // namespace module::output::compressor::vaapi::cl
