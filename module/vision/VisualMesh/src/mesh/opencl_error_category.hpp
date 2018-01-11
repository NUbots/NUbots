#ifndef OPENCL_ERROR_CATEGORY
#define OPENCL_ERROR_CATEGORY

enum class opencl_error_code {
    SUCCESS                                   = CL_SUCCESS,
    DEVICE_NOT_FOUND                          = CL_DEVICE_NOT_FOUND,
    DEVICE_NOT_AVAILABLE                      = CL_DEVICE_NOT_AVAILABLE,
    COMPILER_NOT_AVAILABLE                    = CL_COMPILER_NOT_AVAILABLE,
    MEM_OBJECT_ALLOCATION_FAILURE             = CL_MEM_OBJECT_ALLOCATION_FAILURE,
    OUT_OF_RESOURCES                          = CL_OUT_OF_RESOURCES,
    OUT_OF_HOST_MEMORY                        = CL_OUT_OF_HOST_MEMORY,
    PROFILING_INFO_NOT_AVAILABLE              = CL_PROFILING_INFO_NOT_AVAILABLE,
    MEM_COPY_OVERLAP                          = CL_MEM_COPY_OVERLAP,
    IMAGE_FORMAT_MISMATCH                     = CL_IMAGE_FORMAT_MISMATCH,
    IMAGE_FORMAT_NOT_SUPPORTED                = CL_IMAGE_FORMAT_NOT_SUPPORTED,
    BUILD_PROGRAM_FAILURE                     = CL_BUILD_PROGRAM_FAILURE,
    MAP_FAILURE                               = CL_MAP_FAILURE,
    MISALIGNED_SUB_BUFFER_OFFSET              = CL_MISALIGNED_SUB_BUFFER_OFFSET,
    EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST = CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST,
    COMPILE_PROGRAM_FAILURE                   = CL_COMPILE_PROGRAM_FAILURE,
    LINKER_NOT_AVAILABLE                      = CL_LINKER_NOT_AVAILABLE,
    LINK_PROGRAM_FAILURE                      = CL_LINK_PROGRAM_FAILURE,
    DEVICE_PARTITION_FAILED                   = CL_DEVICE_PARTITION_FAILED,
    KERNEL_ARG_INFO_NOT_AVAILABLE             = CL_KERNEL_ARG_INFO_NOT_AVAILABLE,
    INVALID_VALUE                             = CL_INVALID_VALUE,
    INVALID_DEVICE_TYPE                       = CL_INVALID_DEVICE_TYPE,
    INVALID_PLATFORM                          = CL_INVALID_PLATFORM,
    INVALID_DEVICE                            = CL_INVALID_DEVICE,
    INVALID_CONTEXT                           = CL_INVALID_CONTEXT,
    INVALID_QUEUE_PROPERTIES                  = CL_INVALID_QUEUE_PROPERTIES,
    INVALID_COMMAND_QUEUE                     = CL_INVALID_COMMAND_QUEUE,
    INVALID_HOST_PTR                          = CL_INVALID_HOST_PTR,
    INVALID_MEM_OBJECT                        = CL_INVALID_MEM_OBJECT,
    INVALID_IMAGE_FORMAT_DESCRIPTOR           = CL_INVALID_IMAGE_FORMAT_DESCRIPTOR,
    INVALID_IMAGE_SIZE                        = CL_INVALID_IMAGE_SIZE,
    INVALID_SAMPLER                           = CL_INVALID_SAMPLER,
    INVALID_BINARY                            = CL_INVALID_BINARY,
    INVALID_BUILD_OPTIONS                     = CL_INVALID_BUILD_OPTIONS,
    INVALID_PROGRAM                           = CL_INVALID_PROGRAM,
    INVALID_PROGRAM_EXECUTABLE                = CL_INVALID_PROGRAM_EXECUTABLE,
    INVALID_KERNEL_NAME                       = CL_INVALID_KERNEL_NAME,
    INVALID_KERNEL_DEFINITION                 = CL_INVALID_KERNEL_DEFINITION,
    INVALID_KERNEL                            = CL_INVALID_KERNEL,
    INVALID_ARG_INDEX                         = CL_INVALID_ARG_INDEX,
    INVALID_ARG_VALUE                         = CL_INVALID_ARG_VALUE,
    INVALID_ARG_SIZE                          = CL_INVALID_ARG_SIZE,
    INVALID_KERNEL_ARGS                       = CL_INVALID_KERNEL_ARGS,
    INVALID_WORK_DIMENSION                    = CL_INVALID_WORK_DIMENSION,
    INVALID_WORK_GROUP_SIZE                   = CL_INVALID_WORK_GROUP_SIZE,
    INVALID_WORK_ITEM_SIZE                    = CL_INVALID_WORK_ITEM_SIZE,
    INVALID_GLOBAL_OFFSET                     = CL_INVALID_GLOBAL_OFFSET,
    INVALID_EVENT_WAIT_LIST                   = CL_INVALID_EVENT_WAIT_LIST,
    INVALID_EVENT                             = CL_INVALID_EVENT,
    INVALID_OPERATION                         = CL_INVALID_OPERATION,
    INVALID_GL_OBJECT                         = CL_INVALID_GL_OBJECT,
    INVALID_BUFFER_SIZE                       = CL_INVALID_BUFFER_SIZE,
    INVALID_MIP_LEVEL                         = CL_INVALID_MIP_LEVEL,
    INVALID_GLOBAL_WORK_SIZE                  = CL_INVALID_GLOBAL_WORK_SIZE,
    INVALID_PROPERTY                          = CL_INVALID_PROPERTY,
    INVALID_IMAGE_DESCRIPTOR                  = CL_INVALID_IMAGE_DESCRIPTOR,
    INVALID_COMPILER_OPTIONS                  = CL_INVALID_COMPILER_OPTIONS,
    INVALID_LINKER_OPTIONS                    = CL_INVALID_LINKER_OPTIONS,
    INVALID_DEVICE_PARTITION_COUNT            = CL_INVALID_DEVICE_PARTITION_COUNT,
    UNKNOWN
};

namespace std {
template <>
struct is_error_condition_enum<opencl_error_code> : public true_type {};
}  // namespace std


class opencl_error_category_t : public std::error_category {
public:
    virtual const char* name() const noexcept;

    virtual std::error_condition default_error_condition(int code) const noexcept;

    virtual bool equivalent(const std::error_code& code, int condition) const noexcept;

    virtual std::string message(int code) const noexcept;
};

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
    switch (code) {
        case CL_SUCCESS: return std::error_condition(opencl_error_code::SUCCESS);
        case CL_DEVICE_NOT_FOUND: return std::error_condition(opencl_error_code::DEVICE_NOT_FOUND);
        case CL_DEVICE_NOT_AVAILABLE: return std::error_condition(opencl_error_code::DEVICE_NOT_AVAILABLE);
        case CL_COMPILER_NOT_AVAILABLE: return std::error_condition(opencl_error_code::COMPILER_NOT_AVAILABLE);
        case CL_MEM_OBJECT_ALLOCATION_FAILURE:
            return std::error_condition(opencl_error_code::MEM_OBJECT_ALLOCATION_FAILURE);
        case CL_OUT_OF_RESOURCES: return std::error_condition(opencl_error_code::OUT_OF_RESOURCES);
        case CL_OUT_OF_HOST_MEMORY: return std::error_condition(opencl_error_code::OUT_OF_HOST_MEMORY);
        case CL_PROFILING_INFO_NOT_AVAILABLE:
            return std::error_condition(opencl_error_code::PROFILING_INFO_NOT_AVAILABLE);
        case CL_MEM_COPY_OVERLAP: return std::error_condition(opencl_error_code::MEM_COPY_OVERLAP);
        case CL_IMAGE_FORMAT_MISMATCH: return std::error_condition(opencl_error_code::IMAGE_FORMAT_MISMATCH);
        case CL_IMAGE_FORMAT_NOT_SUPPORTED: return std::error_condition(opencl_error_code::IMAGE_FORMAT_NOT_SUPPORTED);
        case CL_BUILD_PROGRAM_FAILURE: return std::error_condition(opencl_error_code::BUILD_PROGRAM_FAILURE);
        case CL_MAP_FAILURE: return std::error_condition(opencl_error_code::MAP_FAILURE);
        case CL_MISALIGNED_SUB_BUFFER_OFFSET:
            return std::error_condition(opencl_error_code::MISALIGNED_SUB_BUFFER_OFFSET);
        case CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST:
            return std::error_condition(opencl_error_code::EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST);
        case CL_COMPILE_PROGRAM_FAILURE: return std::error_condition(opencl_error_code::COMPILE_PROGRAM_FAILURE);
        case CL_LINKER_NOT_AVAILABLE: return std::error_condition(opencl_error_code::LINKER_NOT_AVAILABLE);
        case CL_LINK_PROGRAM_FAILURE: return std::error_condition(opencl_error_code::LINK_PROGRAM_FAILURE);
        case CL_DEVICE_PARTITION_FAILED: return std::error_condition(opencl_error_code::DEVICE_PARTITION_FAILED);
        case CL_KERNEL_ARG_INFO_NOT_AVAILABLE:
            return std::error_condition(opencl_error_code::KERNEL_ARG_INFO_NOT_AVAILABLE);
        case CL_INVALID_VALUE: return std::error_condition(opencl_error_code::INVALID_VALUE);
        case CL_INVALID_DEVICE_TYPE: return std::error_condition(opencl_error_code::INVALID_DEVICE_TYPE);
        case CL_INVALID_PLATFORM: return std::error_condition(opencl_error_code::INVALID_PLATFORM);
        case CL_INVALID_DEVICE: return std::error_condition(opencl_error_code::INVALID_DEVICE);
        case CL_INVALID_CONTEXT: return std::error_condition(opencl_error_code::INVALID_CONTEXT);
        case CL_INVALID_QUEUE_PROPERTIES: return std::error_condition(opencl_error_code::INVALID_QUEUE_PROPERTIES);
        case CL_INVALID_COMMAND_QUEUE: return std::error_condition(opencl_error_code::INVALID_COMMAND_QUEUE);
        case CL_INVALID_HOST_PTR: return std::error_condition(opencl_error_code::INVALID_HOST_PTR);
        case CL_INVALID_MEM_OBJECT: return std::error_condition(opencl_error_code::INVALID_MEM_OBJECT);
        case CL_INVALID_IMAGE_FORMAT_DESCRIPTOR:
            return std::error_condition(opencl_error_code::INVALID_IMAGE_FORMAT_DESCRIPTOR);
        case CL_INVALID_IMAGE_SIZE: return std::error_condition(opencl_error_code::INVALID_IMAGE_SIZE);
        case CL_INVALID_SAMPLER: return std::error_condition(opencl_error_code::INVALID_SAMPLER);
        case CL_INVALID_BINARY: return std::error_condition(opencl_error_code::INVALID_BINARY);
        case CL_INVALID_BUILD_OPTIONS: return std::error_condition(opencl_error_code::INVALID_BUILD_OPTIONS);
        case CL_INVALID_PROGRAM: return std::error_condition(opencl_error_code::INVALID_PROGRAM);
        case CL_INVALID_PROGRAM_EXECUTABLE: return std::error_condition(opencl_error_code::INVALID_PROGRAM_EXECUTABLE);
        case CL_INVALID_KERNEL_NAME: return std::error_condition(opencl_error_code::INVALID_KERNEL_NAME);
        case CL_INVALID_KERNEL_DEFINITION: return std::error_condition(opencl_error_code::INVALID_KERNEL_DEFINITION);
        case CL_INVALID_KERNEL: return std::error_condition(opencl_error_code::INVALID_KERNEL);
        case CL_INVALID_ARG_INDEX: return std::error_condition(opencl_error_code::INVALID_ARG_INDEX);
        case CL_INVALID_ARG_VALUE: return std::error_condition(opencl_error_code::INVALID_ARG_VALUE);
        case CL_INVALID_ARG_SIZE: return std::error_condition(opencl_error_code::INVALID_ARG_SIZE);
        case CL_INVALID_KERNEL_ARGS: return std::error_condition(opencl_error_code::INVALID_KERNEL_ARGS);
        case CL_INVALID_WORK_DIMENSION: return std::error_condition(opencl_error_code::INVALID_WORK_DIMENSION);
        case CL_INVALID_WORK_GROUP_SIZE: return std::error_condition(opencl_error_code::INVALID_WORK_GROUP_SIZE);
        case CL_INVALID_WORK_ITEM_SIZE: return std::error_condition(opencl_error_code::INVALID_WORK_ITEM_SIZE);
        case CL_INVALID_GLOBAL_OFFSET: return std::error_condition(opencl_error_code::INVALID_GLOBAL_OFFSET);
        case CL_INVALID_EVENT_WAIT_LIST: return std::error_condition(opencl_error_code::INVALID_EVENT_WAIT_LIST);
        case CL_INVALID_EVENT: return std::error_condition(opencl_error_code::INVALID_EVENT);
        case CL_INVALID_OPERATION: return std::error_condition(opencl_error_code::INVALID_OPERATION);
        case CL_INVALID_GL_OBJECT: return std::error_condition(opencl_error_code::INVALID_GL_OBJECT);
        case CL_INVALID_BUFFER_SIZE: return std::error_condition(opencl_error_code::INVALID_BUFFER_SIZE);
        case CL_INVALID_MIP_LEVEL: return std::error_condition(opencl_error_code::INVALID_MIP_LEVEL);
        case CL_INVALID_GLOBAL_WORK_SIZE: return std::error_condition(opencl_error_code::INVALID_GLOBAL_WORK_SIZE);
        case CL_INVALID_PROPERTY: return std::error_condition(opencl_error_code::INVALID_PROPERTY);
        case CL_INVALID_IMAGE_DESCRIPTOR: return std::error_condition(opencl_error_code::INVALID_IMAGE_DESCRIPTOR);
        case CL_INVALID_COMPILER_OPTIONS: return std::error_condition(opencl_error_code::INVALID_COMPILER_OPTIONS);
        case CL_INVALID_LINKER_OPTIONS: return std::error_condition(opencl_error_code::INVALID_LINKER_OPTIONS);
        case CL_INVALID_DEVICE_PARTITION_COUNT:
            return std::error_condition(opencl_error_code::INVALID_DEVICE_PARTITION_COUNT);
        default: return std::error_condition(opencl_error_code::UNKNOWN);
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
        default: return "Unknown error";
    }
}

#endif  // OPENCL_ERROR_CATEGORY
