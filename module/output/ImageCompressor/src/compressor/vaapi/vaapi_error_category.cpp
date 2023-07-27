#include "vaapi_error_category.hpp"

namespace module::output::compressor::vaapi {

    const char* vaapi_error_category_t::name() const noexcept {
        return "vaapi_error_category";
    }

    std::error_condition vaapi_error_category_t::default_error_condition(int code) const noexcept {
        using ve = vaapi_error_code;
        switch (code) {
            case VA_STATUS_SUCCESS: return {ve::SUCCESS};
            case VA_STATUS_ERROR_OPERATION_FAILED: return {ve::OPERATION_FAILED};
            case VA_STATUS_ERROR_ALLOCATION_FAILED: return {ve::ALLOCATION_FAILED};
            case VA_STATUS_ERROR_INVALID_DISPLAY: return {ve::INVALID_DISPLAY};
            case VA_STATUS_ERROR_INVALID_CONFIG: return {ve::INVALID_CONFIG};
            case VA_STATUS_ERROR_INVALID_CONTEXT: return {ve::INVALID_CONTEXT};
            case VA_STATUS_ERROR_INVALID_SURFACE: return {ve::INVALID_SURFACE};
            case VA_STATUS_ERROR_INVALID_BUFFER: return {ve::INVALID_BUFFER};
            case VA_STATUS_ERROR_INVALID_IMAGE: return {ve::INVALID_IMAGE};
            case VA_STATUS_ERROR_INVALID_SUBPICTURE: return {ve::INVALID_SUBPICTURE};
            case VA_STATUS_ERROR_ATTR_NOT_SUPPORTED: return {ve::ATTR_NOT_SUPPORTED};
            case VA_STATUS_ERROR_MAX_NUM_EXCEEDED: return {ve::MAX_NUM_EXCEEDED};
            case VA_STATUS_ERROR_UNSUPPORTED_PROFILE: return {ve::UNSUPPORTED_PROFILE};
            case VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT: return {ve::UNSUPPORTED_ENTRYPOINT};
            case VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT: return {ve::UNSUPPORTED_RT_FORMAT};
            case VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE: return {ve::UNSUPPORTED_BUFFERTYPE};
            case VA_STATUS_ERROR_SURFACE_BUSY: return {ve::SURFACE_BUSY};
            case VA_STATUS_ERROR_FLAG_NOT_SUPPORTED: return {ve::FLAG_NOT_SUPPORTED};
            case VA_STATUS_ERROR_INVALID_PARAMETER: return {ve::INVALID_PARAMETER};
            case VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED: return {ve::RESOLUTION_NOT_SUPPORTED};
            case VA_STATUS_ERROR_UNIMPLEMENTED: return {ve::UNIMPLEMENTED};
            case VA_STATUS_ERROR_SURFACE_IN_DISPLAYING: return {ve::SURFACE_IN_DISPLAYING};
            case VA_STATUS_ERROR_INVALID_IMAGE_FORMAT: return {ve::INVALID_IMAGE_FORMAT};
            case VA_STATUS_ERROR_DECODING_ERROR: return {ve::DECODING_ERROR};
            case VA_STATUS_ERROR_ENCODING_ERROR: return {ve::ENCODING_ERROR};
            case VA_STATUS_ERROR_INVALID_VALUE: return {ve::INVALID_VALUE};
            case VA_STATUS_ERROR_UNSUPPORTED_FILTER: return {ve::UNSUPPORTED_FILTER};
            case VA_STATUS_ERROR_INVALID_FILTER_CHAIN: return {ve::INVALID_FILTER_CHAIN};
            case VA_STATUS_ERROR_HW_BUSY: return {ve::HW_BUSY};
            case VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE: return {ve::UNSUPPORTED_MEMORY_TYPE};
            case VA_STATUS_ERROR_NOT_ENOUGH_BUFFER: return {ve::NOT_ENOUGH_BUFFER};
            default: return {ve::UNKNOWN};
        }
    }

    bool vaapi_error_category_t::equivalent(const std::error_code& code, int condition) const noexcept {
        return *this == code.category() && static_cast<int>(default_error_condition(code.value()).value()) == condition;
    }

    std::string vaapi_error_category_t::message(int code) const noexcept {
        switch (code) {
            case VA_STATUS_SUCCESS: return "Success";
            case VA_STATUS_ERROR_OPERATION_FAILED: return "Operation failed";
            case VA_STATUS_ERROR_ALLOCATION_FAILED: return "Allocation failed";
            case VA_STATUS_ERROR_INVALID_DISPLAY: return "Invalid display";
            case VA_STATUS_ERROR_INVALID_CONFIG: return "Invalid configuration";
            case VA_STATUS_ERROR_INVALID_CONTEXT: return "Invalid context";
            case VA_STATUS_ERROR_INVALID_SURFACE: return "Invalid surface";
            case VA_STATUS_ERROR_INVALID_BUFFER: return "Invalid buffer";
            case VA_STATUS_ERROR_INVALID_IMAGE: return "Invalid image";
            case VA_STATUS_ERROR_INVALID_SUBPICTURE: return "Invalid subpicture";
            case VA_STATUS_ERROR_ATTR_NOT_SUPPORTED: return "Attribute not supported";
            case VA_STATUS_ERROR_MAX_NUM_EXCEEDED: return "Max number exceeded";
            case VA_STATUS_ERROR_UNSUPPORTED_PROFILE: return "Unsupported profile";
            case VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT: return "Unsupported entrypoint";
            case VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT: return "Unsupported render target format";
            case VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE: return "Unsupported buffer type";
            case VA_STATUS_ERROR_SURFACE_BUSY: return "Surface busy";
            case VA_STATUS_ERROR_FLAG_NOT_SUPPORTED: return "Flag not supported";
            case VA_STATUS_ERROR_INVALID_PARAMETER: return "Invalid parameter";
            case VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED: return "Resolution not supported";
            case VA_STATUS_ERROR_UNIMPLEMENTED: return "Unimplemented";
            case VA_STATUS_ERROR_SURFACE_IN_DISPLAYING: return "Surface in displaying";
            case VA_STATUS_ERROR_INVALID_IMAGE_FORMAT: return "Invalid image format";
            case VA_STATUS_ERROR_DECODING_ERROR: return "Decoding error";
            case VA_STATUS_ERROR_ENCODING_ERROR: return "Encoding error";
            case VA_STATUS_ERROR_INVALID_VALUE: return "An invalid/unsupported value was supplied";
            case VA_STATUS_ERROR_UNSUPPORTED_FILTER: return "An unsupported filter was supplied";
            case VA_STATUS_ERROR_INVALID_FILTER_CHAIN: return "An invalid filter chain was supplied";
            case VA_STATUS_ERROR_HW_BUSY: return "Hardware busy";
            case VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE: return "An unsupported memory type was supplied";
            case VA_STATUS_ERROR_NOT_ENOUGH_BUFFER: return "Allocated buffer size is not enough for input or output";
            default: return "Unknown error";
        }
    }

}  // namespace module::output::compressor::vaapi
