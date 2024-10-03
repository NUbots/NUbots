/*
 * MIT License
 *
 * Copyright (c) 2020 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_ERROR_CATEGORY_HPP
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_ERROR_CATEGORY_HPP

#include <system_error>
#include <va/va.h>

namespace module::output::compressor::vaapi {

    enum class vaapi_error_code {
        SUCCESS                  = VA_STATUS_SUCCESS,
        OPERATION_FAILED         = VA_STATUS_ERROR_OPERATION_FAILED,
        ALLOCATION_FAILED        = VA_STATUS_ERROR_ALLOCATION_FAILED,
        INVALID_DISPLAY          = VA_STATUS_ERROR_INVALID_DISPLAY,
        INVALID_CONFIG           = VA_STATUS_ERROR_INVALID_CONFIG,
        INVALID_CONTEXT          = VA_STATUS_ERROR_INVALID_CONTEXT,
        INVALID_SURFACE          = VA_STATUS_ERROR_INVALID_SURFACE,
        INVALID_BUFFER           = VA_STATUS_ERROR_INVALID_BUFFER,
        INVALID_IMAGE            = VA_STATUS_ERROR_INVALID_IMAGE,
        INVALID_SUBPICTURE       = VA_STATUS_ERROR_INVALID_SUBPICTURE,
        ATTR_NOT_SUPPORTED       = VA_STATUS_ERROR_ATTR_NOT_SUPPORTED,
        MAX_NUM_EXCEEDED         = VA_STATUS_ERROR_MAX_NUM_EXCEEDED,
        UNSUPPORTED_PROFILE      = VA_STATUS_ERROR_UNSUPPORTED_PROFILE,
        UNSUPPORTED_ENTRYPOINT   = VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT,
        UNSUPPORTED_RT_FORMAT    = VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT,
        UNSUPPORTED_BUFFERTYPE   = VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE,
        SURFACE_BUSY             = VA_STATUS_ERROR_SURFACE_BUSY,
        FLAG_NOT_SUPPORTED       = VA_STATUS_ERROR_FLAG_NOT_SUPPORTED,
        INVALID_PARAMETER        = VA_STATUS_ERROR_INVALID_PARAMETER,
        RESOLUTION_NOT_SUPPORTED = VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED,
        UNIMPLEMENTED            = VA_STATUS_ERROR_UNIMPLEMENTED,
        SURFACE_IN_DISPLAYING    = VA_STATUS_ERROR_SURFACE_IN_DISPLAYING,
        INVALID_IMAGE_FORMAT     = VA_STATUS_ERROR_INVALID_IMAGE_FORMAT,
        DECODING_ERROR           = VA_STATUS_ERROR_DECODING_ERROR,
        ENCODING_ERROR           = VA_STATUS_ERROR_ENCODING_ERROR,
        INVALID_VALUE            = VA_STATUS_ERROR_INVALID_VALUE,
        UNSUPPORTED_FILTER       = VA_STATUS_ERROR_UNSUPPORTED_FILTER,
        INVALID_FILTER_CHAIN     = VA_STATUS_ERROR_INVALID_FILTER_CHAIN,
        HW_BUSY                  = VA_STATUS_ERROR_HW_BUSY,
        UNSUPPORTED_MEMORY_TYPE  = VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE,
        NOT_ENOUGH_BUFFER        = VA_STATUS_ERROR_NOT_ENOUGH_BUFFER,
        UNKNOWN
    };
}  // namespace module::output::compressor::vaapi

namespace std {
    template <>
    struct is_error_condition_enum<module::output::compressor::vaapi::vaapi_error_code> : public true_type {};

}  // namespace std

namespace module::output::compressor::vaapi {
    class vaapi_error_category_t : public std::error_category {
    public:
        [[nodiscard]] const char* name() const noexcept override;

        [[nodiscard]] std::error_condition default_error_condition(int code) const noexcept override;

        [[nodiscard]] bool equivalent(const std::error_code& code, int condition) const noexcept override;

        [[nodiscard]] std::string message(int code) const noexcept override;
    };

    inline const std::error_category& vaapi_error_category() {
        static vaapi_error_category_t instance;
        return instance;
    }

    inline std::error_condition make_error_condition(vaapi_error_code e) {
        return {static_cast<int>(e), vaapi_error_category()};
    }

}  // namespace module::output::compressor::vaapi

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_ERROR_CATEGORY_HPP
