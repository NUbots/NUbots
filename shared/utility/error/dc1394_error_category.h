/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_ERROR_DC1394_ERROR_CATEGORY_H
#define UTILITY_ERROR_DC1394_ERROR_CATEGORY_H
#include <system_error>
#include <string>

#include <dc1394/dc1394.h>

namespace utility {
namespace error {
    enum class dc1394_errorc {
        SUCCESS,
        FAILURE,
        NO_FRAME,
        NO_CAMERA,
        NOT_A_CAMERA,
        FUNCTION_NOT_SUPPORTED,
        CAMERA_NOT_INITITIALIZED,
        INVALID_FEATURE,
        INVALID_FORMAT,
        INVALID_MODE,
        INVALID_FRAMERATE,
        INVALID_TRIGGER_MODE,
        INVALID_TRIGGER_SOURCE,
        INVALID_ISO_SPEED,
        INVALID_IIDC_VERSION,
        INVALID_COLOR_MODE,
        INVALID_FORMAT7_COLOR_TILE,
        REQ_VALUE_OUTSIDE_RANGE,
        INVALID_ERROR_CODE,
        MEMORY_ALLOCATION_FAILURE,
        TAGGED_REGISTER_NOT_FOUND,
        FORMAT7_ERROR_FLAG_1,
        FORMAT7_ERROR_FLAG_2,
        INVALID_BAYER_METHOD,
        HANDLE_CREATION_FAILURE,
        GENERIC_INVALID_ARGUMENT,
        INVALID_VIDEO1394_DEVICE,
        NO_ISO_CHANNEL,
        NO_BANDWIDTH,
        IOCTL_FAILURE,
        CAPTURE_IS_NOT_SET,
        CAPTURE_IS_RUNNING,
        RAW1394_FAILURE,
        BASLER_NO_MORE_SFF_CHUNKS,
        BASLER_CORRUPTED_SFF_CHUNK,
        BASLER_UNKNOWN_SFF_CHUNK,
        UNKNOWN
    };
}
}

namespace std {
    template<> struct is_error_condition_enum<utility::error::dc1394_errorc> : public true_type {};
}

namespace utility {
namespace error {

    class dc1394_error_category_t : public std::error_category {
        public:

            virtual const char* name() const noexcept;

            virtual std::error_condition default_error_condition(int code) const noexcept;

            virtual bool equivalent(const std::error_code& code, int condition) const noexcept;

            virtual std::string message(int code) const noexcept;
    };

    const std::error_category& dc1394_error_category();

    std::error_condition make_error_condition(utility::error::dc1394_errorc e);
}
}

namespace utility {
namespace error {
    const std::error_category& dc1394_error_category() {
        static dc1394_error_category_t instance;
        return instance;
    }

    std::error_condition make_error_condition(utility::error::dc1394_errorc e) {
        return std::error_condition(static_cast<int>(e), utility::error::dc1394_error_category());
    }

    const char* dc1394_error_category_t::name() const noexcept {
        return "dc1394_error_category";
    }

    std::error_condition dc1394_error_category_t::default_error_condition(int code) const noexcept {

        switch(code) {
            case DC1394_SUCCESS: return std::error_condition(dc1394_errorc::SUCCESS);
            case DC1394_FAILURE: return std::error_condition(dc1394_errorc::FAILURE);
            // case DC1394_NO_FRAME: return std::error_condition(dc1394_errorc::NO_FRAME);
            // case DC1394_NO_CAMERA: return std::error_condition(dc1394_errorc::NO_CAMERA);
            case DC1394_NOT_A_CAMERA: return std::error_condition(dc1394_errorc::NOT_A_CAMERA);
            case DC1394_FUNCTION_NOT_SUPPORTED: return std::error_condition(dc1394_errorc::FUNCTION_NOT_SUPPORTED);
            // case DC1394_CAMERA_NOT_INITITIALIZED: return std::error_condition(dc1394_errorc::CAMERA_NOT_INITITIALIZED);
            case DC1394_INVALID_FEATURE: return std::error_condition(dc1394_errorc::INVALID_FEATURE);
            // case DC1394_INVALID_FORMAT: return std::error_condition(dc1394_errorc::INVALID_FORMAT);
            // case DC1394_INVALID_MODE: return std::error_condition(dc1394_errorc::INVALID_MODE);
            case DC1394_INVALID_FRAMERATE: return std::error_condition(dc1394_errorc::INVALID_FRAMERATE);
            case DC1394_INVALID_TRIGGER_MODE: return std::error_condition(dc1394_errorc::INVALID_TRIGGER_MODE);
            case DC1394_INVALID_TRIGGER_SOURCE: return std::error_condition(dc1394_errorc::INVALID_TRIGGER_SOURCE);
            case DC1394_INVALID_ISO_SPEED: return std::error_condition(dc1394_errorc::INVALID_ISO_SPEED);
            case DC1394_INVALID_IIDC_VERSION: return std::error_condition(dc1394_errorc::INVALID_IIDC_VERSION);
            // case DC1394_INVALID_COLOR_MODE: return std::error_condition(dc1394_errorc::INVALID_COLOR_MODE);
            // case DC1394_INVALID_FORMAT7_COLOR_TILE: return std::error_condition(dc1394_errorc::INVALID_FORMAT7_COLOR_TILE);
            case DC1394_REQ_VALUE_OUTSIDE_RANGE: return std::error_condition(dc1394_errorc::REQ_VALUE_OUTSIDE_RANGE);
            case DC1394_INVALID_ERROR_CODE: return std::error_condition(dc1394_errorc::INVALID_ERROR_CODE);
            case DC1394_MEMORY_ALLOCATION_FAILURE: return std::error_condition(dc1394_errorc::MEMORY_ALLOCATION_FAILURE);
            case DC1394_TAGGED_REGISTER_NOT_FOUND: return std::error_condition(dc1394_errorc::TAGGED_REGISTER_NOT_FOUND);
            case DC1394_FORMAT7_ERROR_FLAG_1: return std::error_condition(dc1394_errorc::FORMAT7_ERROR_FLAG_1);
            case DC1394_FORMAT7_ERROR_FLAG_2: return std::error_condition(dc1394_errorc::FORMAT7_ERROR_FLAG_2);
            case DC1394_INVALID_BAYER_METHOD: return std::error_condition(dc1394_errorc::INVALID_BAYER_METHOD);
            // case DC1394_HANDLE_CREATION_FAILURE: return std::error_condition(dc1394_errorc::HANDLE_CREATION_FAILURE);
            // case DC1394_GENERIC_INVALID_ARGUMENT: return std::error_condition(dc1394_errorc::GENERIC_INVALID_ARGUMENT);
            case DC1394_INVALID_VIDEO1394_DEVICE: return std::error_condition(dc1394_errorc::INVALID_VIDEO1394_DEVICE);
            case DC1394_NO_ISO_CHANNEL: return std::error_condition(dc1394_errorc::NO_ISO_CHANNEL);
            case DC1394_NO_BANDWIDTH: return std::error_condition(dc1394_errorc::NO_BANDWIDTH);
            case DC1394_IOCTL_FAILURE: return std::error_condition(dc1394_errorc::IOCTL_FAILURE);
            case DC1394_CAPTURE_IS_NOT_SET: return std::error_condition(dc1394_errorc::CAPTURE_IS_NOT_SET);
            case DC1394_CAPTURE_IS_RUNNING: return std::error_condition(dc1394_errorc::CAPTURE_IS_RUNNING);
            case DC1394_RAW1394_FAILURE: return std::error_condition(dc1394_errorc::RAW1394_FAILURE);
            case DC1394_BASLER_NO_MORE_SFF_CHUNKS: return std::error_condition(dc1394_errorc::BASLER_NO_MORE_SFF_CHUNKS);
            case DC1394_BASLER_CORRUPTED_SFF_CHUNK: return std::error_condition(dc1394_errorc::BASLER_CORRUPTED_SFF_CHUNK);
            case DC1394_BASLER_UNKNOWN_SFF_CHUNK: return std::error_condition(dc1394_errorc::BASLER_UNKNOWN_SFF_CHUNK);
            default: return std::error_condition(dc1394_errorc::UNKNOWN);
        }
    }

    bool dc1394_error_category_t::equivalent(const std::error_code& code, int condition) const noexcept {
        return *this == code.category() &&
            static_cast<int>(default_error_condition(code.value()).value()) == condition;
    }

    std::string dc1394_error_category_t::message(int code) const noexcept {
        switch(code) {
            case DC1394_SUCCESS: return "Success";
            case DC1394_FAILURE: return "Failure";
            // case DC1394_NO_FRAME: return "No Frame";
            // case DC1394_NO_CAMERA: return "No Camera";
            case DC1394_NOT_A_CAMERA: return "Not a Camera";
            case DC1394_FUNCTION_NOT_SUPPORTED: return "Function not supported";
            // case DC1394_CAMERA_NOT_INITITIALIZED: return "Camera not initialized";
            case DC1394_INVALID_FEATURE: return "Invalid Feature";
            // case DC1394_INVALID_FORMAT: return "Invalid Format";
            // case DC1394_INVALID_MODE: return "Invalid Mode";
            case DC1394_INVALID_FRAMERATE: return "Invalid Framerate";
            case DC1394_INVALID_TRIGGER_MODE: return "Invalid Trigger Mode";
            case DC1394_INVALID_TRIGGER_SOURCE: return "Invalid Source";
            case DC1394_INVALID_ISO_SPEED: return "Invalid ISO Speed";
            case DC1394_INVALID_IIDC_VERSION: return "Invalid IIDC Version";
            // case DC1394_INVALID_COLOR_MODE: return "Invalid Colour Mode";
            // case DC1394_INVALID_FORMAT7_COLOR_TILE: return "Invalid Format7 Colour Tile";
            case DC1394_REQ_VALUE_OUTSIDE_RANGE: return "Req** value outside range";
            case DC1394_INVALID_ERROR_CODE: return "Invalid Error Code";
            case DC1394_MEMORY_ALLOCATION_FAILURE: return "Memory Allocation Failure";
            case DC1394_TAGGED_REGISTER_NOT_FOUND: return "Tagged Register not found";
            case DC1394_FORMAT7_ERROR_FLAG_1: return "Format7 Error Flag 1";
            case DC1394_FORMAT7_ERROR_FLAG_2: return "Format7 Error Flag 2";
            case DC1394_INVALID_BAYER_METHOD: return "Invalid Bayer Method";
            // case DC1394_HANDLE_CREATION_FAILURE: return "Handle Creation Failure";
            // case DC1394_GENERIC_INVALID_ARGUMENT: return "Invalid Argument";
            case DC1394_INVALID_VIDEO1394_DEVICE: return "Invalid Video1394 Device";
            case DC1394_NO_ISO_CHANNEL: return "No ISO Channel";
            case DC1394_NO_BANDWIDTH: return "No Bandwidth";
            case DC1394_IOCTL_FAILURE: return "ioctl Failure";
            case DC1394_CAPTURE_IS_NOT_SET: return "Capture is not set";
            case DC1394_CAPTURE_IS_RUNNING: return "Capture is running";
            case DC1394_RAW1394_FAILURE: return "raw1394 Failure";
            case DC1394_BASLER_NO_MORE_SFF_CHUNKS: return "Basler no more SFF chunks";
            case DC1394_BASLER_CORRUPTED_SFF_CHUNK: return "Basler corrupted SFF chunk";
            case DC1394_BASLER_UNKNOWN_SFF_CHUNK: return "Basler Unknown SFF chunk";
            default: return "Unknown Error";
        }
    }
}
}
#endif
