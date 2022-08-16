#ifndef MODULE_INPUT_CAMERA_ARAVIS_WRAP_HPP
#define MODULE_INPUT_CAMERA_ARAVIS_WRAP_HPP

#include <aravis-0.8/arv.h>
#include <fmt/format.h>
#include <stdexcept>
#include <string>

namespace module::input::arv {

    template <typename ArvFunction, typename... Arguments>
    inline void wrap(ArvFunction&& arv_function, Arguments&&... args) {
        GError* error = nullptr;
        arv_function(std::forward<Arguments>(args)..., &error);
        if (error != nullptr) {
            std::string message = error->message;
            g_error_free(error);
            throw std::runtime_error(message);
        }
    }

    template <typename ArvFunction, typename... Arguments>
    inline auto wrap_return(ArvFunction&& arv_function, Arguments&&... args) {
        GError* error = nullptr;
        auto value    = arv_function(std::forward<Arguments>(args)..., &error);
        if (error != nullptr) {
            std::string message = error->message;
            g_error_free(error);
            throw std::runtime_error(message);
        }
        return value;
    }

    inline ArvCamera* camera_new(const std::string& device_description) {
        return wrap_return(arv_camera_new, device_description.c_str());
    }

    inline ArvStream* camera_create_stream(ArvCamera* camera, ArvStreamCallback callback, void* user_data) {
        if (ARV_IS_CAMERA(camera) != TRUE) {
            throw(std::runtime_error("Invalid Aravis camera: camera_stop_acquisition"));
        }
        return wrap_return(arv_camera_create_stream, camera, callback, user_data);
    }

    inline void device_execute_command(ArvDevice* device, const char* feature) {
        if (ARV_IS_DEVICE(device) != TRUE) {
            throw(std::runtime_error("Invalid Aravis device: device_execute_command"));
        }
        wrap(arv_device_execute_command, device, feature);
    }

    inline gint64 device_get_integer_feature_value(ArvDevice* device, const char* feature) {
        if (ARV_IS_DEVICE(device) != TRUE) {
            throw(std::runtime_error(fmt::format("Attempt to get feature '{}' from an invalid device", feature)));
        }
        return wrap_return(arv_device_get_integer_feature_value, device, feature);
    }

    inline bool device_get_boolean_feature_value(ArvDevice* device, const char* feature) {
        if (ARV_IS_DEVICE(device) != TRUE) {
            throw(std::runtime_error(fmt::format("Attempt to get feature '{}' from an invalid device", feature)));
        }
        return wrap_return(arv_device_get_boolean_feature_value, device, feature) == TRUE;
    }

    inline void camera_start_acquisition(ArvCamera* camera) {
        if (ARV_IS_CAMERA(camera) != TRUE) {
            throw(std::runtime_error("Invalid Aravis camera: camera_start_acquisition"));
        }
        wrap(arv_camera_start_acquisition, camera);
    }

    inline void camera_stop_acquisition(ArvCamera* camera) {
        if (ARV_IS_CAMERA(camera) != TRUE) {
            throw(std::runtime_error("Invalid Aravis camera: camera_stop_acquisition"));
        }
        wrap(arv_camera_stop_acquisition, camera);
    }

    inline void camera_set_region(ArvCamera* camera, gint x, gint y, gint width, gint height) {
        if (ARV_IS_CAMERA(camera) != TRUE) {
            throw(std::runtime_error("Invalid Aravis camera: camera_set_region"));
        }
        wrap(arv_camera_set_region, camera, x, y, width, height);
    }

    inline void camera_gv_set_packet_size(ArvCamera* camera, gint packet_size) {
        if (ARV_IS_CAMERA(camera) != TRUE) {
            throw(std::runtime_error("Invalid Aravis camera: camera_gv_set_packet_size"));
        }
        wrap(arv_camera_gv_set_packet_size, camera, packet_size);
    }

    inline guint camera_get_payload(ArvCamera* camera) {
        if (ARV_IS_CAMERA(camera) != TRUE) {
            throw(std::runtime_error("Attempt to get payload from a non-valid Aravis camera"));
        }
        return wrap_return(arv_camera_get_payload, camera);
    }

    inline void stream_set_emit_signals(ArvStream* stream, bool emit_signals) {
        if (ARV_IS_STREAM(stream) != TRUE) {
            throw(std::runtime_error("Invalid Aravis stream: stream_set_emit_signals"));
        }
        arv_stream_set_emit_signals(stream, emit_signals ? TRUE : FALSE);
    }

}  // namespace module::input::arv

#endif  // MODULE_INPUT_CAMERA_ARAVIS_WRAP_HPP
