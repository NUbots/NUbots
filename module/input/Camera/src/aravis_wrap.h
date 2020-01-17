#ifndef MODULE_INPUT_CAMERA_ARAVIS_WRAP_H
#define MODULE_INPUT_CAMERA_ARAVIS_WRAP_H

extern "C" {
#include <aravis-0.8/arv.h>
}

namespace module {
namespace input {
    namespace arv {

        inline void device_execute_command(ArvDevice* device, const char* feature) {
            GError* error = nullptr;
            arv_device_execute_command(device, feature, &error);
            if (error) {
                std::string message = error->message;
                g_error_free(error);
                throw std::runtime_error(message);
            }
        }

        inline gint64 device_get_integer_feature_value(ArvDevice* device, const char* feature) {
            GError* error = nullptr;
            gint64 v      = arv_device_get_integer_feature_value(device, feature, &error);
            if (error) {
                std::string message = error->message;
                g_error_free(error);
                throw std::runtime_error(message);
            }
            else {
                return v;
            }
        }

        inline void camera_start_acquisition(ArvCamera* camera) {
            GError* error = nullptr;
            arv_camera_start_acquisition(camera, &error);
            if (error) {
                std::string message = error->message;
                g_error_free(error);
                throw std::runtime_error(message);
            }
        }

        inline void camera_stop_acquisition(ArvCamera* camera) {
            GError* error = nullptr;
            arv_camera_stop_acquisition(camera, &error);
            if (error) {
                std::string message = error->message;
                g_error_free(error);
                throw std::runtime_error(message);
            }
        }


        inline void camera_set_region(ArvCamera* camera, gint x, gint y, gint width, gint height) {
            GError* error = nullptr;
            arv_camera_set_region(camera, x, y, width, height, &error);
            if (error) {
                std::string message = error->message;
                g_error_free(error);
                throw std::runtime_error(message);
            }
        }

        inline void camera_gv_set_packet_size(ArvCamera* camera, gint packet_size) {
            GError* error = nullptr;
            arv_camera_gv_set_packet_size(camera, packet_size, &error);
            if (error) {
                std::string message = error->message;
                g_error_free(error);
                throw std::runtime_error(message);
            }
        }


        inline guint camera_get_payload(ArvCamera* camera) {
            GError* error = nullptr;
            guint result  = arv_camera_get_payload(camera, &error);
            if (error) {
                std::string message = error->message;
                g_error_free(error);
                throw std::runtime_error(message);
            }
            return result;
        }

    }  // namespace arv
}  // namespace input
}  // namespace module

#endif  // MODULE_INPUT_CAMERA_ARAVIS_WRAP_H
