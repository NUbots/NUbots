#include "Camera.h"

#include <fmt/format.h>
#include <armadillo>
#include <cmath>
#include <exception>

#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"
#include "utility/vision/Spinnaker.h"

namespace module {
namespace input {
    using message::input::CameraParameters;
    using extension::Configuration;
    using utility::support::Expression;
    using FOURCC = utility::vision::FOURCC;

    void Camera::initiateAravisCamera(const Configuration& config) {
        arv_update_device_list();
        unsigned int devices = arv_get_n_devices();

        log<NUClear::DEBUG>("Found ", devices, " cameras.");

        if (devices < 1) {
            return;
        }

        std::string deviceID = config["deviceID"].as<std::string>();

        for (unsigned int device = 0; device < devices; device++) {
            if (deviceID.compare(arv_get_device_serial_nbr(device)) == 0) {
                log<NUClear::DEBUG>("Processing camera", config.fileName, "with serial number", deviceID);

                // See if we already have this camera
                auto camera = AravisCameras.find(deviceID);

                if (camera == AravisCameras.end()) {
                    std::unique_ptr<ArvCamera> newCamera(arv_camera_new(arv_get_device_id(device)));

                    // Ensure we found the camera.
                    if (newCamera) {
                        /* Create a new stream object */
                        std::unique_ptr<ArvStream> stream(arv_camera_create_stream(newCamera.get(), NULL, NULL));

                        // Add camera to list.
                        camera =
                            AravisCameras
                                .insert(std::make_pair(
                                    deviceID, std::make_tuple(cameraCount, std::move(newCamera), std::move(stream))))
                                .first;
                    }

                    else {
                        log<NUClear::WARN>("Failed to find camera", config.fileName, " with serial number:", deviceID);
                        return;
                    }
                }

                resetAravisCamera(camera, config);

                auto cameraParameters = std::make_unique<CameraParameters>();

                // Generic camera parameters
                cameraParameters->imageSizePixels << config["format"]["width"].as<uint>(),
                    config["format"]["height"].as<uint>();
                cameraParameters->FOV << config["lens"]["FOV"].as<double>(), config["lens"]["FOV"].as<double>();

                // Radial specific
                cameraParameters->lens                   = CameraParameters::LensType::RADIAL;
                cameraParameters->radial.radiansPerPixel = config["lens"]["radiansPerPixel"].as<float>();
                cameraParameters->centreOffset = convert<int, 2>(config["lens"]["centreOffset"].as<arma::ivec>());

                emit<Scope::DIRECT>(std::move(cameraParameters));

                log("Emitted radial camera parameters for camera", config["deviceID"].as<std::string>());
                return;
            }
        }

        log<NUClear::WARN>("Failed to find Aravis camera with ID", deviceID);
    }

    void Camera::resetAravisCamera(
        std::map<std::string, std::tuple<uint, std::unique_ptr<ArvCamera>, std::unique_ptr<ArvStream>>>::iterator&
            camera,
        const Configuration& config) {

        // Stop the video stream.
        arv_camera_stop_acquisition(std::get<1>(camera->second).get());

        // Stop emitting signals.
        arv_stream_set_emit_signals(std::get<2>(camera->second).get(), FALSE);

        // Add buffers to the queue.
        gint payload = arv_camera_get_payload(std::get<1>(camera->second).get());
        for (size_t i = 0; i < config["buffer_count"].as<size_t>(); i++) {
            arv_stream_push_buffer(std::get<2>(camera->second).get(), arv_buffer_new(payload, NULL));
        }

        // Set the pixel format.
        arv_camera_set_pixel_format(std::get<1>(camera->second).get(),
                                    utility::vision::getAravisPixelFormat(config["format"]["pixel"].as<std::string>()));

        // Set the width and height of the image.
        arv_camera_set_region(std::get<1>(camera->second).get(),
                              0,
                              0,
                              config["format"]["width"].as<size_t>(),
                              config["format"]["height"].as<size_t>());

        // Set exposure.
        auto exposure = config["settings"]["exposure"].as<Expression>();
        if (std::isfinite(exposure)) {
            arv_camera_set_exposure_time_auto(std::get<1>(camera->second).get(), ARV_AUTO_OFF);
            arv_camera_set_exposure_time(std::get<1>(camera->second).get(), exposure);
        }
        else {
            arv_camera_set_exposure_time_auto(std::get<1>(camera->second).get(), ARV_AUTO_CONTINUOUS);
        }

        // Set gain.
        auto gain = config["settings"]["gain"].as<Expression>();
        if (std::isfinite(gain)) {
            arv_camera_set_gain_auto(std::get<1>(camera->second).get(), ARV_AUTO_OFF);
            arv_camera_set_gain(std::get<1>(camera->second).get(), gain);
        }
        else {
            arv_camera_set_gain_auto(std::get<1>(camera->second).get(), ARV_AUTO_CONTINUOUS);
        }

        ImageContext context = {static_cast<uint32_t>(utility::vision::getFourCCFromDescription(
                                    config["format"]["pixel"].as<std::string>())),
                                camera->first,
                                std::get<0>(camera->second),
                                config["is_left"].as<bool>()};

        arv_camera_start_acquisition(std::get<1>(camera->second).get());
        g_signal_connect(
            std::get<2>(camera->second).get(), "new-buffer", G_CALLBACK(&Camera::EmitAravisImage), &context);
        arv_stream_set_emit_signals(std::get<2>(camera->second).get(), TRUE);
    }

    void Camera::EmitAravisImage(ArvStream* stream, ImageContext* context) {
        ArvBuffer* buffer;
        buffer = arv_stream_pop_buffer(stream);
        if ((buffer != NULL) && (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS)) {
            int width, height;
            size_t buffSize;
            arv_buffer_get_image_region(buffer, NULL, NULL, &width, &height);
            const uint8_t* buff = reinterpret_cast<const uint8_t*>(arv_buffer_get_data(buffer, &buffSize));

            auto msg = std::make_unique<ImageData>();
            msg->data.insert(msg->data.end(), buff, buff + buffSize);

            msg->dimensions << (unsigned int) (width), (unsigned int) (height);
            msg->timestamp = NUClear::clock::time_point(std::chrono::nanoseconds(arv_buffer_get_timestamp(buffer)));

            msg->format        = context->fourcc;
            msg->serial_number = context->deviceID;
            msg->camera_id     = context->cameraID;
            msg->isLeft        = context->isLeft;

            emit<Scope::DIRECT>(msg);
            arv_stream_push_buffer(stream, buffer);
        }
    }

    void Camera::ShutdownAravisCamera() {
        for (auto& camera : AravisCameras) {
            // Stop the video stream.
            arv_camera_stop_acquisition(std::get<1>(camera.second).get());

            // Stop emitting signals.
            arv_stream_set_emit_signals(std::get<2>(camera.second).get(), FALSE);

            // Unreference our stream and our camera.
            g_object_unref(std::get<2>(camera.second).get());
            g_object_unref(std::get<1>(camera.second).get());
        }
    }

}  // namespace input
}  // namespace module
