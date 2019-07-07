#include "HDRCapture.h"

#include <fmt/format.h>
#include <armadillo>
#include <cmath>
#include <exception>

#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

#include "message/input/Image.h"
#include "message/input/ImageData.h"
#include "message/input/Sensors.h"
#include "message/motion/KinematicsModel.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

#include "extension/Configuration.h"

namespace module {
namespace tools {

    uint HDRCapture::cameraCount = 0;

    using extension::Configuration;
    using message::input::Image;
    using message::input::Sensors;
    using message::motion::KinematicsModel;

    using extension::Configuration;

    using message::input::Image;

    using message::input::ImageData;

    using utility::support::Expression;
    using FOURCC = utility::vision::FOURCC;

    HDRCapture::HDRCapture(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), AravisCameras() {
        // Needed for Aravis cameras.
        arv_g_type_init();

        // Counter for the number of images
        image_count = 0;

        on<Configuration>("HDRCapture.yaml").then([this](const Configuration& config) {
            // Use configuration here from file HDRCapture.yaml

            // Read in the gain and exposure ranges
            exposure_min = config["settings"]["exposure"]["min"].as<int>();
            exposure_max = config["settings"]["exposure"]["max"].as<int>();
            gain_min     = config["settings"]["gain"]["min"].as<int>();
            gain_max     = config["settings"]["gain"]["max"].as<int>();

            exposure_brackets = config["brackets"]["exposure"].as<int>();
            gain_brackets     = config["brackets"]["gain"].as<int>();

            exposure_step = static_cast<float>(exposure_max - exposure_min) / static_cast<float>(exposure_brackets);
            gain_step     = static_cast<float>(gain_max - gain_min) / static_cast<float>(gain_brackets);

            log("Exposure Step...:", exposure_step);
            log("Gain Step.......:", gain_step);

            exposure = exposure_min;
            gain     = gain_min;
        });

        on<Configuration>("Cameras").then("Camera driver loader", [this](const Configuration& config) {
            // Monitor camera config directory for files.
            // Each file MUST define a "driver", we use this driver to load the
            // appropriate handler for the camera.

            if (config.config["driver"] && config.config["deviceID"]) {
                auto driver   = config["driver"].as<std::string>();
                auto deviceID = config["deviceID"].as<std::string>();

                log("Searching for", driver, "camera with deviceID", deviceID);

                auto cam = AravisCameras.find(deviceID);

                if (cam == AravisCameras.end()) {
                    initiateAravisCamera(config);
                    cameraCount++;
                }

                else {
                    resetAravisCamera(cam, config);
                }
            }
        });

        process_handle = on<Every<1, std::chrono::seconds>>().then([this] {
            // TODO update camera configs
            // capture an image
            // emit(Image)
            process_handle.disable();

            if (exposure >= exposure_max) {
                exposure = exposure_min;
                gain += gain_step;
            }
            else {
                exposure += exposure_step;
            }
            log("Current Exposure:", exposure);
            log("Current Gain....:", gain);
            log("Image Count.....:", image_count, "/", (exposure_brackets * gain_brackets));

            if (image_count >= (exposure_brackets * gain_brackets)) {
                powerplant.shutdown();
            }
            else {
                for (auto& camera : AravisCameras) {
                    // trigger mode can be activated before or after start_acquisition
                    arv_camera_set_trigger(camera.second.camera, "Software");

                    arv_camera_start_acquisition(camera.second.camera);

                    g_signal_connect(
                        camera.second.stream, "new-buffer", G_CALLBACK(&HDRCapture::emitAravisImage), &camera.second);

                    arv_stream_set_emit_signals(camera.second.stream, TRUE);

                    // Set exposure.
                    arv_camera_set_exposure_time_auto(camera.second.camera, ARV_AUTO_OFF);
                    arv_camera_set_exposure_time(camera.second.camera, exposure);

                    // Set gain.
                    arv_camera_set_gain_auto(camera.second.camera, ARV_AUTO_OFF);
                    arv_camera_set_gain(camera.second.camera, gain);

                    log("Triggering");
                    arv_camera_software_trigger(camera.second.camera);
                }
                image_count++;
            }
        });

        on<Trigger<ImageData>, Optional<With<Sensors>>, Optional<With<KinematicsModel>>>().then(
            [this](const ImageData& image_data,
                   std::shared_ptr<const Sensors> sensors,
                   std::shared_ptr<const KinematicsModel> model) {
                static uint8_t count = 0;

                // NOTE: we only emit direct the ImageData messages and steal their data
                // Make sure you do not trigger on them anywhere else
                ImageData& i = const_cast<ImageData&>(image_data);

                // Copy across our data
                auto msg               = std::make_unique<Image>();
                msg->format            = i.format;
                msg->dimensions        = i.dimensions;
                msg->data              = std::move(i.data);
                msg->camera_id         = i.camera_id;
                msg->name              = i.serial_number;
                msg->timestamp         = i.timestamp;
                msg->lens.projection   = int(i.lens.projection);
                msg->lens.focal_length = i.lens.focal_length;
                msg->lens.fov          = i.lens.fov;
                msg->lens.centre       = i.lens.centre;

                // Calculate our transform if we have information
                if (sensors && model) {
                    Eigen::Affine3d Htc(sensors->forward_kinematics[utility::input::ServoID::HEAD_PITCH]);
                    Htc(1, 3) += model->head.INTERPUPILLARY_DISTANCE * 0.5f * (i.isLeft ? 1.0f : -1.0f);

                    msg->Hcw = Htc.inverse() * sensors->Htw;
                }
                else {
                    msg->Hcw.setIdentity();
                }

                utility::vision::saveImage(fmt::format("image-{}.ppm", count++), *msg);

                emit(msg);
                log("Enabling");
                process_handle.enable();
            });

        on<Shutdown>().then([this] { shutdownAravisCamera(); });
    }

    void HDRCapture::initiateAravisCamera(const Configuration& config) {
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
                    ArvDevice* newDevice = reinterpret_cast<ArvDevice*>(arv_open_device(arv_get_device_id(device)));

                    if (!ARV_IS_DEVICE(newDevice)) {
                        log<NUClear::WARN>("Failed to open Aravis device with deviceID", deviceID);
                        return;
                    }

                    ArvCamera* newCamera =
                        reinterpret_cast<ArvCamera*>(g_object_new(ARV_TYPE_CAMERA, "device", newDevice, NULL));

                    // Ensure we found the camera.
                    // ArvCamera* newCamera = arv_camera_new(arv_get_device_id(device));
                    if (!ARV_IS_CAMERA(newCamera)) {
                        log<NUClear::WARN>("Failed to create camera with config",
                                           config.fileName,
                                           "and with serial number:",
                                           deviceID);
                        return;
                    }

                    else {
                        /* Create a new stream object */
                        ArvStream* stream = arv_camera_create_stream(newCamera, NULL, NULL);

                        // Add camera to list.
                        CameraContext context = {
                            static_cast<uint32_t>(
                                utility::vision::getFourCCFromDescription(config["format"]["pixel"].as<std::string>())),
                            deviceID,
                            cameraCount,
                            config["is_left"].as<bool>(),
                            Image::Lens(
                                Image::Lens::Projection::EQUIDISTANT,
                                config["lens"]["focal_length"].as<float>(),
                                Eigen::Vector2f(config["lens"]["fov"].as<float>(), config["lens"]["fov"].as<float>()),
                                Eigen::Vector2f(config["lens"]["centre_offset"][0].as<float>(),
                                                config["lens"]["centre_offset"][1].as<float>())),
                            newCamera,
                            stream,
                            *this};

                        camera = AravisCameras.insert(std::make_pair(deviceID, context)).first;

                        log<NUClear::DEBUG>("Found camera", config.fileName, "with serial number", deviceID);
                    }
                }

                resetAravisCamera(camera, config);

                return;
            }
        }

        log<NUClear::WARN>("Failed to find Aravis camera with ID", deviceID);
    }

    void HDRCapture::resetAravisCamera(std::map<std::string, CameraContext>::iterator& camera,
                                       const Configuration& config) {
        // Stop the video stream.
        arv_camera_stop_acquisition(camera->second.camera);

        // Stop emitting signals.
        arv_stream_set_emit_signals(camera->second.stream, FALSE);

        // Add buffers to the queue.
        gint payload = arv_camera_get_payload(camera->second.camera);
        for (size_t i = 0; i < config["buffer_count"].as<size_t>(); i++) {
            arv_stream_push_buffer(camera->second.stream, arv_buffer_new(payload, NULL));
        }

        // Set the pixel format.
        arv_camera_set_pixel_format(camera->second.camera,
                                    utility::vision::getAravisPixelFormat(config["format"]["pixel"].as<std::string>()));

        // Set the width and height of the image.
        arv_camera_set_region(camera->second.camera,
                              0,
                              0,
                              config["format"]["width"].as<size_t>(),
                              config["format"]["height"].as<size_t>());
    }

    void HDRCapture::emitAravisImage(ArvStream* stream, CameraContext* context) {
        ArvBuffer* buffer;
        buffer = arv_stream_try_pop_buffer(stream);

        if ((buffer != NULL) && (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS)) {
            int width, height;
            size_t buffSize;
            arv_buffer_get_image_region(buffer, NULL, NULL, &width, &height);
            const uint8_t* buff = reinterpret_cast<const uint8_t*>(arv_buffer_get_data(buffer, &buffSize));

            auto msg = std::make_unique<ImageData>();
            msg->data.insert(msg->data.end(), buff, buff + buffSize);

            msg->dimensions << (unsigned int) (width), (unsigned int) (height);
            msg->timestamp = NUClear::clock::time_point(std::chrono::nanoseconds(arv_buffer_get_timestamp(buffer)));

            msg->format            = context->fourcc;
            msg->serial_number     = context->deviceID;
            msg->camera_id         = context->cameraID;
            msg->isLeft            = context->isLeft;
            msg->lens.projection   = int(context->lens.projection);
            msg->lens.focal_length = context->lens.focal_length;
            msg->lens.fov          = context->lens.fov;
            msg->lens.centre       = context->lens.centre;

            context->reactor.emit<Scope::DIRECT>(msg);
            context->reactor.log("Emitting");
            arv_stream_push_buffer(stream, buffer);
        }
    }

    void HDRCapture::shutdownAravisCamera() {
        for (auto& camera : AravisCameras) {
            // Stop the video stream.
            arv_camera_stop_acquisition(camera.second.camera);

            // Stop emitting signals.
            arv_stream_set_emit_signals(camera.second.stream, FALSE);

            // Unreference our stream and our camera.
            g_object_unref(camera.second.stream);
            g_object_unref(camera.second.camera);
        }

        arv_debug_shutdown();
        arv_shutdown();
    }
}  // namespace tools
}  // namespace module
