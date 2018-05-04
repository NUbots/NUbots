#include "Camera.h"

#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/motion/KinematicsModel.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace input {

    uint Camera::cameraCount = 0;

    using extension::Configuration;
    using message::input::CameraParameters;
    using message::input::Image;
    using message::input::Sensors;
    using message::motion::KinematicsModel;

    Camera::Camera(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , dumpImages(false)
        , V4L2FrameRateHandle()
        , V4L2SettingsHandle()
        , V4L2Cameras()
        , AravisCameras() {

        // Needed for Aravis cameras.
        arv_g_type_init();

        on<Configuration>("Camera.yaml").then("Camera Module Configuration", [this](const Configuration& config) {
            dumpImages = config["dump_images"].as<bool>();
        });

        on<Configuration>("Cameras").then("Camera driver loader", [this](const Configuration& config) {
            // Monitor camera config directory for files.
            // Each file MUST define a "driver", we use this driver to load the appropriate handler for the camera.

            if (config.config["driver"] && config.config["deviceID"]) {
                auto driver   = config["driver"].as<std::string>();
                auto deviceID = config["deviceID"].as<std::string>();

                log("Searching for", driver, "camera with deviceID", deviceID);

                if (driver == "V4L2") {
                    auto cam = V4L2Cameras.find(deviceID);

                    if (cam == V4L2Cameras.end()) {
                        V4L2Cameras.insert(std::make_pair(deviceID, std::move(initiateV4L2Camera(config))));
                        cameraCount++;
                    }

                    else {
                        cam->second.setConfig(config);
                    }
                }

                else if (driver == "Aravis") {
                    auto cam = AravisCameras.find(deviceID);

                    if (cam == AravisCameras.end()) {
                        initiateAravisCamera(config);
                        cameraCount++;
                    }

                    else {
                        resetAravisCamera(cam, config);
                    }
                }

                else {
                    log<NUClear::FATAL>("Unsupported camera driver:", driver);
                }
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
                auto msg           = std::make_unique<Image>();
                msg->format        = i.format;
                msg->dimensions    = i.dimensions;
                msg->data          = std::move(i.data);
                msg->camera_id     = i.camera_id;
                msg->serial_number = i.serial_number;
                msg->timestamp     = i.timestamp;

                // Calculate our transform if we have information
                if (sensors && model) {

                    Eigen::Affine3d Htc(sensors->forwardKinematics[utility::input::ServoID::HEAD_PITCH]);
                    Htc(1, 3) += model->head.INTERPUPILLARY_DISTANCE * 0.5f * (i.isLeft ? 1.0f : -1.0f);

                    msg->Hcw = Htc.inverse() * sensors->world;
                }
                else {
                    msg->Hcw.setIdentity();
                }

                if (dumpImages) {
                    utility::vision::saveImage(fmt::format("image-{}.ppm", count++), *msg);
                }

                emit(msg);
            });

        on<Shutdown>().then([this] {
            ShutdownV4L2Camera();
            ShutdownAravisCamera();
        });
    }

    // When we shutdown, we must tell our camera class to close (stop streaming)
    void Camera::ShutdownV4L2Camera() {
        for (auto& camera : V4L2Cameras) {
            camera.second.closeCamera();
            camera.second.disableHandles();
        }

        V4L2Cameras.clear();
    }
}  // namespace input
}  // namespace module
