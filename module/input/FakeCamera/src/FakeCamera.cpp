#include <fmt/format.h>

#include "FakeCamera.h"

#include "extension/Configuration.h"

#include "message/input/CameraParameters.h"
#include "message/input/Image.h"

namespace module {
namespace input {

    using extension::Configuration;

    FakeCamera::FakeCamera(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , image_folder()
        , image_prefix()
        , image_ext()
        , num_images(0)
        , count(0)
        , image_format(utility::vision::FOURCC::UNKNOWN) {

        on<Configuration>("FakeCamera.yaml").then([this](const Configuration& config) {
            image_folder = config["image_folder"].as<std::string>();
            image_prefix = config["image_prefix"].as<std::string>();
            image_ext    = config["image_ext"].as<std::string>();
            num_images   = config["num_images"].as<size_t>();
            image_format = utility::vision::getFourCCFromDescription(config["image_format"].as<std::string>());
        });

        on<Startup>().then([this] {
            auto cameraParameters = std::make_unique<message::input::CameraParameters>();

            // Generic camera parameters
            cameraParameters->imageSizePixels << 1280, 1024;
            cameraParameters->FOV << 3.14, 3.14;

            // Radial specific
            cameraParameters->lens                   = message::input::CameraParameters::LensType::RADIAL;
            cameraParameters->radial.radiansPerPixel = 0.0026997136600899543;
            cameraParameters->centreOffset << 0, 0;

            emit<Scope::DIRECT>(std::move(cameraParameters));
        });

        on<Every<30, Per<std::chrono::seconds>>, Single>().then([this] {
            auto msg           = std::make_unique<message::input::Image>();
            msg->format        = image_format;
            msg->camera_id     = count % 2;
            msg->serial_number = "FakeCamera";
            msg->timestamp     = NUClear::clock::now();
            utility::vision::loadImage(fmt::format("{}/{}-{}.{}", image_folder, image_prefix, count, image_ext), *msg);

            emit(msg);

            if ((++count) == num_images) {
                count = 0;
            }
        });
    }
}  // namespace input
}  // namespace module
