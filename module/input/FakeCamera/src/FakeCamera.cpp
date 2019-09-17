#include <fmt/format.h>
#include <Eigen/Core>

#include "FakeCamera.h"

#include "extension/Configuration.h"

namespace module {
namespace input {

    using extension::Configuration;

    using message::input::Image;

    FakeCamera::FakeCamera(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , image_folder()
        , image_prefix()
        , image_ext()
        , num_images(0)
        , count(0)
        , image_format(utility::vision::FOURCC::UNKNOWN) {

        on<Configuration>("FakeCamera.yaml").then([this](const Configuration& config) {
            image_folder = config["image"]["folder"].as<std::string>();
            image_prefix = config["image"]["prefix"].as<std::string>();
            image_ext    = config["image"]["ext"].as<std::string>();
            num_images   = config["image"]["count"].as<size_t>();
            image_format = utility::vision::getFourCCFromDescription(config["image"]["format"].as<std::string>());

            lens = Image::Lens(Image::Lens::Projection(config["lens"]["projection"].as<std::string>()),
                               config["lens"]["focal_length"].as<float>(),
                               Eigen::Vector2f(config["lens"]["fov"].as<float>(), config["lens"]["fov"].as<float>()),
                               Eigen::Vector2f(config["lens"]["centre_offset"][0].as<float>(),
                                               config["lens"]["centre_offset"][1].as<float>()));
        });

        on<Every<30, Per<std::chrono::seconds>>, Single>().then([this] {
            auto msg       = std::make_unique<Image>();
            msg->format    = image_format;
            msg->camera_id = 0;
            msg->name      = "FakeCamera";
            msg->timestamp = NUClear::clock::now();
            msg->lens      = lens;
            utility::vision::loadImage(fmt::format("{}/{}-{}.{}", image_folder, image_prefix, count, image_ext), *msg);

            emit(msg);

            if ((++count) == num_images) {
                count = 0;
            }
        });
    }
}  // namespace input
}  // namespace module
