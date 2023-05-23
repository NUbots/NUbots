#include "FakeCamera.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <filesystem>
#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include "extension/Configuration.hpp"

#include "message/output/CompressedImage.hpp"

#include "utility/strutil/strutil.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/fourcc.hpp"

namespace module::input {

    using extension::Configuration;

    using message::output::CompressedImage;

    using utility::support::Expression;

    FakeCamera::FakeCamera(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("FakeCamera.yaml").then([this](const Configuration& cfg) {
            // clang-format off
            auto lvl = cfg["log_level"].as<std::string>();
            if (lvl == "TRACE") { this->log_level = NUClear::TRACE; }
            else if (lvl == "DEBUG") { this->log_level = NUClear::DEBUG; }
            else if (lvl == "INFO") { this->log_level = NUClear::INFO; }
            else if (lvl == "WARN") { this->log_level = NUClear::WARN; }
            else if (lvl == "ERROR") { this->log_level = NUClear::ERROR; }
            else if (lvl == "FATAL") { this->log_level = NUClear::FATAL; }
            // clang-format on

            const auto image_folder = cfg["image_folder"].as<std::string>();
            const auto image_prefix = cfg["image_prefix"].as<std::string>();
            const auto lens_prefix  = cfg["lens_prefix"].as<std::string>();

            if ((std::filesystem::is_directory(image_folder) && image_folder != config.image_folder)
                || image_prefix != config.image_prefix || lens_prefix != config.lens_prefix) {
                config.image_folder = image_folder;
                config.image_prefix = image_prefix;
                config.lens_prefix  = lens_prefix;

                {  // mutex scope
                    std::lock_guard<std::mutex> lock(images_mutex);

                    // Clear out old images and reset the index
                    images.clear();
                    image_index = 0;

                    // Traverse specified directory and locate all image/lens pairs
                    for (const auto& p : std::filesystem::recursive_directory_iterator(config.image_folder)) {
                        if (utility::strutil::endsWith(p.path(), std::vector<std::string>{".jpg", ".jpeg"})) {
                            const std::filesystem::path& image_file = p.path();
                            std::filesystem::path lens_file         = p.path();

                            // Replace the filename component and change the extension
                            // This will take image000001.jpg and return lens000001.yaml
                            // Assuming image_prefix = "image" and lens_prefix = "lens"
                            lens_file
                                .replace_filename(lens_file.filename().string().replace(0,
                                                                                        config.image_prefix.length(),
                                                                                        config.lens_prefix))
                                .replace_extension("yaml");
                            log<NUClear::TRACE>(fmt::format("Looking for lens file '{}' for image '{}'",
                                                            lens_file.string(),
                                                            image_file.string()));
                            if (std::filesystem::exists(lens_file)) {
                                log<NUClear::TRACE>(fmt::format("Found lens file '{}'", lens_file.string()));
                                images.emplace_back(image_file.string(), lens_file.string());
                            }
                        }
                    }
                }
            }
        });

        on<Every<1, Per<std::chrono::seconds>>, Single>().then([this] {
            {  // mutex scope
                std::lock_guard<std::mutex> lock(images_mutex);

                if (image_index < images.size()) {
                    auto msg = std::make_unique<CompressedImage>();

                    // Extract lens parameters from the lens file
                    YAML::Node lens        = YAML::LoadFile(images[image_index].second);
                    msg->lens.projection   = lens["projection"].as<std::string>();
                    msg->lens.focal_length = lens["focal_length"].as<float>();
                    msg->lens.fov          = lens["fov"].as<float>();
                    msg->lens.centre       = lens["centre"].as<Expression>();
                    msg->lens.k            = lens["k"].as<Expression>();

                    // Convert Hoc from lens file to Hcw. We are assuming that the observation plane corresponds to the
                    // world
                    const Eigen::Isometry3d Hoc(Eigen::Matrix4d(lens["Hoc"].as<Expression>()));
                    msg->Hcw = Hoc.inverse().matrix();

                    // Set image properties
                    msg->format    = utility::vision::fourcc("JPEG");
                    msg->id        = 0;
                    msg->name      = "FakeCamera";
                    msg->timestamp = NUClear::clock::now();

                    // Load image file into vector
                    msg->data = utility::file::readFile(images[image_index].first);

                    // Extract file dimensions from file data
                    std::array<int, 2> dimensions{};
                    int subsamp = 0;
                    if (tjDecompressHeader2(decompressor.get(),
                                            msg->data.data(),
                                            msg->data.size(),
                                            &dimensions[0],
                                            &dimensions[1],
                                            &subsamp)
                        == 0) {

                        // Set the dimensions
                        msg->dimensions[0] = dimensions[0];
                        msg->dimensions[1] = dimensions[1];

                        // Normalise the focal length
                        msg->lens.focal_length /= msg->dimensions[0];

                        emit(msg);
                    }
                    else {
                        log<NUClear::DEBUG>(fmt::format("Failed to extract image dimensions from JPEG data for '{}'",
                                                        images[image_index].first));
                    }
                }
                if (++image_index >= images.size()) {
                    image_index = 0;
                }
            }
        });
    }

}  // namespace module::input
