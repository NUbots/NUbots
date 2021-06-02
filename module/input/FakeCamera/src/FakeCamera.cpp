#include "FakeCamera.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include "extension/Configuration.hpp"

#include "message/output/CompressedImage.hpp"

#include "utility/file/fileutil.hpp"
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

            const std::string image_folder = cfg["image_folder"].as<std::string>();
            const std::string image_prefix = cfg["image_prefix"].as<std::string>();
            const std::string lens_prefix  = cfg["lens_prefix"].as<std::string>();

            if ((utility::file::isDir(image_folder) && image_folder != config.image_folder)
                || image_prefix != config.image_prefix || lens_prefix != config.lens_prefix) {
                config.image_folder = image_folder;
                config.image_prefix = image_prefix;
                config.lens_prefix  = lens_prefix;

                {  // mutex scope
                    std::lock_guard<std::mutex> lock(images_mutex);

                    images.clear();
                    image_counter = 0;
                    for (const auto& file : utility::file::listFiles(config.image_folder, true)) {
                        if (utility::strutil::endsWith(file, std::vector<std::string>{".jpg", ".jpeg"})) {
                            std::string image_file = file;
                            std::string lens_file  = file;
                            lens_file.replace(config.image_folder.length() + 1,
                                              config.image_prefix.length(),
                                              config.lens_prefix);
                            lens_file.replace(lens_file.begin() + lens_file.find_last_of("."),
                                              lens_file.end(),
                                              ".yaml");
                            log<NUClear::TRACE>(
                                fmt::format("Looking for lens file '{}' for image '{}'", lens_file, image_file));
                            if (utility::file::exists(lens_file)) {
                                log<NUClear::TRACE>(fmt::format("Found lens file '{}'", lens_file));
                                images.emplace_back(image_file, lens_file);
                            }
                        }
                    }
                }
            }
        });

        on<Every<1, Per<std::chrono::seconds>>, Single>().then([this] {
            {  // mutex scope
                std::lock_guard<std::mutex> lock(images_mutex);

                if (image_counter < images.size()) {
                    auto msg                     = std::make_unique<message::output::CompressedImage>();
                    YAML::Node lens              = YAML::LoadFile(images[image_counter].second);
                    const std::string projection = lens["projection"].as<std::string>();
                    if (projection == "RECTILINEAR") {
                        msg->lens.projection = CompressedImage::Lens::Projection::RECTILINEAR;
                    }
                    else if (projection == "EQUIDISTANT") {
                        msg->lens.projection = CompressedImage::Lens::Projection::EQUIDISTANT;
                    }
                    else if (projection == "EQUISOLID") {
                        msg->lens.projection = CompressedImage::Lens::Projection::EQUISOLID;
                    }
                    else {
                        msg->lens.projection = CompressedImage::Lens::Projection::UNKNOWN;
                    }
                    msg->lens.focal_length = lens["focal_length"].as<float>();
                    msg->lens.fov          = lens["fov"].as<float>();
                    msg->lens.centre       = lens["centre"].as<Expression>();
                    msg->lens.k            = lens["k"].as<Expression>();
                    msg->format            = utility::vision::fourcc("JPEG");
                    msg->id                = 0;
                    msg->name              = "FakeCamera";
                    msg->timestamp         = NUClear::clock::now();
                    const Eigen::Affine3d Hoc(Eigen::Matrix4d(lens["Hoc"].as<Expression>()));
                    msg->Hcw = Hoc.inverse().matrix();

                    // Load image file into vector
                    msg->data = utility::file::readFile(images[image_counter].first);

                    // Extract file dimensions from file data
                    std::array<int, 2> dimensions;
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
                if (++image_counter >= images.size()) {
                    image_counter = 0;
                }
            }
        });
    }

}  // namespace module::input
