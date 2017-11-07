#include <cmath>

#include "Reprojection.h"

#include "extension/Configuration.h"

#include "message/input/CameraParameters.h"
#include "message/input/Image.h"
#include "message/vision/ReprojectedImage.h"

#include "utility/math/vision.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"

namespace module {
namespace vision {

    using extension::Configuration;

    using message::input::CameraParameters;
    using message::input::Image;
    using message::vision::ReprojectedImage;

    using utility::math::vision::pinhole::getCamFromScreen;
    using utility::math::vision::radial::projectCamSpaceToScreen;

    Reprojection::Reprojection(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Reprojection.yaml").then([this](const Configuration& config) {
            arma::vec dimensions = config["output"]["dimensions"].as<arma::vec>();
            output_dimensions    = arma::conv_to<arma::uvec>::from(dimensions);
            tan_half_FOV         = std::tan(config["output"]["FOV"].as<double>() * M_PI / 360.0);
        });

        on<Trigger<Image>, With<CameraParameters>, Single>().then(
            "Image Reprojection", [this](const Image& image, const CameraParameters& cam) {
                arma::vec2 center =
                    arma::conv_to<arma::vec>::from(convert<unsigned int, 2>(image.dimensions) - 1) * 0.5;
                arma::vec2 output_center = arma::conv_to<arma::vec>::from(output_dimensions - 1) * 0.5;

                CameraParameters pinhole;
                pinhole.pinhole.focalLengthPixels = arma::norm(output_center) / tan_half_FOV;

                ReprojectedImage msg;
                msg.format        = image.format;
                msg.dimensions    = convert<unsigned int, 2>(output_dimensions);
                msg.data          = std::vector<uint8_t>(output_dimensions[0] * output_dimensions[1], 0);
                msg.camera_id     = image.camera_id;
                msg.serial_number = image.serial_number;
                msg.timestamp     = image.timestamp;
                msg.Hcw           = image.Hcw;

                for (size_t yu = 0; yu < output_dimensions[1]; yu++) {
                    for (size_t xu = 0; xu < output_dimensions[0]; xu++) {
                        arma::vec2 point_r  = output_center - arma::vec2({double(xu), double(yu)});
                        arma::vec2 point    = projectCamSpaceToScreen(getCamFromScreen(point_r, pinhole), cam);
                        arma::ivec2 point_s = arma::conv_to<arma::ivec>::from(center - point);

                        if ((int(point_s[0]) >= 0) && (int(point_s[0]) < image.dimensions[0]) && (int(point_s[1]) >= 0)
                            && (int(point_s[1]) < image.dimensions[1])) {

                            msg.data[xu + yu * output_dimensions[0]] =
                                image.data[point_s[0] + point_s[1] * image.dimensions[0]];
                        }
                    }
                }

                emit(std::make_unique<ReprojectedImage>(msg));
            });
    }
}  // namespace vision
}  // namespace module
