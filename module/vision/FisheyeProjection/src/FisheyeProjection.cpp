#include "FisheyeProjection.hpp"

#include <Eigen/Core>

#include "extension/Configuration.hpp"

#include "message/output/CompressedImage.hpp"

#include "utility/vision/projection.hpp"

namespace module::vision {

    using extension::Configuration;
    using message::output::CompressedImage;

    FisheyeProjection::FisheyeProjection(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("FisheyeProjection.yaml").then([this](const Configuration& config) {
            cfg.log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<CompressedImage>>().then("Fisheye Projection Test", [this](const CompressedImage& image) {
            struct LocalLens {
                Eigen::Matrix<double, 4, 1> k;
                double focal_length;
                Eigen::Vector2d centre;
            };

            // Kevin's camera intrinsics (hardcoded for now)
            LocalLens lens;
            lens.k << -0.04247437, -0.0076668, 0.00308963, -0.00099647;
            lens.focal_length = 0.3446512345140941;
            lens.centre       = Eigen::Vector2d(0.005418370784106563, -0.08661520006894258);

            // Known image dimensions from Kevin's camera
            Eigen::Vector2d dims(1280.0, 1024.0);  // width, height

            // Project a ray and test
            Eigen::Vector3d ray(0.1, 0.7, 0.7);
            Eigen::Vector2d px = utility::vision::project(ray.normalized(), lens, dims);
            log<INFO>("Projected ray ", ray.transpose(), " to pixel ", px.transpose());
            Eigen::Vector3d back = utility::vision::unproject(px, lens, dims);
            log<INFO>("Unprojected back to ", back.transpose());
        });
    }

}  // namespace module::vision
