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
            Eigen::Vector2d dims(1280.0, 1024.0);  // width, height

            // Build lens intrinsics from YAML-normalised values
            utility::vision::Lens lens;
            lens.k << -0.04247437, -0.0076668, 0.00308963, -0.00099647;
            lens.focal_length = 0.3446512345140941 * dims.x();

            Eigen::Vector2d image_center  = dims * 0.5;
            Eigen::Vector2d centre_offset = Eigen::Vector2d(0.005418370784106563, -0.08661520006894258) * dims.x();
            lens.centre                   = image_center + centre_offset;

            // Set of rays to test
            std::vector<Eigen::Vector3d> test_rays = {
                {0.0, 0.0, 1.0},
                {0.1, 0.7, 0.7},
                {0.0, 1.0, 1.0},
                {1.0, 0.0, 1.0},
                {-1.0, 0.0, 1.0},
                {0.5, -0.5, 1.0},
                {-0.5, 0.5, 1.0},
                {0.0, -1.0, 1.0},
            };

            for (const auto& ray : test_rays) {
                Eigen::Vector3d ray_n = ray.normalized();
                Eigen::Vector2d px    = utility::vision::project(ray_n, lens);
                Eigen::Vector3d back  = utility::vision::unproject(px, lens);

                double err = (ray_n - back.normalized()).norm();

                log<INFO>("Ray:     ", ray_n.transpose());
                log<INFO>("→ Pixel: ", px.transpose());
                log<INFO>("→ Back:  ", back.transpose());
                log<INFO>("→ Error: ", err);
            }
        });
    }

}  // namespace module::vision
