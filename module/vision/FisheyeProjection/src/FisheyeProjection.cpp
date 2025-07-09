#include "FisheyeProjection.hpp"

#include <Eigen/Core>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
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
            message::input::Image::Lens lens;
            lens.k << -0.04247437, -0.0076668, 0.00308963, -0.00099647;
            lens.fx = 0.3446512345140941 * dims.x();
            lens.fy = 0.430567319 * dims.y();

            Eigen::Vector2d image_center = dims * 0.5;
            Eigen::Vector2d norm_offset(0.00541837, -0.0866152);
            Eigen::Vector2d pixel_offset(norm_offset.x() * dims.x(), norm_offset.y() * dims.y());
            lens.centre = (image_center + pixel_offset).cast<float>();

            // Set of rays to test
            std::vector<Eigen::Vector3d> test_rays = {
                {1.0, 0.0, 0.0},    // Straight forward (optical axis) - should hit image center
                {1.0, 0.1, 0.0},    // Slightly left
                {1.0, -0.1, 0.0},   // Slightly right
                {1.0, 0.0, 0.1},    // Slightly up
                {1.0, 0.0, -0.1},   // Slightly down
                {1.0, 0.3, 0.3},    // Forward + left + up (moderate angle)
                {1.0, -0.3, 0.3},   // Forward + right + up
                {1.0, 0.3, -0.3},   // Forward + left + down
                {1.0, -0.3, -0.3},  // Forward + right + down
                {0.7, 0.5, 0.5},    // Wider angle (about 45°)
                {0.5, 0.7, 0.3},    // Even wider angle
                {0.3, 0.8, 0.5},    // Very wide angle (should still work for fisheye)
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
