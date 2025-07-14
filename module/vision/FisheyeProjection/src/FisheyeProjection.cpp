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
            // ========================================
            // TEST 1: COORDINATE SYSTEM VERIFICATION
            // ========================================
            log<INFO>("=== COORDINATE SYSTEM TEST ===");

            // Your transformation matrices (should match projection.hpp)
            Eigen::Matrix3d R_nubots_to_opencv;
            R_nubots_to_opencv << 0, -1, 0, 0, 0, -1, 1, 0, 0;

            Eigen::Matrix3d R_opencv_to_nubots = R_nubots_to_opencv.transpose();

            // Test unit vectors
            std::vector<std::pair<std::string, Eigen::Vector3d>> nubots_vectors = {
                {"X-forward", Eigen::Vector3d(1, 0, 0)},
                {"Y-left", Eigen::Vector3d(0, 1, 0)},
                {"Z-up", Eigen::Vector3d(0, 0, 1)}};

            log<INFO>("NUbots -> OpenCV transformations:");
            for (const auto& [name, vec] : nubots_vectors) {
                Eigen::Vector3d opencv_vec = R_nubots_to_opencv * vec;
                log<INFO>(name, " [", vec.transpose(), "] -> [", opencv_vec.transpose(), "]");
            }

            // Verify transformation matrices are inverses
            Eigen::Matrix3d identity = R_nubots_to_opencv * R_opencv_to_nubots;
            double error             = (identity - Eigen::Matrix3d::Identity()).norm();
            log<INFO>("Transformation matrix inverse error: ", error, (error < 1e-15 ? " PASS" : " FAIL"));

            // ========================================
            // TEST 2: LENS PARAMETER CONSISTENCY
            // ========================================
            log<INFO>("=== LENS PARAMETER TEST ===");

            Eigen::Vector2d dims(1280.0, 1024.0);

            // Build calibrated lens - USING CONSISTENT UNITS
            message::input::Image::Lens lens;
            lens.k << -0.04247437, -0.0076668, 0.00308963, -0.00099647;

            // Method 1: Your current calculation
            lens.fx = 0.3446512345140941 * dims.x();  // 441.15
            lens.fy = 0.430567319 * dims.y();         // 441.06

            Eigen::Vector2d image_center = dims * 0.5;  // [640, 512]
            Eigen::Vector2d norm_offset(0.00541837, -0.0866152);
            lens.centre = (image_center + norm_offset.cwiseProduct(dims)).cast<float>();

            log<INFO>("Calculated lens parameters:");
            log<INFO>("fx: ", lens.fx, ", fy: ", lens.fy);
            log<INFO>("centre: [", lens.centre.x(), ", ", lens.centre.y(), "]");
            log<INFO>("Image center: [", image_center.x(), ", ", image_center.y(), "]");

            // Method 2: Direct from Left.yaml values for comparison
            message::input::Image::Lens lens_direct;
            lens_direct.fx = 441.15;
            lens_direct.fy = 440.01;
            lens_direct.centre << 646.93, 423.30;
            lens_direct.k = lens.k;  // Same distortion coefficients

            log<INFO>("Direct lens parameters from Left.yaml:");
            log<INFO>("fx: ", lens_direct.fx, ", fy: ", lens_direct.fy);
            log<INFO>("centre: [", lens_direct.centre.x(), ", ", lens_direct.centre.y(), "]");

            // Compare the two approaches
            log<INFO>("Differences:");
            log<INFO>("fx diff: ", std::abs(lens.fx - lens_direct.fx));
            log<INFO>("fy diff: ", std::abs(lens.fy - lens_direct.fy));
            log<INFO>("centre diff: ", (lens.centre - lens_direct.centre).norm());

            // ========================================
            // TEST 3: ROUND-TRIP PROJECTION TEST
            // ========================================
            log<INFO>("=== ROUND-TRIP PROJECTION TEST ===");

            std::vector<Eigen::Vector2d> test_pixels = {
                // Near optical center (should work perfectly)
                {lens_direct.centre.x(), lens_direct.centre.y()},            // Optical center
                {lens_direct.centre.x() + 10, lens_direct.centre.y() + 10},  // Near optical center

                // Moderate distances (typical object detection range)
                {lens_direct.centre.x() + 100, lens_direct.centre.y()},  // 100px from center
                {lens_direct.centre.x() + 200, lens_direct.centre.y()},  // 200px from center
                {lens_direct.centre.x(), lens_direct.centre.y() + 300},  // 300px from center

                // Larger but reasonable distances
                {lens_direct.centre.x() + 400, lens_direct.centre.y()},  // 400px from center
                {lens_direct.centre.x(), lens_direct.centre.y() + 500},  // 500px from center

                // Image boundaries but not corners
                {50, lens_direct.centre.y()},    // Left edge center
                {1230, lens_direct.centre.y()},  // Right edge center
                {lens_direct.centre.x(), 50},    // Top edge center
                {lens_direct.centre.x(), 974},   // Bottom edge center

                // Moderate corners (75% toward corners)
                {lens_direct.centre.x() * 0.25, lens_direct.centre.y() * 0.25},  // 75% to top-left
                {lens_direct.centre.x() + (1280 - lens_direct.centre.x()) * 0.75,
                 lens_direct.centre.y() + (1024 - lens_direct.centre.y()) * 0.75}  // 75% to bottom-right
            };

            log<INFO>("Testing round-trip: pixel -> ray -> pixel");
            double max_error = 0.0;
            for (const auto& original_pixel : test_pixels) {
                // Forward: pixel -> ray
                Eigen::Vector3d ray = utility::vision::unproject(original_pixel, lens_direct);

                // Backward: ray -> pixel
                Eigen::Vector2d reprojected_pixel = utility::vision::project(ray, lens_direct);

                // Calculate error
                double error_magnitude = (original_pixel - reprojected_pixel).norm();
                max_error              = std::max(max_error, error_magnitude);

                log<INFO>("(",
                          original_pixel.x(),
                          ", ",
                          original_pixel.y(),
                          ") -> ",
                          "(",
                          reprojected_pixel.x(),
                          ", ",
                          reprojected_pixel.y(),
                          ") ",
                          "error: ",
                          error_magnitude,
                          " pixels");
            }

            log<INFO>("Maximum round-trip error: ",
                      max_error,
                      " pixels ",
                      (max_error < 0.1   ? "EXCELLENT"
                       : max_error < 1.0 ? "GOOD"
                                         : "POOR"));

            // ========================================
            // TEST 4: YOUR ORIGINAL FIELD PROJECTION TEST
            // ========================================
            log<INFO>("=== FIELD PROJECTION TEST ===");

            // Define camera transform (same for all tests)
            Eigen::Isometry3d Hcw = Eigen::Isometry3d::Identity();
            Hcw.translate(Eigen::Vector3d(0.0, 0.0, 0.45));
            Hcw.rotate(Eigen::AngleAxisd(-5.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()));
            Eigen::Isometry3d Hwc = Hcw.inverse();

            // Define test points on the field
            std::vector<Eigen::Vector3d> field_points = {{1.0, 0.0, 0.0},    // straight ahead
                                                         {1.0, 0.5, 0.0},    // left
                                                         {1.0, -0.5, 0.0},   // right
                                                         {1.0, 0.0, 0.3},    // up
                                                         {1.0, 0.0, -0.3},   // down
                                                         {1.0, 0.3, 0.3},    // up-left
                                                         {1.0, -0.3, 0.3},   // up-right
                                                         {1.0, 0.3, -0.3},   // down-left
                                                         {1.0, -0.3, -0.3},  // down-right
                                                         {0.7, 0.7, 0.0},    // off-axis
                                                         {0.7, -0.7, 0.0},
                                                         {0.5, 0.8, 0.2},  // wide angle
                                                         {0.5, -0.8, -0.2}};

            double total_error    = 0.0;
            int valid_projections = 0;

            for (const auto& rPWw : field_points) {
                // Project and unproject cycle
                Eigen::Vector3d rPCc = Hcw * rPWw;

                // Skip points behind the camera
                if (rPCc.x() <= 0) {
                    log<WARN>("Point behind camera, skipping: ", rPWw.transpose());
                    continue;
                }

                Eigen::Vector3d ray = rPCc.normalized();
                Eigen::Vector2d px  = utility::vision::project(ray, lens_direct);

                // Check if projection is within image bounds
                if (px.x() < 0 || px.x() > dims.x() || px.y() < 0 || px.y() > dims.y()) {
                    log<WARN>("Projection outside image bounds: ", px.transpose(), " for point: ", rPWw.transpose());
                    continue;
                }

                Eigen::Vector3d ray_back  = utility::vision::unproject(px, lens_direct);
                Eigen::Vector3d ray_world = Hwc.rotation() * ray_back;

                // Calculate the scale factor 't' such that the reconstructed point has the same z as the original
                double t                        = (rPWw.z() - Hwc.translation().z()) / ray_world.z();
                Eigen::Vector3d r_reconstructed = ray_world * t + Hwc.translation();

                double error = (r_reconstructed - rPWw).norm();
                total_error += error;
                valid_projections++;

                // Log
                log<INFO>("----------------------------");
                log<INFO>("Input point: [", rPWw.transpose(), "]");
                log<INFO>("Camera coords: [", rPCc.transpose(), "]");
                log<INFO>("Projected px: [", px.transpose(), "]");
                log<INFO>("Reconstructed: [", r_reconstructed.transpose(), "]");
                log<INFO>("Error: ", error, " meters");
            }

            if (valid_projections > 0) {
                double avg_error = total_error / valid_projections;
                log<INFO>("=== SUMMARY ===");
                log<INFO>("Valid projections: ", valid_projections, "/", field_points.size());
                log<INFO>("Average reconstruction error: ", avg_error, " meters");
                log<INFO>("Error assessment: ",
                          (avg_error < 0.01   ? "EXCELLENT"
                           : avg_error < 0.05 ? "GOOD"
                           : avg_error < 0.1  ? "ACCEPTABLE"
                                              : "POOR"));
            }
        });
    }

}  // namespace module::vision
