#include "FieldLineDetector.h"

#include <Eigen/Core>

#include "extension/Configuration.h"

// TODO: Replace VisualMesh with GreenHorizon
#include "message/vision/Line.h"
#include "message/vision/VisualMesh.h"

#include "utility/math/ransac/Ransac.h"
#include "utility/math/ransac/RansacLineModel.h"
#include "utility/math/ransac/RansacResult.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace vision {

    using extension::Configuration;

    // TODO: Replace VisualMesh with GreenHorizon
    using message::vision::Line;
    using message::vision::Lines;
    using message::vision::VisualMesh;

    using utility::math::ransac::Ransac;
    using utility::math::ransac::RansacLineModel;
    using utility::math::ransac::RansacResult;

    static constexpr int LINE_INDEX = 2;

    FieldLineDetector::FieldLineDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("FieldLineDetector.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file FieldLineDetector.yaml
            config.confidence_threshold       = cfg["confidence_threshold"].as<double>();
            config.min_points_for_consensus   = cfg["min_points_for_consensus"].as<uint>();
            config.max_iterations_per_fitting = cfg["max_iterations_per_fitting"].as<uint>();
            config.max_fitted_models          = cfg["max_fitted_models"].as<uint>();
            config.consensus_error_threshold  = cfg["consensus_error_threshold"].as<double>();
            config.max_angle_difference       = cfg["max_angle_difference"].as<double>();
            config.max_line_distance          = cfg["max_line_distance"].as<double>();
        });

        // TODO: Replace VisualMesh with GreenHorizon
        on<Trigger<VisualMesh>>().then("Field Line Detector", [this](const VisualMesh& mesh) {
            // Convinence variables
            const auto& cls    = mesh.classifications;
            const auto& coords = mesh.coordinates;

            // List of clusters
            std::vector<RansacLineModel::DataPoint> line_points;

            // Find all field line points
            // TODO: Filter this through the green horizon
            for (size_t i = 0; i < mesh.indices.size(); ++i) {
                if (cls(i, LINE_INDEX) > config.confidence_threshold) {
                    line_points.emplace_back(RansacLineModel::DataPoint{coords(i, 0), coords(i, 1)});
                }
            }

            // RANSAC the crap out of the line points
            auto models = Ransac<RansacLineModel>::fitModels(line_points.begin(),
                                                             line_points.end(),
                                                             config.min_points_for_consensus,
                                                             config.max_iterations_per_fitting,
                                                             config.max_fitted_models,
                                                             config.consensus_error_threshold);

            // Merge any lines which are approximately colinear
            for (auto it = std::next(models.begin()); it != models.end();) {
                const RansacLineModel& l0 = std::prev(it)->model;
                const RansacLineModel& l1 = it->model;

                // Lines are nearly parallel and close to each other, discard the second line
                if ((l0.angleBetween(l1) < config.max_angle_difference)
                    && (std::abs(l0.distance - l1.distance) < config.max_line_distance)) {
                    it = models.erase(it);
                }
                else {
                    it = std::next(it);
                }
            }

            auto msg = std::make_unique<Lines>();

            for (auto it = models.begin(); it != models.end(); it = std::next(it)) {
                // Sort line points in order of increasing x and then increasing y
                std::vector<RansacLineModel::DataPoint> points(it->begin(), it->end());
                std::sort(points.begin(),
                          points.end(),
                          [](const RansacLineModel::DataPoint& a, const RansacLineModel::DataPoint& b) {
                              return (a[0] < b[0]) || ((a[0] == b[0]) && a[1] < b[1]);
                          });

                // Add line to message
                msg->lines.emplace_back(mesh.camera_id,
                                        NUClear::clock::now(),
                                        convert<double, 2>(points.front()).cast<int>(),
                                        convert<double, 2>(points.back()).cast<int>(),
                                        Eigen::Vector4d{1.0, 1.0, 1.0, 1.0});
            }

            emit(std::move(msg));
        });
    }
}  // namespace vision
}  // namespace module
