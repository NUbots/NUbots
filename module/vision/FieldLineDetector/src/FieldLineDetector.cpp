#include "FieldLineDetector.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <numeric>

#include "extension/Configuration.hpp"

#include "message/vision/FieldLines.hpp"
#include "message/vision/GreenHorizon.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"


namespace module::vision {

    using extension::Configuration;

    using message::vision::FieldLines;
    using message::vision::GreenHorizon;

    FieldLineDetector::FieldLineDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("FieldLineDetector.yaml").then([this](const Configuration& config) {
            // Use configuration here from file FieldLineDetector.yaml
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.confidence_threshold = config["confidence_threshold"].as<float>();
            cfg.cluster_points       = config["cluster_points"].as<int>();
        });

        on<Trigger<GreenHorizon>, Buffer<2>>().then("Field Line Detector", [this](const GreenHorizon& horizon) {
            // Convenience variables
            const auto& cls        = horizon.mesh->classifications;
            const auto& neighbours = horizon.mesh->neighbourhood;
            // Unit vectors from camera to a point in the mesh, in world space
            const Eigen::Matrix<float, 3, Eigen::Dynamic>& uPCw = horizon.mesh->rays;
            const int LINE_INDEX                                = horizon.class_map.at("line");
            // PARTITION INDICES AND CLUSTER
            // Get some indices to partition
            std::vector<int> indices(horizon.mesh->indices.size());
            std::iota(indices.begin(), indices.end(), 0);
            // Partition the indices such that the field line points that have field line points surrounding them are
            // removed, and then resize the vector to remove those points
            auto boundary = utility::vision::visualmesh::partition_points(
                indices.begin(),
                indices.end(),
                neighbours,
                [&](const int& idx) {
                    return idx == int(indices.size()) || (cls(LINE_INDEX, idx) >= cfg.confidence_threshold);
                });
            indices.resize(std::distance(indices.begin(), boundary));
            log<NUClear::DEBUG>(fmt::format("Partitioned {} points", indices.size()));
            std::vector<std::vector<int>> clusters;
            utility::vision::visualmesh::cluster_points(indices.begin(),
                                                        indices.end(),
                                                        neighbours,
                                                        cfg.cluster_points,
                                                        clusters);
            log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));
            // Partition the clusters such that clusters above the green horizons are removed,
            // and then resize the vector to remove them
            auto green_boundary = utility::vision::visualmesh::check_green_horizon_side(clusters.begin(),
                                                                                        clusters.end(),
                                                                                        horizon.horizon.begin(),
                                                                                        horizon.horizon.end(),
                                                                                        uPCw.cast<float>(),
                                                                                        false,
                                                                                        true);
            clusters.resize(std::distance(clusters.begin(), green_boundary));
            log<NUClear::DEBUG>(fmt::format("Found {} clusters below green horizon", clusters.size()));


            // Create the FieldLines message, which will contain a FieldLine for every cluster that is a valid field
            // line
            auto lines            = std::make_unique<FieldLines>();
            lines->id             = horizon.id;         // camera id
            lines->timestamp      = horizon.timestamp;  // time when the image was taken
            lines->Hcw            = horizon.Hcw;        // world to camera transform at the time the image was taken
            Eigen::Isometry3d Hwc = horizon.Hcw.inverse();
            for (auto& cluster : clusters) {
                for (const auto& idx : cluster) {
                    lines->points.push_back(uPCw.col(idx));
                    // Project the field line point onto the field plane
                    Eigen::Vector3f rCWw = Eigen::Vector3f(Hwc.translation().x(), Hwc.translation().y(), 0.0);
                    Eigen::Vector3f rPCw = uPCw.col(idx) * std::abs(Hwc.translation().z() / uPCw.col(idx).z()) + rCWw;
                    lines->rPCw.push_back(rPCw);
                }
            }

            emit(lines);
        });
    }

}  // namespace module::vision
