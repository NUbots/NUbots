#include "GreenHorizonDetector.hpp"

#include <fmt/format.h>
#include <numeric>
#include <set>

#include "extension/Configuration.hpp"

#include "message/vision/GreenHorizon.hpp"
#include "message/vision/VisualMesh.hpp"

#include "utility/math/geometry/ChanConvexHull.hpp"
#include "utility/math/geometry/ConvexHull.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

namespace module::vision {

    using extension::Configuration;
    using message::vision::VisualMesh;
    using utility::nusight::graph;
    using GreenHorizonMsg = message::vision::GreenHorizon;

    GreenHorizonDetector::GreenHorizonDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("GreenHorizonDetector.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file GreenHorizonDetector.yaml
            log_level = cfg["log_level"].as<NUClear::LogLevel>();

            config.confidence_threshold = cfg["confidence_threshold"].as<double>();
            config.cluster_points       = cfg["cluster_points"].as<uint>();
            config.distance_offset      = cfg["distance_offset"].as<double>();
        });

        on<Trigger<VisualMesh>, Buffer<2>>().then("Green Horizon", [this](const VisualMesh& mesh) {
            // Convenience variables
            const auto& cls                                      = mesh.classifications;
            const auto& neighbours                               = mesh.neighbourhood;
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& uPCw = mesh.rays.cast<double>();
            const Eigen::Isometry3d Hwc                          = mesh.Hcw.inverse();
            const uint32_t LINE_INDEX                            = mesh.class_map.at("line");
            const uint32_t FIELD_INDEX                           = mesh.class_map.at("field");

            // Convert rays to world space
            Eigen::Matrix<double, 3, Eigen::Dynamic> rPWw(3, uPCw.cols());

            // Loop over each position and add the transformed ray
            for (int idx = 0; idx < uPCw.cols(); idx++) {
                rPWw.col(idx) = uPCw.col(idx) * std::abs(Hwc.translation().z() / uPCw.col(idx).z()) + Hwc.translation();
            }

            // Get some indices to partition
            std::vector<int> indices(mesh.indices.size());
            std::iota(indices.begin(), indices.end(), 0);

            // Partition the indices such that we only have the field points that dont have field surrounding them
            auto boundary = utility::vision::visualmesh::partition_points(
                indices.begin(),
                indices.end(),
                neighbours,
                [&](const int& idx) {
                    return cls(FIELD_INDEX, idx) >= config.confidence_threshold
                           || cls(LINE_INDEX, idx) >= config.confidence_threshold;
                });

            // Discard indices that are not on the boundary
            indices.resize(std::distance(indices.begin(), boundary));

            // Graph the remaining points if debugging
            if (log_level <= NUClear::DEBUG) {
                for (int idx : indices) {
                    emit(graph("Field point", rPWw.col(idx).x(), rPWw.col(idx).y()));
                }
            }

            // The remaining points are on the boundary of the field
            // They may also appear around other objects such as the ball
            log<NUClear::DEBUG>(fmt::format("Found {} points on the boundary of the field to create a convex hull with",
                                            indices.size()));

            // Convex hull algorithms require at least three points
            if (indices.size() < 3) {
                log<NUClear::DEBUG>("Not enough points to make a convex hull");
                return;
            }

            log<NUClear::DEBUG>("Calculating convex hull");

            // Find the convex hull of the field points
            auto hull_indices = utility::math::geometry::chans_convex_hull(indices, rPWw);

            log<NUClear::DEBUG>("Calculated a convex hull");

            for (int idx : hull_indices) {
                emit(graph("Convex hull point", rPWw.col(idx).x(), rPWw.col(idx).y()));
            }

            auto msg = std::make_unique<GreenHorizonMsg>();

            // Preserve mesh so that anyone using the GreenHorizon can access the original data
            msg->mesh = const_cast<VisualMesh*>(&mesh)->shared_from_this();

            msg->id        = mesh.id;
            msg->Hcw       = mesh.Hcw;
            msg->timestamp = mesh.timestamp;
            msg->class_map = mesh.class_map;

            if (mesh.vision_ground_truth.exists) {
                msg->vision_ground_truth = mesh.vision_ground_truth;
            }

            // Find the convex hull of the cluster
            msg->horizon.reserve(hull_indices.size());
            for (const auto& idx : hull_indices) {
                const Eigen::Vector3d ray      = uPCw.col(idx);
                const double d                 = mesh.Hcw(2, 3) / ray.z();
                Eigen::Vector3d ray_projection = ray * d;
                const double norm              = ray_projection.head<2>().norm();
                ray_projection.head<2>() *= 1.0f + config.distance_offset / norm;
                msg->horizon.emplace_back(ray_projection.normalized().cast<float>());
            }
            log<NUClear::DEBUG>(fmt::format("Calculated a convex hull with {} points from a boundary with {} points",
                                            hull_indices.size(),
                                            indices.size()));
            emit(std::move(msg));
        });
    }
}  // namespace module::vision
