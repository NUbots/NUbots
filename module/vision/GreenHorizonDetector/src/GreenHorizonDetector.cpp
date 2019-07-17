#include "GreenHorizonDetector.h"

#include <fmt/format.h>
#include <numeric>
#include <set>

#include "extension/Configuration.h"

#include "message/vision/GreenHorizon.h"
#include "message/vision/VisualMesh.h"

#include "utility/math/geometry/ConvexHull.h"
#include "utility/vision/visualmesh/VisualMesh.h"

namespace module {
namespace vision {

    using extension::Configuration;

    using message::vision::VisualMesh;
    using GreenHorizonMsg = message::vision::GreenHorizon;

    static constexpr int LINE_INDEX  = 2;
    static constexpr int FIELD_INDEX = 3;

    GreenHorizonDetector::GreenHorizonDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("GreenHorizonDetector.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file GreenHorizonDetector.yaml
            config.confidence_threshold = cfg["confidence_threshold"].as<float>();
            config.cluster_points       = cfg["cluster_points"].as<uint>();
            config.distance_offset      = cfg["distance_offset"].as<float>();
            config.debug                = cfg["debug"].as<bool>();
        });

        on<Trigger<VisualMesh>, Buffer<2>>().then("Green Horizon", [this](const VisualMesh& mesh) {
            // Convenience variables
            const auto& cls                                     = mesh.classifications;
            const auto& neighbours                              = mesh.neighbourhood;
            const Eigen::Matrix<float, 3, Eigen::Dynamic>& rays = mesh.rays;
            const float world_offset                            = std::atan2(mesh.Hcw(0, 1), mesh.Hcw(0, 0));

            // Get some indices to partition
            std::vector<int> indices(mesh.indices.size());
            std::iota(indices.begin(), indices.end(), 0);

            // Partition the indices such that we only have the field points that dont have field surrounding them
            auto boundary = utility::vision::visualmesh::partition_points(
                indices.begin(),
                indices.end(),
                neighbours,
                [&](const int& idx) {
                    return idx == indices.size()
                           || (cls(FIELD_INDEX, idx) + cls(LINE_INDEX, idx) >= config.confidence_threshold);
                },
                {4, 5});


            // Discard indices that are not on the boundary
            indices.resize(std::distance(indices.begin(), boundary));

            if (config.debug) {
                log<NUClear::DEBUG>(fmt::format("Partitioned {} points", indices.size()));
            }

            // Cluster the points
            // Points are clustered based on their connectivity to other field points
            // Clustering is down in two steps
            // 1) We take the set of field points found above and partition them into potential clusters by
            //    a) Add the first point and its field neighbours to a cluster
            //    b) Find all other field points who are neighbours of the points in the cluster
            //    c) Partition all of the indices that are in the cluster
            //    d) Repeat a-c for all points that were not partitioned
            //    e) Delete all partitions smaller than a given threshold
            // 2) Merge the clusters down into a single cluster
            //    Clusters are merged when they are not overlapping
            //    If the clusters do overlap then we keep the largest one
            std::vector<std::vector<int>> clusters;
            utility::vision::visualmesh::cluster_points(
                indices.begin(), indices.end(), neighbours, config.cluster_points, clusters);

            if (config.debug) {
                log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));
            }

            // Merge clusters until only 1 remains
            while (clusters.size() > 1) {
                for (auto it = clusters.begin(); it != clusters.end(); it = std::next(it)) {
                    // Get the largest and smallest theta values
                    auto range_a = std::minmax_element(it->begin(), it->end(), [&rays](const int& a, const int& b) {
                        return std::atan2(rays(1, a), rays(0, a)) < std::atan2(rays(1, b), rays(0, b));
                    });

                    const float min_a = std::atan2(rays(1, *range_a.first), rays(0, *range_a.first));
                    const float max_a = std::atan2(rays(1, *range_a.second), rays(0, *range_a.second));

                    for (auto it2 = std::next(it); it2 != clusters.end();) {
                        // Get the largest and smallest theta values
                        auto range_b =
                            std::minmax_element(it2->begin(), it2->end(), [&rays](const int& a, const int& b) {
                                return std::atan2(rays(1, a), rays(0, a)) < std::atan2(rays(1, b), rays(0, b));
                            });

                        const float min_b = std::atan2(rays(1, *range_b.first), rays(0, *range_b.first));
                        const float max_b = std::atan2(rays(1, *range_b.second), rays(0, *range_b.second));

                        // Ranges do not overlap
                        // Merge the clusters
                        if ((max_a < min_b) || (max_b < min_a)) {
                            // Append the second cluster on to the first
                            it->insert(it->end(), it2->begin(), it2->end());
                            // Delete the second cluster
                            it2 = clusters.erase(it2);
                        }
                        // Second cluster is overlapping first cluster either on the left or the right
                        // Keep the largest cluster
                        else if (((min_a <= min_b) && (min_b <= max_a)) || ((min_b <= min_a) && (min_a <= max_b))) {
                            // Delete the second cluster
                            it2 = clusters.erase(it2);
                        }
                        else {
                            if (config.debug) {
                                log<NUClear::DEBUG>(
                                    "The clusters are neither overlapping, nor are they not overlapping. What have you "
                                    "done???");
                                log<NUClear::DEBUG>(fmt::format("[{}, {}] -> [{}, {}], [{}, {}] -> [{}, {}]",
                                                                *range_a.first,
                                                                *range_a.second,
                                                                min_a,
                                                                max_a,
                                                                *range_b.first,
                                                                *range_b.second,
                                                                min_b,
                                                                max_b));
                            }
                        }
                    }
                }
            }

            if (clusters.size() < 1) {
                if (config.debug) {
                    log<NUClear::DEBUG>("Found no clusters to make a convex hull from");
                }
            }
            else if (clusters.front().size() < 3) {
                if (config.debug) {
                    log<NUClear::DEBUG>("Unable to make a convex hull with less than 3 points");
                }
            }
            else {
                if (config.debug) {
                    log<NUClear::DEBUG>(fmt::format("Making a convex hull from {} points", clusters.front().size()));
                }
                // Find the convex hull of the cluster
                auto hull_indices = utility::math::geometry::upper_convex_hull(clusters.front(), rays, world_offset);

                auto msg = std::make_unique<GreenHorizonMsg>();

                // Preserve mesh so that anyone using the GreenHorizon can access the original data
                msg->mesh = const_cast<VisualMesh*>(&mesh)->shared_from_this();

                msg->camera_id = mesh.camera_id;
                msg->Hcw       = mesh.Hcw;
                msg->timestamp = mesh.timestamp;

                // Find the convex hull of the cluster
                msg->horizon.reserve(hull_indices.size());
                for (const auto& idx : hull_indices) {
                    const Eigen::Vector3f ray      = rays.col(idx);
                    const float d                  = mesh.Hcw(2, 3) / ray.z();
                    Eigen::Vector3f ray_projection = ray * d;
                    const float norm               = ray_projection.head<2>().norm();
                    ray_projection.head<2>() *= 1.0f + config.distance_offset / norm;
                    msg->horizon.emplace_back(ray_projection.normalized());
                }

                if (config.debug) {
                    log<NUClear::DEBUG>(fmt::format("Calculated convex hull with {} points from cluster with {} points",
                                                    hull_indices.size(),
                                                    clusters.front().size()));
                }

                emit(std::move(msg));
            }
        });
    }  // namespace vision
}  // namespace vision
}  // namespace module
