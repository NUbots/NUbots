#include "GreenHorizonDetector.h"

#include <fmt/format.h>
#include <set>

#include "extension/Configuration.h"

#include "message/vision/GreenHorizon.h"
#include "message/vision/VisualMesh.h"

#include "utility/math/geometry/ConvexHull.h"
#include "utility/math/vision.h"

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
            config.debug                = cfg["debug"].as<bool>();
        });

        on<Trigger<VisualMesh>, Buffer<2>>().then("Green Horizon", [this](const VisualMesh& mesh) {
            // Convenience variables
            const auto& cls                                     = mesh.classifications;
            const auto& neighbours                              = mesh.neighbourhood;
            const Eigen::Matrix<float, Eigen::Dynamic, 3>& rays = mesh.rays;
            const float world_offset                            = std::atan2(mesh.Hcw(0, 1), mesh.Hcw(0, 0));

            // Get some indices to partition
            std::vector<int> indices(mesh.indices.size());
            std::iota(indices.begin(), indices.end(), 0);

            // Partition the indices such that we only have the field points that dont have field surrounding them
            auto boundary = std::partition(indices.begin(), indices.end(), [&](const int& idx) {
                // If point is not a field or field line point then we also ignore it
                if ((cls(idx, FIELD_INDEX) + cls(idx, LINE_INDEX)) < config.confidence_threshold) {
                    return false;
                }
                // If at least one neighbour is not a field or a field line then this point should be on the edge
                for (int n = 4; n < 6; ++n) {
                    const int neighbour_idx = neighbours(idx, n);
                    if (neighbour_idx == indices.size()) {
                        continue;
                    }
                    if ((cls(neighbour_idx, FIELD_INDEX) + cls(neighbour_idx, LINE_INDEX))
                        < config.confidence_threshold) {
                        return true;
                    }
                }
                return false;
            });

            // Discard indices that are not on the boundary
            indices.resize(std::distance(indices.begin(), boundary));

            // Sort indices by increasing theta
            utility::math::geometry::sort_by_theta(indices.begin(), indices.end(), rays, world_offset);

            // Cluster the points
            // Points are clustered based on their connectivity to other field points
            // Clustering is down in three steps
            // 1) We take the set of field points found above and partition them into potential clusters by
            //    a) Add the first point and its field neighbours to a cluster
            //    b) Find all other field points who are neighbours of the points in the cluster
            //    c) Partition all of the indices that are in the cluster
            //    d) Repeat a-c for all points that were not partitioned
            // 2) Discard all partitions smaller than a given threshold
            //    This also copies the indices for partitions larger than the threshold into separate vectors
            // 3) Merge the clusters down into a single clulster
            //    Clusters are merged when they are not overlapping
            //    If the clusters do overlap then we keep the largest one
            auto first                                         = indices.begin();
            std::vector<std::vector<int>::iterator> partitions = {first};

            while (first != indices.end()) {
                // Add the first point and all of its neighbours to the cluster
                std::vector<int> cluster = {*first};
                for (int n = 0; n < 6; ++n) {
                    const int neighbour_idx = neighbours(*first, n);
                    if (neighbour_idx == indices.size()) {
                        continue;
                    }
                    if (std::find(indices.begin(), indices.end(), neighbour_idx) == cluster.end()) {
                        cluster.push_back(neighbour_idx);
                    }
                }
                partitions.push_back(std::partition(std::next(first), indices.end(), [&](const int& idx) {
                    // At least one neighbour of the current point must be a neighbour of one of the points in the
                    // current cluster
                    for (const auto& search_idx : cluster) {
                        for (int n = 0; n < 6; ++n) {
                            const int neighbour_idx = neighbours(search_idx, n);
                            // Make sure we haven't already considered this point
                            if (std::find(cluster.begin(), cluster.end(), idx) != cluster.end()) {
                                return true;
                            }
                            if (neighbour_idx == indices.size()) {
                                cluster.push_back(idx);
                                return true;
                            }
                            if (neighbour_idx == idx) {
                                cluster.push_back(idx);
                                return true;
                            }
                        }
                    }
                    return false;
                }));

                first = partitions.back();
            }

            if (config.debug) {
                log<NUClear::DEBUG>(fmt::format("Found {} partitions", partitions.size()));
            }

            // Discard all partitions smaller than a given size
            // Copy the indices of the large clusters into separate vectors
            std::vector<std::vector<int>> clusters;
            for (int i = 1; i < partitions.size(); ++i) {
                if (std::distance(partitions[i - 1], partitions[i]) >= config.cluster_points) {
                    clusters.emplace_back(partitions[i - 1], partitions[i]);
                }
            }

            if (config.debug) {
                log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));
            }

            // Merge clusters until only 1 remains
            while (clusters.size() > 1) {
                for (auto it = clusters.begin(); it != clusters.end(); it = std::next(it)) {
                    // Get the largest and smallest theta values
                    auto range_a = std::minmax_element(it->begin(), it->end(), [&rays](const auto& a, const auto& b) {
                        return std::atan2(rays(a, 1), rays(a, 0)) < std::atan2(rays(b, 1), rays(b, 2));
                    });

                    const float min_a = std::atan2(rays(*range_a.first, 1), rays(*range_a.first, 0));
                    const float max_a = std::atan2(rays(*range_a.second, 1), rays(*range_a.second, 0));

                    for (auto it2 = std::next(it); it2 != clusters.end();) {
                        // Get the largest and smallest theta values
                        auto range_b =
                            std::minmax_element(it2->begin(), it2->end(), [&rays](const auto& a, const auto& b) {
                                return std::atan2(rays(a, 1), rays(a, 0)) < std::atan2(rays(b, 1), rays(b, 2));
                            });

                        const float min_b = std::atan2(rays(*range_b.first, 1), rays(*range_b.first, 0));
                        const float max_b = std::atan2(rays(*range_b.second, 1), rays(*range_b.second, 0));

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
                        else if (((min_a < min_b) && (min_b < max_a)) || ((min_b < min_a) && (min_a < max_b))) {
                            // Delete the second cluster
                            it2 = clusters.erase(it2);
                        }
                    }
                }
            }

            if (clusters.front().size() < 3) {
                if (config.debug) {
                    log<NUClear::DEBUG>("Unable to make a convex hull with less than 3 points");
                }
            }
            else {
                // Find the convex hull of the cluster
                auto hull_indices = utility::math::geometry::upper_convex_hull(clusters.front(), rays, world_offset);

                auto msg = std::make_unique<GreenHorizonMsg>();

                // Preserve mesh so that anyone using the GreenHorizon can access the original data
                msg->mesh = const_cast<VisualMesh*>(&mesh)->shared_from_this();

                msg->camera_id = mesh.camera_id;
                msg->Hcw       = mesh.Hcw;

                // Find the convex hull of the cluster
                msg->horizon.reserve(hull_indices.size());
                for (const auto& idx : hull_indices) {
                    msg->horizon.emplace_back(rays.row(idx));
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
