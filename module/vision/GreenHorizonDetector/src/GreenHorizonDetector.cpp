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
            config.seed_confidence = cfg["seed_confidence"].as<float>();
            config.end_confidence  = cfg["end_confidence"].as<float>();
            config.cluster_points  = cfg["cluster_points"].as<int>();
            config.debug           = cfg["debug"].as<bool>();
        });

        on<Trigger<VisualMesh>, Buffer<2>>().then("Green Horizon", [this](const VisualMesh& mesh) {
            // Convinence variables
            const auto& cls        = mesh.classifications;
            const auto& neighbours = mesh.neighbourhood;
            const auto& coords     = mesh.coordinates;

            // List of clusters
            std::vector<std::vector<int>> clusters;

            // To keep track of all visited indices
            std::vector<uint8_t> visited(mesh.indices.size(), 0);

            for (size_t i = 0; i < mesh.indices.size(); ++i) {
                if (visited[i] == 0) {
                    // Visiting this point
                    visited[i] = 1;

                    // If this point is confident enough to be a seed point depth first search from here
                    if ((cls(i, LINE_INDEX) + cls(i, FIELD_INDEX)) >= config.seed_confidence) {
                        // Start a new cluster
                        std::vector<int> cluster;
                        std::vector<int> stack;

                        // Start with this point
                        stack.push_back(i);

                        while (!stack.empty()) {
                            // Grab the next point
                            int search_idx = stack.back();
                            stack.pop_back();

                            // If it's good enough continue our search
                            if ((cls(search_idx, LINE_INDEX) + cls(search_idx, FIELD_INDEX)) >= config.end_confidence) {
                                cluster.push_back(search_idx);

                                // Skip the first point as it's always just us
                                for (int j = 0; j < 6; ++j) {
                                    int neighbour_idx = neighbours(search_idx, j);
                                    if ((visited[neighbour_idx] == 0) && (neighbour_idx < mesh.indices.size())) {
                                        visited[neighbour_idx] = 1;
                                        stack.push_back(neighbour_idx);
                                    }
                                }
                            }
                        }

                        // Discard clusters smaller than a given threshold
                        if (cluster.size() >= config.cluster_points) {
                            clusters.emplace_back(std::move(cluster));
                        }
                        else {
                            if (config.debug) {
                                log<NUClear::DEBUG>(fmt::format("Discarding cluster with size {}", cluster.size()));
                            }
                        }
                    }
                }
            }

            // Sort clusters by size
            std::sort(
                clusters.begin(), clusters.end(), [](const auto& a, const auto& b) { return a.size() < b.size(); });

            // Merge or discard clusters until only 1 remains
            if (config.debug) {
                log<NUClear::DEBUG>(fmt::format("Merging {} clusters", clusters.size()));
            }
            while (clusters.size() > 1) {
                for (auto it = clusters.begin(); it != clusters.end(); it = std::next(it)) {
                    // Get the largest and smallest X values
                    auto range_a = std::minmax_element(it->begin(), it->end(), [&coords](const auto& a, const auto& b) {
                        return coords(a, 0) < coords(b, 0);
                    });

                    const int& min_a = *range_a.first;
                    const int& max_a = *range_a.second;

                    for (auto it2 = std::next(it); it2 != clusters.end();) {
                        // Get the largest and smallest X values
                        auto range_b =
                            std::minmax_element(it2->begin(), it2->end(), [&coords](const auto& a, const auto& b) {
                                return coords(a, 0) < coords(b, 0);
                            });

                        const int& min_b = *range_b.first;
                        const int& max_b = *range_b.second;

                        // Ranges do not overlap
                        // Merge the clusters
                        if ((max_a < min_b) || (max_b < min_a)) {
                            // Append the second cluster on to the first
                            it->insert(it->end(), it2->begin(), it2->end());
                            // Delete the second cluster
                            it2 = clusters.erase(it2);
                        }
                        // Second cluster is overlapping first cluster either on the left or the right
                        // Keep the largest cluster, or merge if the clusters are the same size
                        else if (((min_a < min_b) && (min_b < max_a)) || ((min_b < min_a) && (min_a < max_b))) {
                            if (it->size() > it2->size()) {
                                // Delete the second cluster
                                it2 = clusters.erase(it2);
                            }
                            else {
                                // Clusters are the same size
                                // If we have distinct min and max elements for both clusters, merge
                                // them Otherwise, discard the second cluster
                                if ((min_a != min_b) && (max_a != max_b)) {
                                    // Append the second cluster on to the first
                                    it->insert(it->end(), it2->begin(), it2->end());
                                    // Delete the second cluster
                                    it2 = clusters.erase(it2);
                                }
                                else {
                                    // Delete the second cluster
                                    it2 = clusters.erase(it2);
                                }
                            }
                        }
                    }
                }
            }

            if (clusters.empty()) {
                if (config.debug) {
                    log<NUClear::DEBUG>("Failed to find green horizon");
                }
            }
            else {
                // Purge all interior points
                std::cout << "\t\tBefore purge: " << clusters.front().size() << std::endl;
                for (auto it = clusters.front().begin(); it != clusters.front().end();) {
                    bool interior = true;
                    for (int j = 0; j < 6; ++j) {
                        int neighbour_idx = neighbours(*it, j);
                        if ((cls(neighbour_idx, LINE_INDEX) + cls(neighbour_idx, FIELD_INDEX))
                            < config.end_confidence) {
                            interior = false;
                        }
                    }
                    if (interior) {
                        it = clusters.front().erase(it);
                    }
                    else {
                        it = std::next(it);
                    }
                }

                // Sort remaining cluster from left to right
                std::sort(clusters.front().begin(), clusters.front().end(), [&](const int& a, const int& b) {
                    return coords(a, 0) < coords(b, 0);
                });

                auto msg = std::make_unique<GreenHorizonMsg>();

                // Preserve mesh so that anyone using the GreenHorizon can access the original data
                msg->mesh = const_cast<VisualMesh*>(&mesh)->shared_from_this();

                msg->camera_id = mesh.camera_id;
                msg->Hcw       = mesh.Hcw;

                // Find the convex hull of the cluster
                std::vector<int> horizon_indices(utility::math::geometry::graham_scan(clusters.front(), coords));

                msg->horizon.reserve(horizon_indices.size());
                for (const auto& index : horizon_indices) {
                    auto unit = utility::math::vision::getCamFromImage(convert<float, 2>(coords.row(index)),
                                                                       convert<unsigned int, 2>(mesh.image->dimensions),
                                                                       mesh.image->lens);
                    msg->horizon.emplace_back(convert<double, 3>(unit).cast<float>());
                }

                if (config.debug) {
                    log<NUClear::DEBUG>(fmt::format("Calculated convex hull with {} points from cluster with {} points",
                                                    msg->horizon.size(),
                                                    clusters.front().size()));
                }

                emit(std::move(msg));
            }
        });
    }  // namespace vision
}  // namespace vision
}  // namespace module
