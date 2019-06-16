#include "GreenHorizonDetector.h"

#include <set>

#include "extension/Configuration.h"

#include "message/vision/GreenHorizon.h"
#include "message/vision/Line.h"
#include "message/vision/VisualMesh.h"

#include "utility/math/geometry/ConvexHull.h"

namespace module {
namespace vision {

    using extension::Configuration;

    using message::vision::Line;
    using message::vision::Lines;
    using message::vision::VisualMesh;
    using GreenHorizonMsg = message::vision::GreenHorizon;

    static constexpr int LINE_INDEX  = 2;
    static constexpr int FIELD_INDEX = 3;

    GreenHorizonDetector::GreenHorizonDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("GreenHorizonDetector.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file GreenHorizonDetector.yaml
            config.seed_confidence  = cfg["seed_confidence"].as<float>();
            config.end_confidence   = cfg["end_confidence"].as<float>();
            config.cluster_points   = cfg["cluster_points"].as<int>();
            config.draw_convex_hull = cfg["debug"]["draw_convex_hull"].as<bool>();
        });

        on<Trigger<VisualMesh>>().then("Green Horizon", [this](const VisualMesh& mesh) {
            // Convinence variables
            const auto& cls        = mesh.classifications;
            const auto& neighbours = mesh.neighbourhood;
            const auto& coords     = mesh.coordinates;

            // List of clusters
            std::vector<std::vector<int>> clusters;

            // To keep track of all visited indices
            std::set<int> visited;

            // Threshold all field and field line detections
            auto cls_seed_thresholded = (cls.col(LINE_INDEX) + cls.col(FIELD_INDEX)).array() > config.seed_confidence;
            auto cls_end_thresholded  = (cls.col(LINE_INDEX) + cls.col(FIELD_INDEX)).array() > config.end_confidence;

            for (size_t i = 0; i < mesh.indices.size(); ++i) {
                if (!visited.count(i)) {
                    // Visiting this point
                    visited.insert(i);

                    // If this point is confident enough to be a seed point breadth first search from here
                    if (cls_seed_thresholded(i)) {
                        // Start a new cluster
                        std::vector<int> cluster;
                        std::vector<int> stack;

                        // Start with this point
                        stack.push_back(i);

                        while (!stack.empty()) {
                            // Grab the next point
                            int idx = stack.back();
                            stack.pop_back();

                            // If it's good enough continue our search
                            if (cls_end_thresholded(i)) {
                                cluster.push_back(idx);

                                // Skip the first point as it's always just us
                                for (int j = 1; j < 7; ++j) {
                                    int idx = neighbours(i, j);
                                    if (!visited.count(idx)) {
                                        visited.insert(idx);
                                        stack.push_back(idx);
                                    }
                                }
                            }
                        }

                        clusters.emplace_back(std::move(cluster));
                    }
                }
            }

            // Discard clusters smaller than a given threshold
            clusters.erase(std::remove_if(clusters.begin(),
                                          clusters.end(),
                                          [this](const std::vector<int>& cluster) {
                                              return cluster.size() < config.cluster_points;
                                          }),
                           clusters.end());

            // Sort clusters by size
            std::sort(
                clusters.begin(), clusters.end(), [](const auto& a, const auto& b) { return a.size() < b.size(); });

            // Merge or discard clusters until only 1 remains
            while (clusters.size() > 1) {
                for (auto it = clusters.begin(); it != clusters.end(); it = std::next(it)) {
                    // Get the largest and smallest X values
                    auto range_a = std::minmax_element(it->begin(), it->end(), [&coords](const auto& a, const auto& b) {
                        return coords(a, 0) < coords(b, 0);
                    });

                    for (auto it2 = std::next(it); it != clusters.end();) {
                        // Get the largest and smallest X values
                        auto range_b =
                            std::minmax_element(it->begin(), it->end(), [&coords](const auto& a, const auto& b) {
                                return coords(a, 0) < coords(b, 0);
                            });

                        // Ranges do not overlap
                        // Merge the clusters
                        if (range_a.second < range_b.first) {
                            // Append the second cluster on to the first
                            it->insert(it->end(), it2->begin(), it2->end());
                            // Delete the second cluster
                            it2 = clusters.erase(it2);
                            // Step over first cluster
                            it = std::next(it);
                        }
                        // Second cluster is overlapping first cluster either on the left or the right
                        // Keep the largest cluster, or merge if the clusters are the same size
                        else if (((range_a.first < range_b.first) && (range_b.first < range_a.second))
                                 || ((range_b.first < range_a.first) && (range_a.first < range_b.second))) {
                            if (it->size() > it2->size()) {
                                // Delete the second cluster
                                it2 = clusters.erase(it2);
                                // Step over first cluster
                                it = std::next(it);
                            }
                            else if (it->size() < it2->size()) {
                                // Delete the first cluster
                                it = clusters.erase(it);
                                // Step over second cluster
                                it2 = std::next(it2);
                            }
                            else {
                                // Clusters are the same size
                                // If we have distinct min and max elements for both clusters, merge them
                                // Otherwise, discard the second cluster
                                if ((range_a.first != range_b.first) && (range_a.second != range_b.second)) {
                                    // Append the second cluster on to the first
                                    it->insert(it->end(), it2->begin(), it2->end());
                                    // Delete the second cluster
                                    it2 = clusters.erase(it2);
                                    // Step over first cluster
                                    it = std::next(it);
                                }
                                else {
                                    // Delete the second cluster
                                    it2 = clusters.erase(it2);
                                    // Step over first cluster
                                    it = std::next(it);
                                }
                            }
                        }
                    }
                }
            }

            // Sort remaining cluster from left to right
            std::vector<int>& cluster = clusters.front();
            std::sort(cluster.begin(), cluster.end(), [&](const int& a, const int& b) {
                return coords(a, 0) < coords(b, 0);
            });

            auto msg = std::make_unique<GreenHorizonMsg>();

            // Preserve mesh so that anyone using the GreenHorizon can access the original data
            msg->mesh = mesh;

            // Find the convex hull of the cluster
            msg->cluster_indices = cluster;
            msg->horizon_indices = utility::math::geometry::graham_scan(cluster, coords);

            // Create vision lines for debugging purposes
            if (config.draw_convex_hull) {
                auto lines_msg = std::make_unique<Lines>();
                for (auto it = std::next(msg->horizon_indices.begin()); it != msg->horizon_indices.end();
                     it      = std::next(it)) {
                    lines_msg->lines.emplace_back(mesh.camera_id,
                                                  NUClear::clock::now(),
                                                  coords.row(*std::prev(it)).cast<int>(),
                                                  coords.row(*it).cast<int>(),
                                                  Eigen::Vector4d{0.0, 1.0, 0.0, 1.0});
                }
                if (lines_msg->lines.size() > 0) {
                    emit(std::move(lines_msg));
                }
            }

            emit(std::move(msg));
        });
    }
}  // namespace vision
}  // namespace module
