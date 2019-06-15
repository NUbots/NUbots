#include "GreenHorizonDetector.h"

#include <set>

#include "extension/Configuration.h"

#include "message/vision/GreenHorizon.h"
#include "message/vision/VisualMesh.h"

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

            auto msg             = std::make_unique<GreenHorizonMsg>();
            msg->cluster_indices = cluster;

            // Find the convex hull of the cluster
            msg->horizon_indices = graham_scan(cluster, coords);

            emit(std::move(msg));
        });
    }

    // Finds the convex hull of a set of points using the Graham Scan algorithm
    // https://en.wikipedia.org/wiki/Graham_scan
    std::vector<int> GreenHorizonDetector::graham_scan(const std::vector<int>& indices, const Eigen::MatrixXf& coords) {
        // The convex hull indices
        std::vector<int> hull_indices;

        // Make a local copy of indices so we can mutate it
        std::vector<int> local_indices(indices.begin(), indices.end());

        // Find the bottom left point
        size_t bottom_left = 0;
        for (size_t idx = 1; idx < indices.size(); ++idx) {
            const Eigen::Vector2f& min_p = coords.row(local_indices[bottom_left]);
            const Eigen::Vector2f& p     = coords.row(local_indices[idx]);

            if ((p.y() < min_p.y()) || ((p.y() == min_p.y()) && (p.x() < min_p.x()))) {
                bottom_left = idx;
            }
        }

        // Move bottom left to front of list
        if (bottom_left != 0) {
            std::swap(local_indices[0], local_indices[bottom_left]);
            bottom_left = 0;
        }

        // Calculates the turning direction of 3 points
        // Returns -1 indicating an anti-clockwise turn
        // Returns  0 indicating a colinear set of points (no turn)
        // Returns  1 indicating a clockwise turn
        auto turn_direction = [](const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) {
            // Compute z-coordinate of the cross product of P0->P1 and P0->P2
            float cross_z = (p1.y() - p0.y()) * (p2.x() - p1.x()) - (p1.x() - p0.x()) * (p2.y() - p1.y());

            // Anti-clockwise turn
            if (cross_z < 0.0f) {
                return -1;
            }
            // Clockwise turn
            else if (cross_z > 0.0f) {
                return 1;
            }
            // Colinear
            else {
                return 0;
            }
        };

        // Sort points by increasing angle with respect to the bottom left point (don't include the bottom left point in
        // the sort)
        std::sort(std::next(local_indices.begin()),
                  local_indices.end(),
                  [&bottom_left, &coords, &turn_direction](const int& a, const int& b) {
                      const Eigen::Vector2f& p0 = coords.row(bottom_left);
                      const Eigen::Vector2f& p1 = coords.row(a);
                      const Eigen::Vector2f& p2 = coords.row(b);

                      int direction = turn_direction(p0, p1, p2);

                      // If there is no turn then closest point should be sorted before the furthest point
                      if (direction == 0) {
                          return ((p1 - p0).squaredNorm() < (p2 - p0).squaredNorm());
                      }
                      // Otherwise, sort anti-clockwise turns before clockwise turns
                      else {
                          return (direction < 0);
                      }
                  });

        // Remove all colinear triples
        for (auto it = std::next(local_indices.begin()); it != local_indices.end(); it = std::next(it)) {
            for (auto it2 = std::next(it); it != std::prev(local_indices.end());) {
                const Eigen::Vector2f& p0 = coords.row(bottom_left);
                const Eigen::Vector2f& p1 = coords.row(*it2);
                const Eigen::Vector2f& p2 = coords.row(*std::next(it2));

                if (turn_direction(p0, p1, p2) == 0) {
                    it2 = local_indices.erase(it2);
                }
                else {
                    it2 = std::next(it2);
                }
            }
        }

        // We need a minimum of 3 non-colinear points to calculate the convex hull
        if (local_indices.size() < 3) {
            return hull_indices;
        }

        // Add the initial points to the convex hull
        hull_indices.push_back(local_indices[0]);
        hull_indices.push_back(local_indices[1]);
        hull_indices.push_back(local_indices[2]);

        // Now go through the rest of the points and add them to the convex hull if each triple makes an anti-clockwise
        // turn
        for (auto it = std::next(local_indices.begin(), 3); it != local_indices.end(); it = std::next(it)) {
            // Triple does not make an anti-clockwise turn, replace the last element in the list
            while ((hull_indices.size() > 1)
                   && (turn_direction(coords.row(*std::prev(hull_indices.end(), 2)),
                                      coords.row(*std::prev(hull_indices.end(), 1)),
                                      coords.row(*it))
                       >= 0)) {
                // Remove the offending point from the convex hull
                hull_indices.pop_back();
            }

            // Add the new point to the convex hull
            hull_indices.push_back(*it);
        }

        return hull_indices;
    }
}  // namespace vision
}  // namespace module
