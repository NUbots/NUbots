/*
 * MIT License
 *
 * Copyright (c) 2014 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "GoalDetector.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <numeric>

#include "extension/Configuration.hpp"

#include "message/support/FieldDescription.hpp"
#include "message/vision/Goal.hpp"
#include "message/vision/GreenHorizon.hpp"

#include "utility/support/yaml_expression.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

namespace module::vision {

    using extension::Configuration;

    using message::support::FieldDescription;
    using message::vision::Goal;
    using message::vision::Goals;
    using message::vision::GreenHorizon;

    using utility::support::Expression;
    using utility::vision::visualmesh::cluster_points;
    using utility::vision::visualmesh::partition_points;

    GoalDetector::GoalDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("GoalDetector.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.confidence_threshold = config["confidence_threshold"].as<double>();
            cfg.cluster_points       = config["cluster_points"].as<int>();
            cfg.disagreement_ratio   = config["disagreement_ratio"].as<double>();
        });

        on<Trigger<GreenHorizon>, With<FieldDescription>, Buffer<2>>().then(
            "Goal Detector",
            [this](const GreenHorizon& horizon, const FieldDescription& field) {
                // Convenience variables
                const auto& cls                                      = horizon.mesh->classifications;
                const auto& neighbours                               = horizon.mesh->neighbourhood;
                const Eigen::Matrix<double, 3, Eigen::Dynamic>& uPCw = horizon.mesh->uPCw.cast<double>();
                const Eigen::Matrix<double, 3, Eigen::Dynamic>& rPWw = horizon.mesh->rPWw.cast<double>();
                const int GOAL_INDEX                                 = horizon.class_map.at("goal");

                // Get some indices to partition
                std::vector<int> indices(horizon.mesh->indices.size());
                std::iota(indices.begin(), indices.end(), 0);

                // Partition the indices such that we only have the goal points that dont have goal surrounding them
                auto boundary = partition_points(indices.begin(), indices.end(), neighbours, [&](const int& idx) {
                    return cls(GOAL_INDEX, idx) >= cfg.confidence_threshold;
                });
                indices.resize(std::distance(indices.begin(), boundary));

                // Cluster all points into goal post candidates
                std::vector<std::vector<int>> clusters;
                cluster_points(indices.begin(), indices.end(), neighbours, cfg.cluster_points, clusters);

                log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));

                auto green_boundary = utility::vision::visualmesh::check_green_horizon_side(clusters,
                                                                                            horizon.horizon,
                                                                                            rPWw,
                                                                                            false,
                                                                                            false,
                                                                                            true);
                clusters.resize(std::distance(clusters.begin(), green_boundary));

                log<NUClear::DEBUG>(fmt::format("Found {} clusters that intersect the green horizon", clusters.size()));

                // Find overlapping clusters and merge them
                for (auto it = clusters.begin(); it != clusters.end(); it = std::next(it)) {
                    // Get the largest and smallest theta values
                    auto range_a = std::minmax_element(it->begin(), it->end(), [&uPCw](const int& a, const int& b) {
                        return std::atan2(uPCw(1, a), uPCw(0, a)) < std::atan2(uPCw(1, b), uPCw(0, b));
                    });

                    const double min_a = std::atan2(uPCw(1, *range_a.first), uPCw(0, *range_a.first));
                    const double max_a = std::atan2(uPCw(1, *range_a.second), uPCw(0, *range_a.second));

                    for (auto it2 = std::next(it); it2 != clusters.end();) {
                        // Get the largest and smallest theta values
                        auto range_b =
                            std::minmax_element(it2->begin(), it2->end(), [&uPCw](const int& a, const int& b) {
                                return std::atan2(uPCw(1, a), uPCw(0, a)) < std::atan2(uPCw(1, b), uPCw(0, b));
                            });

                        const double min_b = std::atan2(uPCw(1, *range_b.first), uPCw(0, *range_b.first));
                        const double max_b = std::atan2(uPCw(1, *range_b.second), uPCw(0, *range_b.second));

                        // The clusters are overlapping, merge them
                        if (((min_a <= min_b) && (min_b <= max_a)) || ((min_b <= min_a) && (min_a <= max_b))) {
                            // Append the second cluster on to the first
                            it->insert(it->end(), it2->begin(), it2->end());
                            // Delete the second cluster
                            it2 = clusters.erase(it2);
                        }
                        else {
                            it2 = std::next(it2);
                        }
                    }
                }

                log<NUClear::DEBUG>(fmt::format("{} clusters remaining after merging overlaps", clusters.size()));

                if (!clusters.empty()) {
                    auto goals = std::make_unique<Goals>();
                    goals->goals.reserve(clusters.size());

                    goals->id        = horizon.id;
                    goals->timestamp = horizon.timestamp;
                    goals->Hcw       = horizon.Hcw;

                    for (auto& cluster : clusters) {
                        Goal g;

                        // Grab the goal point closest in the cluster and assume it is the bottom of the post
                        auto closest_uGCw_idx =
                            std::min_element(cluster.begin(), cluster.end(), [&](const int& a, const int& b) {
                                return uPCw.col(a).z() < uPCw.col(b).z();
                            });
                        Eigen::Vector3d closest_uGCw = Eigen::Vector3d(uPCw.col(*closest_uGCw_idx));

                        // Project goal point onto the ground plane
                        Eigen::Vector3d rCWw = horizon.Hcw.inverse().translation();
                        Eigen::Vector3d rGWw = closest_uGCw * std::abs(rCWw.z() / closest_uGCw.z()) + rCWw;

                        // Calculate the distance from the camera to the bottom of the post
                        Eigen::Vector3d rGCw = rGWw - rCWw;
                        g.post.distance      = rGCw.norm();

                        // Create unit vectors pointing to the top and bottom of the post in camera {c} space
                        g.post.bottom = (horizon.Hcw * rGWw).normalized();
                        g.post.top =
                            (horizon.Hcw * (rGWw + Eigen::Vector3d(0, 0, field.goalpost_top_height))).normalized();
                        g.measurements.emplace_back();  // Emplaces default constructed object
                        g.measurements.back().type = Goal::MeasurementType::CENTRE;
                        g.measurements.back().rGCc = horizon.Hcw * rGWw;
                        goals->goals.push_back(std::move(g));
                    }

                    // Returns true if uGCc0 is to the left of uGCc1, with respect to camera z
                    auto is_left_of = [&](const Eigen::Vector3d& uGCc0, const Eigen::Vector3d& uGCc1) {
                        const Eigen::Vector3d cam_space_z = horizon.Hcw.matrix().block<3, 1>(0, 2);
                        return uGCc0.cross(uGCc1).dot(cam_space_z) < 0.0f;
                    };

                    // Calculates the distance between 2 goal posts using the law of cosines
                    auto distance_between = [&](const Eigen::Vector3d& uGCc0,
                                                const double& rGCc0_norm,
                                                const Eigen::Vector3d& uGCc1,
                                                const double& rGCc1_norm) {
                        const double a2 = rGCc0_norm * rGCc0_norm;
                        const double b2 = rGCc1_norm * rGCc1_norm;
                        return std::sqrt(a2 + b2 - 2.0f * rGCc0_norm * rGCc1_norm * uGCc0.dot(uGCc1));
                    };

                    // Find goal post pairs
                    std::map<std::vector<Goal>::iterator, std::pair<std::vector<Goal>::iterator, double>> pairs;
                    const double actual_width = field.dimensions.goal_width;
                    for (auto uGCc0 = goals->goals.begin(); uGCc0 != goals->goals.end(); uGCc0 = std::next(uGCc0)) {
                        for (auto uGCc1 = std::next(uGCc0); uGCc1 != goals->goals.end(); uGCc1 = std::next(uGCc1)) {

                            const double width = distance_between(uGCc0->post.bottom,
                                                                  uGCc0->post.distance,
                                                                  uGCc1->post.bottom,
                                                                  uGCc1->post.distance);

                            const double disagreement = std::abs(width - actual_width) / std::max(width, actual_width);

                            // Check the width between the posts
                            log<NUClear::DEBUG>(
                                fmt::format("Camera {}: Goal post 0 distance = {}", horizon.id, uGCc0->post.distance));
                            log<NUClear::DEBUG>(
                                fmt::format("Camera {}: Goal post 1 distance = {}", horizon.id, uGCc1->post.distance));
                            log<NUClear::DEBUG>(
                                fmt::format("Camera {}: Goal width = {} ({})", horizon.id, width, disagreement));

                            if (disagreement < cfg.disagreement_ratio) {
                                auto it = pairs.find(uGCc0);
                                if (it != pairs.end()) {
                                    if (disagreement < it->second.second) {
                                        pairs[uGCc0] = std::make_pair(uGCc1, disagreement);
                                    }
                                }
                                else {
                                    pairs[uGCc0] = std::make_pair(uGCc1, disagreement);
                                }
                            }
                        }
                    }

                    // We now have all the valid goal post pairs, determine which is left and right
                    for (const auto& pair : pairs) {
                        bool is_left            = is_left_of(pair.first->post.bottom, pair.second.first->post.bottom);
                        pair.first->side        = is_left ? Goal::Side::LEFT : Goal::Side::RIGHT;
                        pair.second.first->side = is_left ? Goal::Side::RIGHT : Goal::Side::LEFT;
                    }

                    log<NUClear::DEBUG>(fmt::format("Found {} goal posts", goals->goals.size()));

                    emit(std::move(goals));
                }
            });
    }
}  // namespace module::vision
