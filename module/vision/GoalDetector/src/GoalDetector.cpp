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

#include "utility/math/coordinates.hpp"
#include "utility/math/geometry/ConvexHull.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/Vision.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

namespace module::vision {

    using extension::Configuration;

    using message::support::FieldDescription;
    using message::vision::Goal;
    using message::vision::Goals;
    using message::vision::GreenHorizon;

    using utility::math::coordinates::cartesianToReciprocalSpherical;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::support::Expression;

    GoalDetector::GoalDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Trigger the same function when either update
        on<Configuration>("GoalDetector.yaml").then([this](const Configuration& cfg) {
            log_level = cfg["log_level"].as<NUClear::LogLevel>();

            config.confidence_threshold       = cfg["confidence_threshold"].as<double>();
            config.cluster_points             = cfg["cluster_points"].as<int>();
            config.disagreement_ratio         = cfg["disagreement_ratio"].as<double>();
            config.goal_projection_covariance = Eigen::Vector3d(cfg["goal_projection_covariance"].as<Expression>());
            config.use_median                 = cfg["use_median"].as<bool>();
            config.max_goal_distance          = cfg["max_goal_distance"].as<double>();
        });

        on<Trigger<GreenHorizon>, With<FieldDescription>, Buffer<2>>().then(
            "Goal Detector",
            [this](const GreenHorizon& horizon, const FieldDescription& field) {
                // Convenience variables
                const auto& cls                                      = horizon.mesh->classifications;
                const auto& neighbours                               = horizon.mesh->neighbourhood;
                const Eigen::Matrix<double, 3, Eigen::Dynamic>& rays = horizon.mesh->rays.cast<double>();
                const double world_offset                            = std::atan2(horizon.Hcw(0, 1), horizon.Hcw(0, 0));
                const int GOAL_INDEX                                 = horizon.class_map.at("goal");

                // Get some indices to partition
                std::vector<int> indices(horizon.mesh->indices.size());
                std::iota(indices.begin(), indices.end(), 0);

                // Partition the indices such that we only have the goal points that dont have goal surrounding them
                auto boundary = utility::vision::visualmesh::partition_points(
                    indices.begin(),
                    indices.end(),
                    neighbours,
                    [&](const int& idx) {
                        return idx == int(indices.size()) || (cls(GOAL_INDEX, idx) >= config.confidence_threshold);
                    });

                // Discard indices that are not on the boundary and are not below the green horizon
                indices.resize(std::distance(indices.begin(), boundary));

                // Cluster all points into ball candidates
                // Points are clustered based on their connectivity to other ball points
                // Clustering is down in two steps
                // 1) We take the set of goal points found above and partition them into potential clusters by
                //    a) Add the first point and its goal neighbours to a cluster
                //    b) Find all other goal points who are neighbours of the points in the cluster
                //    c) Partition all of the indices that are in the cluster
                //    d) Repeat a-c for all points that were not partitioned
                //    e) Delete all partitions smaller than a given threshold
                // 2) Discard all clusters that do not intersect the green horizon
                std::vector<std::vector<int>> clusters;
                utility::vision::visualmesh::cluster_points(indices.begin(),
                                                            indices.end(),
                                                            neighbours,
                                                            config.cluster_points,
                                                            clusters);

                log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));

                auto green_boundary =
                    utility::vision::visualmesh::check_green_horizon_side(clusters, horizon.horizon, rays, true, true);
                clusters.resize(std::distance(clusters.begin(), green_boundary));

                log<NUClear::DEBUG>(fmt::format("Found {} clusters that intersect the green horizon", clusters.size()));

                // Find overlapping clusters and merge them
                for (auto it = clusters.begin(); it != clusters.end(); it = std::next(it)) {
                    // Get the largest and smallest theta values
                    auto range_a = std::minmax_element(it->begin(), it->end(), [&rays](const int& a, const int& b) {
                        return std::atan2(rays(1, a), rays(0, a)) < std::atan2(rays(1, b), rays(0, b));
                    });

                    const double min_a = std::atan2(rays(1, *range_a.first), rays(0, *range_a.first));
                    const double max_a = std::atan2(rays(1, *range_a.second), rays(0, *range_a.second));

                    for (auto it2 = std::next(it); it2 != clusters.end();) {
                        // Get the largest and smallest theta values
                        auto range_b =
                            std::minmax_element(it2->begin(), it2->end(), [&rays](const int& a, const int& b) {
                                return std::atan2(rays(1, a), rays(0, a)) < std::atan2(rays(1, b), rays(0, b));
                            });

                        const double min_b = std::atan2(rays(1, *range_b.first), rays(0, *range_b.first));
                        const double max_b = std::atan2(rays(1, *range_b.second), rays(0, *range_b.second));

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
                        // Split cluster into left-side and right-side
                        // Return true if the left neighbour is NOT a goal point
                        auto right = utility::vision::visualmesh::partition_points(
                            cluster.begin(),
                            cluster.end(),
                            neighbours,
                            [&](const int& idx) {
                                return idx == int(indices.size())
                                       || (cls(GOAL_INDEX, idx) >= config.confidence_threshold);
                            },
                            {0});
                        // Return true if the right neighbour is NOT a goal point
                        auto other = utility::vision::visualmesh::partition_points(
                            right,
                            cluster.end(),
                            neighbours,
                            [&](const int& idx) {
                                return idx == int(indices.size())
                                       || (cls(GOAL_INDEX, idx) >= config.confidence_threshold);
                            },
                            {3});
                        log<NUClear::DEBUG>(
                            fmt::format("Cluster split into {} left points, {} right points, {} top/bottom points",
                                        std::distance(cluster.begin(), right),
                                        std::distance(right, other),
                                        std::distance(other, cluster.end())));

                        if ((std::distance(cluster.begin(), right) != 0) && (std::distance(right, other) != 0)) {
                            Eigen::Vector3d left_side    = Eigen::Vector3d::Zero();
                            Eigen::Vector3d right_side   = Eigen::Vector3d::Zero();
                            Eigen::Vector3d bottom_point = Eigen::Vector3d::Zero();

                            // Find the median of the left side and the right side
                            if (config.use_median) {
                                utility::math::geometry::sort_by_theta(cluster.begin(), right, rays, world_offset);
                                utility::math::geometry::sort_by_theta(right, other, rays, world_offset);
                                left_side =
                                    rays.col(*std::next(cluster.begin(), std::distance(cluster.begin(), right) / 2));
                                right_side = rays.col(*std::next(right, std::distance(right, other) / 2));
                                for (auto it = cluster.begin(); it != cluster.end(); it = std::next(it)) {
                                    const Eigen::Vector3d& p0(rays.col(*it));
                                    if (p0.z() < bottom_point.z()) {
                                        bottom_point = p0;
                                    }
                                }
                            }
                            // Find the average of the left side and the right side
                            else {
                                // Calculate average of left_xy and right_xy and find lowest z point
                                for (auto it = cluster.begin(); it != right; it = std::next(it)) {
                                    const Eigen::Vector3d& p0(rays.col(*it));
                                    left_side += p0;
                                    if (p0.z() < bottom_point.z()) {
                                        bottom_point = p0;
                                    }
                                }
                                for (auto it = right; it != other; it = std::next(it)) {
                                    const Eigen::Vector3d& p0(rays.col(*it));
                                    right_side += p0;
                                    if (p0.z() < bottom_point.z()) {
                                        bottom_point = p0;
                                    }
                                }
                                for (auto it = other; it != cluster.end(); it = std::next(it)) {
                                    const Eigen::Vector3d& p0(rays.col(*it));
                                    if (p0.z() < bottom_point.z()) {
                                        bottom_point = p0;
                                    }
                                }
                                left_side /= std::distance(cluster.begin(), right);
                                right_side /= std::distance(right, other);
                            }

                            // Calculate bottom middle of goal post, z is calculated above
                            bottom_point.x() = (left_side.x() + right_side.x()) * 0.5f;
                            bottom_point.y() = (left_side.y() + right_side.y()) * 0.5f;

                            // https://en.wikipedia.org/wiki/Angular_diameter
                            Eigen::Vector3d middle = ((left_side + right_side) * 0.5f).normalized();
                            double radius          = middle.dot(left_side.normalized());
                            double distance =
                                field.dimensions.goalpost_width * radius * 0.5f / std::sqrt(1.0f - radius * radius);

                            Eigen::Vector3d top_point(bottom_point * distance);
                            top_point.z() += field.dimensions.goal_crossbar_height;
                            g.post.top      = horizon.Hcw.rotation().cast<double>() * top_point.normalized();
                            g.post.bottom   = horizon.Hcw.rotation().cast<double>() * bottom_point.normalized();
                            g.post.distance = distance;

                            // Attach the measurement to the object (distance from camera to bottom center of post)
                            g.measurements.emplace_back();  // Emplaces default constructed object
                            g.measurements.back().type = Goal::MeasurementType::CENTRE;

                            // Spherical Reciprocal Coordinates (1/distance, phi, theta)
                            g.measurements.back().srGCc =
                                cartesianToReciprocalSpherical(Eigen::Vector3d(g.post.bottom * distance));

                            g.measurements.back().covariance = config.goal_projection_covariance.asDiagonal();

                            // Angular positions from the camera
                            g.screen_angular = cartesianToSpherical(g.post.bottom).tail<2>();
                            g.angular_size   = Eigen::Vector2d::Constant(std::acos(radius));

                            /***********************************************
                             *                  THROWOUTS                  *
                             ***********************************************/


                            bool keep = true;  // if false then we will not consider this as a valid goal post

                            // If the goal is too far away, get rid of it!
                            if (distance > config.max_goal_distance) {
                                keep = false;
                                log<NUClear::DEBUG>("**************************************************");
                                log<NUClear::DEBUG>("*                    THROWOUTS                   *");
                                log<NUClear::DEBUG>("**************************************************");

                                log<NUClear::DEBUG>(
                                    fmt::format("Goal discarded: goal distance ({}) > maximum_goal_distance ({})",
                                                distance,
                                                config.max_goal_distance));
                                log<NUClear::DEBUG>("--------------------------------------------------");
                            }

                            if (keep) {
                                // Passed the tests, this post can go onto the next round as a valid goal post!
                                goals->goals.push_back(std::move(g));
                            }
                        }
                    }

                    // Returns true if rGCc0 is to the left of rGCc1, with respect to camera z
                    // The vectors are assumed to have unit norm
                    auto is_left_of = [&](const Eigen::Vector3d& rGCc0, const Eigen::Vector3d& rGCc1) {
                        const Eigen::Vector3d cam_space_z = horizon.Hcw.matrix().block<3, 1>(0, 2).cast<double>();

                        // Direction (determined by RHR) needed to turn to get from one post to the other
                        // Anti-clockwise (negative) turn from rGCc0 to rGCc1 around cam_space_z
                        // ==> rGCc1 is to the left of rGCc0
                        // Clockwise (positive) turn from rGCc0 to rGCc1 around cam_space_z
                        // ==> rGCc0 is to the left of rGCc1
                        return rGCc0.cross(rGCc1).dot(cam_space_z) < 0.0f;
                    };

                    // Calculate the distance between 2 goal posts using the law of cosines
                    // The vectors are assumed to have unit norm
                    auto distance_between = [&](const Eigen::Vector3d& rGCc0,
                                                const double& rGCc0_norm,
                                                const Eigen::Vector3d& rGCc1,
                                                const double& rGCc1_norm) {
                        // Law of consines
                        const double a2 = rGCc0_norm * rGCc0_norm;
                        const double b2 = rGCc1_norm * rGCc1_norm;
                        return std::sqrt(a2 + b2 - 2.0f * rGCc0_norm * rGCc1_norm * rGCc0.dot(rGCc1));
                    };

                    std::map<std::vector<Goal>::iterator, std::pair<std::vector<Goal>::iterator, double>> pairs;
                    const double actual_width = field.dimensions.goal_width;
                    for (auto it1 = goals->goals.begin(); it1 != goals->goals.end(); it1 = std::next(it1)) {
                        for (auto it2 = std::next(it1); it2 != goals->goals.end(); it2 = std::next(it2)) {
                            const Eigen::Vector3d& rGCc0 = it1->post.bottom;
                            const Eigen::Vector3d& rGCc1 = it2->post.bottom;

                            const double width = distance_between(rGCc0, it1->post.distance, rGCc1, it2->post.distance);

                            const double disagreement = std::abs(width - actual_width) / std::max(width, actual_width);

                            // Check the width between the posts
                            // If they are close enough then assign left and right sides

                            log<NUClear::DEBUG>(
                                fmt::format("Camera {}: Goal post 0 distance = {}", horizon.id, it1->post.distance));
                            log<NUClear::DEBUG>(
                                fmt::format("Camera {}: Goal post 1 distance = {}", horizon.id, it2->post.distance));
                            log<NUClear::DEBUG>(
                                fmt::format("Camera {}: Goal width = {} ({})", horizon.id, width, disagreement));

                            if (disagreement < config.disagreement_ratio) {
                                auto it = pairs.find(it1);
                                if (it != pairs.end()) {
                                    if (disagreement < it->second.second) {
                                        pairs[it1] = std::make_pair(it2, disagreement);
                                    }
                                }
                                else {
                                    pairs[it1] = std::make_pair(it2, disagreement);
                                }
                            }
                        }
                    }

                    // We now have all the valid goal post pairs, determine left and right
                    for (const auto& pair : pairs) {
                        // Divide by distance to get back to unit vectors
                        const Eigen::Vector3d& rGCc0 = pair.first->post.bottom;
                        const Eigen::Vector3d& rGCc1 = pair.second.first->post.bottom;

                        // Determine which post is the left one
                        if (is_left_of(rGCc0, rGCc1)) {
                            pair.first->side        = Goal::Side::LEFT;
                            pair.second.first->side = Goal::Side::RIGHT;
                        }
                        else {
                            pair.first->side        = Goal::Side::RIGHT;
                            pair.second.first->side = Goal::Side::LEFT;
                        }
                    }

                    log<NUClear::DEBUG>(fmt::format("Found {} goal posts", goals->goals.size()));

                    emit(std::move(goals));
                }
            });
    }
}  // namespace module::vision
