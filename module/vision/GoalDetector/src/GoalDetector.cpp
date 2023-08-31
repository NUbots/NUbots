/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
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
#include "utility/nusight/NUhelpers.hpp"
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
    using utility::nusight::graph;
    using utility::support::Expression;

    GoalDetector::GoalDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Trigger the same function when either update
        on<Configuration>("GoalDetector.yaml").then([this](const Configuration& cfg) {
            log_level = cfg["log_level"].as<NUClear::LogLevel>();

            config.confidence_threshold       = cfg["confidence_threshold"].as<float>();
            config.cluster_points             = cfg["cluster_points"].as<int>();
            config.disagreement_ratio         = cfg["disagreement_ratio"].as<float>();
            config.goal_projection_covariance = Eigen::Vector3f(cfg["goal_projection_covariance"].as<Expression>());
            config.use_median                 = cfg["use_median"].as<bool>();
            config.max_goal_distance          = cfg["max_goal_distance"].as<float>();
            config.max_benchmark_error        = cfg["max_benchmark_error"].as<float>();
        });

        on<Trigger<GreenHorizon>, With<FieldDescription>, Buffer<2>>().then(
            "Goal Detector",
            [this](const GreenHorizon& horizon, const FieldDescription& field) {
                // Convenience variables
                const auto& cls                                     = horizon.mesh->classifications;
                const auto& neighbours                              = horizon.mesh->neighbourhood;
                const Eigen::Matrix<float, 3, Eigen::Dynamic>& rays = horizon.mesh->rays;
                const float world_offset                            = std::atan2(horizon.Hcw(0, 1), horizon.Hcw(0, 0));
                const int GOAL_INDEX                                = horizon.class_map.at("goal");

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

                auto green_boundary = utility::vision::visualmesh::check_green_horizon_side(clusters.begin(),
                                                                                            clusters.end(),
                                                                                            horizon.horizon.begin(),
                                                                                            horizon.horizon.end(),
                                                                                            rays,
                                                                                            true,
                                                                                            true);
                clusters.resize(std::distance(clusters.begin(), green_boundary));

                log<NUClear::DEBUG>(fmt::format("Found {} clusters that intersect the green horizon", clusters.size()));

                // Find overlapping clusters and merge them
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
                            Eigen::Vector3f left_side    = Eigen::Vector3f::Zero();
                            Eigen::Vector3f right_side   = Eigen::Vector3f::Zero();
                            Eigen::Vector3f bottom_point = Eigen::Vector3f::Zero();

                            // Find the median of the left side and the right side
                            if (config.use_median) {
                                utility::math::geometry::sort_by_theta(cluster.begin(), right, rays, world_offset);
                                utility::math::geometry::sort_by_theta(right, other, rays, world_offset);
                                left_side =
                                    rays.col(*std::next(cluster.begin(), std::distance(cluster.begin(), right) / 2));
                                right_side = rays.col(*std::next(right, std::distance(right, other) / 2));
                                for (auto it = cluster.begin(); it != cluster.end(); it = std::next(it)) {
                                    const Eigen::Vector3f& p0(rays.col(*it));
                                    if (p0.z() < bottom_point.z()) {
                                        bottom_point = p0;
                                    }
                                }
                            }
                            // Find the average of the left side and the right side
                            else {
                                // Calculate average of left_xy and right_xy and find lowest z point
                                for (auto it = cluster.begin(); it != right; it = std::next(it)) {
                                    const Eigen::Vector3f& p0(rays.col(*it));
                                    left_side += p0;
                                    if (p0.z() < bottom_point.z()) {
                                        bottom_point = p0;
                                    }
                                }
                                for (auto it = right; it != other; it = std::next(it)) {
                                    const Eigen::Vector3f& p0(rays.col(*it));
                                    right_side += p0;
                                    if (p0.z() < bottom_point.z()) {
                                        bottom_point = p0;
                                    }
                                }
                                for (auto it = other; it != cluster.end(); it = std::next(it)) {
                                    const Eigen::Vector3f& p0(rays.col(*it));
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
                            Eigen::Vector3f middle = ((left_side + right_side) * 0.5f).normalized();
                            float radius           = middle.dot(left_side.normalized());
                            float distance =
                                field.dimensions.goalpost_width * radius * 0.5f / std::sqrt(1.0f - radius * radius);

                            Eigen::Vector3f top_point(bottom_point * distance);
                            top_point.z() += field.dimensions.goal_crossbar_height;
                            g.post.top      = horizon.Hcw.rotation().cast<float>() * top_point.normalized();
                            g.post.bottom   = horizon.Hcw.rotation().cast<float>() * bottom_point.normalized();
                            g.post.distance = distance;

                            // Attach the measurement to the object (distance from camera to bottom center of post)
                            g.measurements.emplace_back();  // Emplaces default constructed object
                            g.measurements.back().type = Goal::MeasurementType::CENTRE;

                            // Spherical Reciprocal Coordinates (1/distance, phi, theta)
                            g.measurements.back().srGCc =
                                cartesianToReciprocalSpherical(Eigen::Vector3f(g.post.bottom * distance));

                            g.measurements.back().covariance = config.goal_projection_covariance.asDiagonal();

                            // Angular positions from the camera
                            g.screen_angular = cartesianToSpherical(g.post.bottom).tail<2>();
                            g.angular_size   = Eigen::Vector2f::Constant(std::acos(radius));

                            /***********************************************
                             *                  THROWOUTS                  *
                             ***********************************************/


                            bool keep = true;  // if false then we will not consider this as a good_post goal post

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
                                // Passed the tests, this post can go onto the next round as a good_post goal post!
                                goals->goals.push_back(std::move(g));
                            }
                        }
                    }

                    // Returns true if rGCc0 is to the left of rGCc1, with respect to camera z
                    // The vectors are assumed to have unit norm
                    auto is_left_of = [&](const Eigen::Vector3f& rGCc0, const Eigen::Vector3f& rGCc1) {
                        const Eigen::Vector3f cam_space_z = horizon.Hcw.matrix().block<3, 1>(0, 2).cast<float>();

                        // Direction (determined by RHR) needed to turn to get from one post to the other
                        // Anti-clockwise (negative) turn from rGCc0 to rGCc1 around cam_space_z
                        // ==> rGCc1 is to the left of rGCc0
                        // Clockwise (positive) turn from rGCc0 to rGCc1 around cam_space_z
                        // ==> rGCc0 is to the left of rGCc1
                        return rGCc0.cross(rGCc1).dot(cam_space_z) < 0.0f;
                    };

                    // Calculate the distance between 2 goal posts using the law of cosines
                    // The vectors are assumed to have unit norm
                    auto distance_between = [&](const Eigen::Vector3f& rGCc0,
                                                const float& rGCc0_norm,
                                                const Eigen::Vector3f& rGCc1,
                                                const float& rGCc1_norm) {
                        // Law of consines
                        const float a2 = rGCc0_norm * rGCc0_norm;
                        const float b2 = rGCc1_norm * rGCc1_norm;
                        return std::sqrt(a2 + b2 - 2.0f * rGCc0_norm * rGCc1_norm * rGCc0.dot(rGCc1));
                    };

                    std::map<std::vector<Goal>::iterator, std::pair<std::vector<Goal>::iterator, float>> pairs;
                    const float actual_width = field.dimensions.goal_width;
                    for (auto it1 = goals->goals.begin(); it1 != goals->goals.end(); it1 = std::next(it1)) {
                        for (auto it2 = std::next(it1); it2 != goals->goals.end(); it2 = std::next(it2)) {
                            const Eigen::Vector3f& rGCc0 = it1->post.bottom;
                            const Eigen::Vector3f& rGCc1 = it2->post.bottom;

                            const float width = distance_between(rGCc0, it1->post.distance, rGCc1, it2->post.distance);

                            const float disagreement = std::abs(width - actual_width) / std::max(width, actual_width);

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

                    // We now have all the good_post goal post pairs, determine left and right
                    for (const auto& pair : pairs) {
                        // Divide by distance to get back to unit vectors
                        const Eigen::Vector3f& rGCc0 = pair.first->post.bottom;
                        const Eigen::Vector3f& rGCc1 = pair.second.first->post.bottom;

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

                    if (horizon.vision_ground_truth.exists) {

                        benchmark_goals(field, horizon, goals);
                    }

                    log<NUClear::DEBUG>(fmt::format("Found {} goal posts", goals->goals.size()));

                    emit(std::move(goals));
                }
            });
    }

    void GoalDetector::benchmark_goals(const FieldDescription& field,
                                       const GreenHorizon& horizon,
                                       std::unique_ptr<Goals>& goals) {


        // Calculate error for detected goals using groundtruth
        auto calc_error = [&](const Eigen::Vector3f& post1, const Eigen::Vector3f& post2) {
            return (post1 - post2).norm();
        };

        // Calculate goal post positons using field dimensions
        auto calc_goal_pos = [&](const Eigen::Vector3f& rFWw, const int& x_sign, const int& y_sign) {
            return Eigen::Vector3f(
                rFWw.x() + (field.dimensions.field_length / 2 * x_sign),
                rFWw.y() + (((field.dimensions.goal_width / 2) + (field.dimensions.goalpost_width / 2)) * y_sign),
                rFWw.z());
        };

        const Eigen::Affine3f Hcw(horizon.Hcw.cast<float>());
        const Eigen::Vector3f rFWw = horizon.vision_ground_truth.rFWw;

        // Get both goal posts for both teams
        const Eigen::Vector3f rGWw_own_l = calc_goal_pos(rFWw, -1, -1);
        const Eigen::Vector3f rGWw_own_r = calc_goal_pos(rFWw, -1, +1);
        const Eigen::Vector3f rGWw_opp_l = calc_goal_pos(rFWw, +1, +1);
        const Eigen::Vector3f rGWw_opp_r = calc_goal_pos(rFWw, +1, -1);

        // Transform goal post positions into camera space
        const Eigen::Vector3f rGCc_own_l = (Hcw * rGWw_own_l).normalized();
        const Eigen::Vector3f rGCc_own_r = (Hcw * rGWw_own_r).normalized();
        const Eigen::Vector3f rGCc_opp_l = (Hcw * rGWw_opp_l).normalized();
        const Eigen::Vector3f rGCc_opp_r = (Hcw * rGWw_opp_r).normalized();

        int bad_posts                    = 0;
        float goal_error                 = 0.0;
        Eigen::Vector3f goal_error3f     = Eigen::Vector3f::Zero();
        float goal_error_bad             = 0.0;
        Eigen::Vector3f goal_error3f_bad = Eigen::Vector3f::Zero();

        // Loop through all detected goal posts
        for (auto it = goals->goals.begin(); it != goals->goals.end(); it = std::next(it)) {

            bool good_post        = false;
            float min_error       = 1.0;
            Eigen::Vector3f error = Eigen::Vector3f::Zero();

            const float dist_own_l = calc_error(it->post.bottom, rGCc_own_l);
            const float dist_own_r = calc_error(it->post.bottom, rGCc_own_r);
            const float dist_opp_l = calc_error(it->post.bottom, rGCc_opp_l);
            const float dist_opp_r = calc_error(it->post.bottom, rGCc_opp_r);

            // If a detected post has less error than the max_benchmark_error set in the yaml file,
            // we says it is a good post.
            if (dist_own_l < config.max_benchmark_error || dist_own_r < config.max_benchmark_error
                || dist_opp_l < config.max_benchmark_error || dist_opp_r < config.max_benchmark_error) {
                good_post = true;
            }

            // Record error for the posts closest match
            if (dist_own_l < min_error) {
                min_error = dist_own_l;
                error     = (it->post.bottom - rGCc_own_l).cwiseAbs();
            }
            if (dist_own_r < min_error) {
                min_error = dist_own_r;
                error     = (it->post.bottom - rGCc_own_r).cwiseAbs();
            }
            if (dist_opp_l < min_error) {
                min_error = dist_opp_l;
                error     = (it->post.bottom - rGCc_opp_l).cwiseAbs();
            }
            if (dist_opp_r < min_error) {
                min_error = dist_opp_r;
                error     = (it->post.bottom - rGCc_opp_r).cwiseAbs();
            }

            if (good_post) {
                goal_error += min_error;
                goal_error3f = error;
            }

            goal_error_bad += min_error;
            goal_error3f_bad = error;

            if (!good_post) {
                bad_posts++;
            }
        }

        // Avoid dividing by 0
        if (goals->goals.size() > bad_posts) {
            goal_error   = (goal_error / ((goals->goals.size() - bad_posts)));
            goal_error3f = (goal_error3f / ((goals->goals.size() - bad_posts)));
        }

        goal_error_bad   = (goal_error_bad / (goals->goals.size()));
        goal_error3f_bad = (goal_error3f_bad / (goals->goals.size()));

        emit(graph("Vector Goal Error without Bad Goals", goal_error3f.x(), goal_error3f.y(), goal_error3f.z()));
        emit(graph("Scalar Goal Error without Bad Goals", goal_error));

        emit(graph("Vector Goal Error with Bad Goals",
                   goal_error3f_bad.x(),
                   goal_error3f_bad.y(),
                   goal_error3f_bad.z()));
        emit(graph("Scalar Goal Error with Bad Goals", goal_error_bad));
        emit(graph("Bad Goal Posts", bad_posts));
    }
}  // namespace module::vision
