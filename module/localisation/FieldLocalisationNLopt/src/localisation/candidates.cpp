/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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

#include <Eigen/Geometry>
#include <algorithm>
#include <array>

#include "FieldLocalisationNLopt.hpp"

#include "utility/math/angle.hpp"

namespace module::localisation {

    using message::vision::FieldIntersection;
    using message::vision::FieldIntersections;

    namespace {
        /// @brief Whether two candidate poses are close enough to be considered duplicates: within 0.3 m in
        /// position and 0.3 rad in (wrapped) heading.
        bool is_duplicate_pose(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
            constexpr double position_tolerance = 0.3;
            constexpr double angle_tolerance    = 0.3;
            double position_distance            = (a.head<2>() - b.head<2>()).norm();
            double angle_distance = std::abs(utility::math::angle::signedDifference(a.z(), b.z()));
            return position_distance < position_tolerance && angle_distance < angle_tolerance;
        }
    }  // namespace

    Eigen::Vector3d pose_from_point_pair(const Eigen::Vector2d& p1w,
                                         const Eigen::Vector2d& p2w,
                                         const Eigen::Vector2d& q1f,
                                         const Eigen::Vector2d& q2f) {
        Eigen::Vector2d dp = p2w - p1w;
        Eigen::Vector2d dq = q2f - q1f;
        double theta = utility::math::angle::normalise_angle(std::atan2(dq.y(), dq.x()) - std::atan2(dp.y(), dp.x()));

        Eigen::Rotation2Dd R(theta);
        Eigen::Vector2d t = q1f - R * p1w;

        return Eigen::Vector3d(t.x(), t.y(), theta);
    }

    Eigen::Vector3d mirror_pose(const Eigen::Vector3d& s) {
        return Eigen::Vector3d(-s.x(), -s.y(), utility::math::angle::normalise_angle(s.z() + M_PI));
    }

    std::vector<Eigen::Vector3d> FieldLocalisationNLopt::goal_pair_candidates(
        const std::shared_ptr<const Goals>& goals) {
        std::vector<Eigen::Vector3d> candidates;

        if (!goals || goals->goals.size() != 2) {
            return candidates;
        }

        // Compute the two observed goal post positions in world space (mirrors the goal-post cost term in
        // evaluate_cost).
        auto Hwc    = Eigen::Isometry3d(goals->Hcw).inverse();
        auto rGWw_1 = Hwc * (goals->goals[0].post.bottom * goals->goals[0].post.distance);
        auto rGWw_2 = Hwc * (goals->goals[1].post.bottom * goals->goals[1].post.distance);

        double distance_apart = (rGWw_1 - rGWw_2).norm();
        if (std::abs(distance_apart - expected_goal_post_distance) >= cfg.goal_post_error_tolerance) {
            return candidates;
        }

        std::array<GoalPost, 2> ends{own_goal_posts, opp_goal_posts};
        for (const auto& end : ends) {
            // Both assignment orders: post 1 -> left/post 2 -> right, and the swap.
            candidates.push_back(
                pose_from_point_pair(rGWw_1.head<2>(), rGWw_2.head<2>(), end.left.head<2>(), end.right.head<2>()));
            candidates.push_back(
                pose_from_point_pair(rGWw_2.head<2>(), rGWw_1.head<2>(), end.left.head<2>(), end.right.head<2>()));
        }

        return candidates;
    }

    std::vector<Eigen::Vector3d> FieldLocalisationNLopt::intersection_pair_candidates(
        const std::shared_ptr<const FieldIntersections>& field_intersections) {
        std::vector<Eigen::Vector3d> candidates;

        if (!field_intersections || field_intersections->intersections.size() < 2) {
            return candidates;
        }

        const auto& intersections = field_intersections->intersections;

        // (candidate pose, generating pair separation) - separation is used both as a matching gate and to
        // prefer widely-separated (better angularly-conditioned) pairs when capping the candidate count.
        std::vector<std::pair<Eigen::Vector3d, double>> candidates_with_separation;

        for (size_t i = 0; i < intersections.size(); ++i) {
            for (size_t j = i + 1; j < intersections.size(); ++j) {
                const auto& obs_i = intersections[i];
                const auto& obs_j = intersections[j];
                double d          = (obs_i.rIWw - obs_j.rIWw).head<2>().norm();

                // Match against every ordered pair of model landmarks with corresponding types. Ordered pairs
                // naturally cover both assignment orders (and mirror twins arise automatically because the
                // model landmark set is symmetric about the field centre).
                for (const auto& landmark_i : landmarks) {
                    if (landmark_i.type != obs_i.type) {
                        continue;
                    }
                    for (const auto& landmark_j : landmarks) {
                        if (landmark_j.type != obs_j.type || &landmark_i == &landmark_j) {
                            continue;
                        }
                        double sep = (landmark_i.rLFf - landmark_j.rLFf).head<2>().norm();
                        if (std::abs(sep - d) < cfg.reset_pair_separation_tolerance) {
                            candidates_with_separation.emplace_back(
                                pose_from_point_pair(obs_i.rIWw.head<2>(),
                                                     obs_j.rIWw.head<2>(),
                                                     landmark_i.rLFf.head<2>(),
                                                     landmark_j.rLFf.head<2>()),
                                d);
                        }
                    }
                }
            }
        }

        // Prefer widely-separated pairs (better angular conditioning): sort descending by separation before
        // deduplicating, so the retained representative of each duplicate cluster is the best-conditioned one.
        std::sort(candidates_with_separation.begin(),
                  candidates_with_separation.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });

        for (const auto& [candidate, separation] : candidates_with_separation) {
            bool duplicate = std::any_of(candidates.begin(), candidates.end(), [&](const Eigen::Vector3d& existing) {
                return is_duplicate_pose(candidate, existing);
            });
            if (!duplicate) {
                candidates.push_back(candidate);
            }
            if (static_cast<int>(candidates.size()) >= cfg.reset_max_candidates) {
                break;
            }
        }

        return candidates;
    }

}  // namespace module::localisation
