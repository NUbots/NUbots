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

#include <catch2/catch_test_macros.hpp>
#include <Eigen/Geometry>
#include <cmath>

#include "FieldLocalisationNLopt.hpp"

using module::localisation::mirror_pose;
using module::localisation::pose_from_point_pair;

// pose_from_point_pair and mirror_pose are pure free functions (no reactor/PowerPlant required), so they are
// exercised directly here. goal_pair_candidates and intersection_pair_candidates are member functions that
// touch cfg/landmarks/goal posts populated by the Startup handler, so (per the module's "no PowerPlant in
// tests" constraint) they are not unit tested here - they are exercised via the Webots forced-delocalisation
// verification instead (see README / verification notes).

TEST_CASE("pose_from_point_pair recovers the identity transform", "[candidates]") {
    Eigen::Vector2d p1w(0.0, 0.0);
    Eigen::Vector2d p2w(1.0, 0.0);
    Eigen::Vector2d q1f(0.0, 0.0);
    Eigen::Vector2d q2f(1.0, 0.0);

    Eigen::Vector3d pose = pose_from_point_pair(p1w, p2w, q1f, q2f);

    REQUIRE(std::abs(pose.x()) < 1e-9);
    REQUIRE(std::abs(pose.y()) < 1e-9);
    REQUIRE(std::abs(pose.z()) < 1e-9);
}

TEST_CASE("pose_from_point_pair recovers a known rotation and translation", "[candidates]") {
    // Ground truth pose we want pose_from_point_pair to recover: Hfw with translation (dx, dy) and yaw dtheta.
    const double dx     = 1.5;
    const double dy     = -0.7;
    const double dtheta = 0.4;

    Eigen::Rotation2Dd R(dtheta);
    Eigen::Vector2d t(dx, dy);

    // Pick two arbitrary world-space points and transform them through the ground truth pose to get their
    // field-space counterparts: q = R*p + t.
    Eigen::Vector2d p1w(0.3, 0.2);
    Eigen::Vector2d p2w(-0.4, 0.9);
    Eigen::Vector2d q1f = R * p1w + t;
    Eigen::Vector2d q2f = R * p2w + t;

    Eigen::Vector3d recovered = pose_from_point_pair(p1w, p2w, q1f, q2f);

    REQUIRE(std::abs(recovered.x() - dx) < 1e-6);
    REQUIRE(std::abs(recovered.y() - dy) < 1e-6);
    REQUIRE(std::abs(recovered.z() - dtheta) < 1e-6);
}

TEST_CASE("mirror_pose is an involution", "[candidates]") {
    std::vector<Eigen::Vector3d> poses = {Eigen::Vector3d(1.0, 2.0, 0.3),
                                          Eigen::Vector3d(-2.0, 0.5, -1.2),
                                          Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(3.0, -1.0, M_PI - 0.1)};

    for (const auto& pose : poses) {
        Eigen::Vector3d twice_mirrored = mirror_pose(mirror_pose(pose));
        REQUIRE(std::abs(twice_mirrored.x() - pose.x()) < 1e-9);
        REQUIRE(std::abs(twice_mirrored.y() - pose.y()) < 1e-9);
        REQUIRE(std::abs(std::abs(twice_mirrored.z() - pose.z())) < 1e-9);
    }
}

TEST_CASE("mirror_pose maps a known pose across the field centre", "[candidates]") {
    Eigen::Vector3d pose     = Eigen::Vector3d(1.0, 2.0, 0.3);
    Eigen::Vector3d mirrored = mirror_pose(pose);

    REQUIRE(std::abs(mirrored.x() - (-1.0)) < 1e-9);
    REQUIRE(std::abs(mirrored.y() - (-2.0)) < 1e-9);
    // 0.3 + pi wraps to 0.3 - pi once normalised into (-pi, pi].
    REQUIRE(std::abs(mirrored.z() - (0.3 - M_PI)) < 1e-9);
}
