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
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "walk_path_control.hpp"

using Catch::Approx;
using module::planning::walk_path::apply_dead_zone;
using module::planning::walk_path::constrain_velocity;
using module::planning::walk_path::walk_to_velocity;
using module::planning::walk_path::WalkToParams;

namespace {

    WalkToParams test_params() {
        WalkToParams p{};
        p.max_velocity          = Eigen::Vector3d(1.0, 0.5, 1.0);
        p.max_backward_velocity = 0.15;
        p.k_translation         = 1.5;
        p.k_theta               = 2.0;
        p.max_align_radius      = 1.5;
        p.min_align_radius      = 0.4;
        p.max_angle_error       = 45.0 * M_PI / 180.0;
        p.min_angle_error       = 20.0 * M_PI / 180.0;
        return p;
    }

}  // namespace

TEST_CASE("Far and aligned target drives straight forward", "[walk_to_velocity]") {
    const auto p      = test_params();
    const auto result = walk_to_velocity(Eigen::Vector2d(3.0, 0.0), 0.0, p);

    // k_translation * d = 4.5, capped at max forward velocity in the far regime
    CHECK(result.velocity.x() == Approx(p.max_velocity.x()));
    CHECK(result.velocity.y() == Approx(0.0));
    CHECK(result.velocity.z() == Approx(0.0));
    CHECK(result.desired_heading == Approx(0.0));
}

TEST_CASE("Far target beyond max angle error rotates on the spot", "[walk_to_velocity]") {
    const auto p = test_params();
    // Target at bearing 90 degrees, well beyond max_angle_error
    const auto result = walk_to_velocity(Eigen::Vector2d(0.0, 3.0), 0.0, p);

    CHECK(result.velocity.x() == Approx(0.0));
    CHECK(result.velocity.y() == Approx(0.0));
    // k_theta * (pi/2) = pi, clamped to the angular limit
    CHECK(result.velocity.z() == Approx(p.max_velocity.z()));
}

TEST_CASE("Far target with partial misalignment slows forward speed", "[walk_to_velocity]") {
    const auto p = test_params();
    // Bearing of 32.5 degrees is halfway between min (20) and max (45) angle error
    const double bearing = 32.5 * M_PI / 180.0;
    const Eigen::Vector2d target(3.0 * std::cos(bearing), 3.0 * std::sin(bearing));
    const auto result = walk_to_velocity(target, 0.0, p);

    CHECK(result.velocity.x() == Approx(p.max_velocity.x() * 0.5).margin(1e-6));
    CHECK(result.velocity.y() == Approx(0.0));
    CHECK(result.velocity.z() == Approx(std::clamp(p.k_theta * bearing, -p.max_velocity.z(), p.max_velocity.z())));
}

TEST_CASE("Near target approaches omnidirectionally with proportional speed", "[walk_to_velocity]") {
    const auto p = test_params();
    const Eigen::Vector2d target(0.3, 0.4);  // d = 0.5, inside max_align_radius
    const double final_heading = 0.5;
    const auto result          = walk_to_velocity(target, final_heading, p);

    // Translational velocity is k_translation * d along the target direction
    const Eigen::Vector2d expected = p.k_translation * 0.5 * target.normalized();
    CHECK(result.velocity.x() == Approx(expected.x()));
    CHECK(result.velocity.y() == Approx(expected.y()));

    // d = 0.5 is between min (0.4) and max (1.5) align radius so the heading is a blend of the
    // angle to the target and the final heading
    const double angle_to_target = std::atan2(target.y(), target.x());
    CHECK(((result.desired_heading > std::min(angle_to_target, final_heading)
            && result.desired_heading < std::max(angle_to_target, final_heading))));
}

TEST_CASE("Near target heading blend takes the shortest arc across the +-pi wrap", "[walk_to_velocity]") {
    const auto p = test_params();
    // Angle to target ~ +179 degrees and final heading -179 degrees are only 2 degrees apart across
    // the wrap, so the blended heading must stay near +-pi rather than pass through zero
    const double bearing = 179.0 * M_PI / 180.0;
    const Eigen::Vector2d target(0.5 * std::cos(bearing), 0.5 * std::sin(bearing));
    const auto result = walk_to_velocity(target, -179.0 * M_PI / 180.0, p);

    CHECK(std::abs(result.desired_heading) > 3.1);
    CHECK(result.velocity.z() == Approx(p.max_velocity.z() * (result.desired_heading > 0 ? 1.0 : -1.0)));
}

TEST_CASE("Inside min align radius the final heading is used exactly", "[walk_to_velocity]") {
    const auto p               = test_params();
    const double final_heading = -0.7;
    const auto result          = walk_to_velocity(Eigen::Vector2d(0.2, 0.1), final_heading, p);

    CHECK(result.desired_heading == Approx(final_heading));
    // k_theta * -0.7 = -1.4, clamped to the angular limit
    CHECK(result.velocity.z() == Approx(std::max(p.k_theta * final_heading, -p.max_velocity.z())));
}

TEST_CASE("Velocity magnitude is continuous across the regime boundary", "[walk_to_velocity]") {
    const auto p = test_params();
    // Aligned target just either side of max_align_radius
    const auto far  = walk_to_velocity(Eigen::Vector2d(p.max_align_radius + 0.001, 0.0), 0.0, p);
    const auto near = walk_to_velocity(Eigen::Vector2d(p.max_align_radius - 0.001, 0.0), 0.0, p);

    CHECK((far.velocity - near.velocity).norm() < 0.05);
}

TEST_CASE("Velocity goes to zero at the target", "[walk_to_velocity]") {
    const auto p      = test_params();
    const auto result = walk_to_velocity(Eigen::Vector2d(0.0, 0.0), 0.0, p);

    CHECK(result.velocity.norm() == Approx(0.0));
}

TEST_CASE("Dead zone snaps, bumps and clamps per axis", "[apply_dead_zone]") {
    const Eigen::Vector3d min_v(0.3, 0.3, 0.25);
    const Eigen::Vector3d zero_tol(0.05, 0.05, 0.05);
    const Eigen::Vector3d max_v(1.0, 0.5, 1.0);

    auto dz = [&](double x) { return apply_dead_zone(Eigen::Vector3d(x, 0.0, 0.0), min_v, zero_tol, max_v).x(); };

    CHECK(dz(0.0) == Approx(0.0));
    CHECK(dz(0.03) == Approx(0.0));    // below zero tolerance -> stop
    CHECK(dz(0.1) == Approx(0.3));     // in the dead zone -> bumped to minimum
    CHECK(dz(-0.1) == Approx(-0.3));   // sign preserved
    CHECK(dz(0.5) == Approx(0.5));     // normal command passes through
    CHECK(dz(1.5) == Approx(1.0));     // clamped to maximum

    // Each axis uses its own thresholds
    const Eigen::Vector3d out = apply_dead_zone(Eigen::Vector3d(0.1, 0.1, 0.1), min_v, zero_tol, max_v);
    CHECK(out.z() == Approx(0.25));
}

TEST_CASE("Constrain velocity preserves translation direction when scaling", "[constrain_velocity]") {
    const Eigen::Vector3d max_v(1.0, 0.5, 1.0);

    // y exceeds its limit by 2x, so both axes scale by 0.5
    const Eigen::Vector3d out = constrain_velocity(Eigen::Vector3d(0.8, 1.0, 0.0), max_v, 0.15);
    CHECK(out.x() == Approx(0.4));
    CHECK(out.y() == Approx(0.5));

    // Under the limits nothing changes
    const Eigen::Vector3d unchanged = constrain_velocity(Eigen::Vector3d(0.5, 0.2, 0.3), max_v, 0.15);
    CHECK(unchanged.x() == Approx(0.5));
    CHECK(unchanged.y() == Approx(0.2));
    CHECK(unchanged.z() == Approx(0.3));
}

TEST_CASE("Constrain velocity clamps backward speed asymmetrically", "[constrain_velocity]") {
    const Eigen::Vector3d max_v(1.0, 0.5, 1.0);
    const Eigen::Vector3d out = constrain_velocity(Eigen::Vector3d(-1.0, 0.0, 0.0), max_v, 0.15);

    CHECK(out.x() == Approx(-0.15));
}

TEST_CASE("Constrain velocity clamps angular velocity", "[constrain_velocity]") {
    const Eigen::Vector3d max_v(1.0, 0.5, 1.0);

    CHECK(constrain_velocity(Eigen::Vector3d(0.0, 0.0, 2.0), max_v, 0.15).z() == Approx(1.0));
    CHECK(constrain_velocity(Eigen::Vector3d(0.0, 0.0, -2.0), max_v, 0.15).z() == Approx(-1.0));
}

TEST_CASE("Pivot orbit kinematics", "[pivot]") {
    // Orbiting a point at radius r directly ahead with angular velocity w requires vy = -w * r
    const double pivot_radius           = 0.4;
    const double pivot_angular_velocity = 0.8;

    for (const int sign : {-1, 1}) {
        const double w  = sign * pivot_angular_velocity;
        const double vy = -w * pivot_radius;
        CHECK(vy == Approx(-sign * 0.32));
    }
}
