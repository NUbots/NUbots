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
#include "WalkMPC.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

using Catch::Approx;
using module::planning::walk_mpc::Config;
using module::planning::walk_mpc::WalkMPC;

namespace {

    /// @brief A walk to a pose: the robot starts at the origin facing +x; target and obstacles are in the world
    struct Scenario {
        std::string name;
        Eigen::Vector3d target;
        std::vector<Eigen::Vector2d> obstacles{};
    };

    struct Run {
        std::optional<double> arrival_time{};
        double min_clearance         = std::numeric_limits<double>::infinity();
        Eigen::Vector3d peak_rate    = Eigen::Vector3d::Zero();
        Eigen::Vector3d peak_command = Eigen::Vector3d::Zero();
        double min_vx                = 0.0;
        int failures                 = 0;
    };

    /// @brief Closed loop against an ideal robot (it delivers the command exactly): the planner at 1/DT, the robot
    /// integrated at 50 Hz. Arrival is within 5 cm and 0.05 rad, staying there for 0.5 s.
    Run simulate(WalkMPC& mpc, const Scenario& scenario, const double duration = 30.0) {
        Run run{};
        Eigen::Vector3d pose = Eigen::Vector3d::Zero();
        Eigen::Vector3d sent = Eigen::Vector3d::Zero();
        double inside_since  = -1.0;  // when the robot last entered the target, -1 while outside
        const int substeps   = 5;
        const double dt      = WalkMPC::DT / substeps;

        for (int tick = 0; tick * WalkMPC::DT < duration; ++tick) {
            const double t = tick * WalkMPC::DT;
            const double c = std::cos(pose.z());
            const double s = std::sin(pose.z());
            auto to_robot  = [&](const Eigen::Vector2d& p) {
                const Eigen::Vector2d d = p - pose.head<2>();
                return Eigen::Vector2d(c * d.x() + s * d.y(), -s * d.x() + c * d.y());
            };

            std::vector<Eigen::Vector2d> obstacles{};
            for (const auto& o : scenario.obstacles) {
                obstacles.push_back(to_robot(o));
            }
            const Eigen::Vector2d target_xy = to_robot(scenario.target.head<2>());
            const Eigen::Vector3d target(target_xy.x(),
                                         target_xy.y(),
                                         std::remainder(scenario.target.z() - pose.z(), 2 * M_PI));

            const auto solution     = mpc.solve(target, obstacles);
            Eigen::Vector3d command = solution.command;
            if (!solution.success) {
                ++run.failures;
                command.setZero();
                mpc.set_previous_command(command);
            }
            run.peak_rate    = run.peak_rate.cwiseMax((command - sent).cwiseAbs() / WalkMPC::DT);
            run.peak_command = run.peak_command.cwiseMax(command.cwiseAbs());
            run.min_vx       = std::min(run.min_vx, command.x());
            sent             = command;

            for (int k = 0; k < substeps; ++k) {
                const double cc = std::cos(pose.z());
                const double ss = std::sin(pose.z());
                pose += dt
                        * Eigen::Vector3d(cc * command.x() - ss * command.y(),
                                          ss * command.x() + cc * command.y(),
                                          command.z());
                for (const auto& o : scenario.obstacles) {
                    run.min_clearance = std::min(run.min_clearance, (pose.head<2>() - o).norm());
                }
            }

            const bool inside = (pose.head<2>() - scenario.target.head<2>()).norm() < 0.05
                                && std::abs(std::remainder(pose.z() - scenario.target.z(), 2 * M_PI)) < 0.05;
            if (!inside) {
                inside_since = -1.0;
            }
            else if (inside_since < 0.0) {
                inside_since = t;
            }
            else if (t - inside_since >= 0.5) {
                run.arrival_time = inside_since;
                break;
            }
        }
        return run;
    }

    const std::vector<Scenario> BASIC = {
        {"ahead_2m", {2.0, 0.0, 0.0}},
        {"fine_approach", {0.3, 0.1, 20.0 * M_PI / 180.0}},
        {"lateral_turned", {0.5, 1.5, M_PI / 2}},
        {"behind_reversed", {-3.0, 0.0, M_PI}},
        {"far_diagonal", {4.0, 3.0, -M_PI / 2}},
        {"turn_in_place", {0.0, 0.0, M_PI / 2}},
        {"obstacle_in_path", {4.0, 0.0, 0.0}, {{2.0, 0.1}}},
        {"two_obstacles", {5.0, 0.0, 0.0}, {{1.8, 0.3}, {3.4, -0.4}}},
    };

}  // namespace

TEST_CASE("The first command matches the walk-mpc prototype's", "[WalkMPC]") {
    // walk-mpc's AcadosWalkMPC with MPCConfig(max_velocity=(1.0, 0.5, 1.5)), from standing, converged:
    // [0.1, 0.09999999, -0.1622366]. Also checks that the configured limits survive the solver's reset (resetting its
    // numerical values puts the generated placeholder limits back, which turned at 1 rad/s and 1 rad/s²).
    WalkMPC mpc{};
    const auto solution = mpc.solve(Eigen::Vector3d(2.0, 0.5, 0.3), {});
    REQUIRE(solution.success);
    CHECK(solution.status == 0);
    CHECK(solution.command.x() == Approx(0.1).margin(1e-4));
    CHECK(solution.command.y() == Approx(0.1).margin(1e-4));
    CHECK(solution.command.z() == Approx(-0.1622366).margin(1e-4));
}

TEST_CASE("Arrives at every basic goal on an ideal robot, within the limits", "[WalkMPC]") {
    const Config cfg{};
    WalkMPC mpc{cfg};
    for (const auto& scenario : BASIC) {
        INFO(scenario.name);
        mpc.reset();
        const Run run = simulate(mpc, scenario);
        CHECK(run.arrival_time.has_value());
        CHECK(run.failures == 0);
        CHECK(run.peak_command.x() <= cfg.max_velocity.x() + 1e-9);
        CHECK(run.peak_command.y() <= cfg.max_velocity.y() + 1e-9);
        CHECK(run.peak_command.z() <= cfg.max_velocity.z() + 1e-9);
        CHECK(run.min_vx >= -cfg.max_backward_velocity - 1e-9);
        CHECK(run.peak_rate.x() <= cfg.max_acceleration.x() + 1e-6);
        CHECK(run.peak_rate.y() <= cfg.max_acceleration.y() + 1e-6);
        CHECK(run.peak_rate.z() <= cfg.max_acceleration.z() + 1e-6);
        if (!scenario.obstacles.empty()) {
            CHECK(run.min_clearance >= cfg.obstacle_radius - 0.05);
        }
    }
}

TEST_CASE("Uses the full turn rate it is configured with", "[WalkMPC]") {
    // Turning on the spot by pi/2 at 1.5 rad/s and 2 rad/s² takes about 1.5 s; at the placeholder 1 rad/s it took 2.1
    WalkMPC mpc{};
    const Run run = simulate(mpc, {"turn_in_place", {0.0, 0.0, M_PI / 2}});
    REQUIRE(run.arrival_time.has_value());
    CHECK(*run.arrival_time < 1.8);
    CHECK(run.peak_command.z() > 1.2);
}

TEST_CASE("A new configuration's limits apply to the next solve", "[WalkMPC]") {
    Config cfg{};
    WalkMPC mpc{cfg};
    cfg.max_velocity = Eigen::Vector3d(0.3, 0.2, 0.5);
    mpc.configure(cfg);
    const Run run = simulate(mpc, {"ahead_2m", {2.0, 0.0, 0.0}});
    CHECK(run.arrival_time.has_value());
    CHECK(run.peak_command.x() <= 0.3 + 1e-9);
    CHECK(run.peak_command.z() <= 0.5 + 1e-9);
}

TEST_CASE("Non-finite input fails without changing the state, and the next solve recovers", "[WalkMPC]") {
    WalkMPC mpc{};
    mpc.solve(Eigen::Vector3d(2.0, 0.0, 0.0), {});
    const Eigen::Vector3d before = mpc.previous_command();

    const auto bad = mpc.solve(Eigen::Vector3d(std::nan(""), 0.0, 0.0), {});
    CHECK_FALSE(bad.success);
    CHECK(bad.status == -1);
    CHECK(mpc.previous_command() == before);

    const auto bad_obstacle = mpc.solve(Eigen::Vector3d(2.0, 0.0, 0.0), {Eigen::Vector2d(INFINITY, 0.0)});
    CHECK_FALSE(bad_obstacle.success);

    const auto good = mpc.solve(Eigen::Vector3d(2.0, 0.0, 0.0), {});
    CHECK(good.success);
    CHECK(good.command.allFinite());
}

TEST_CASE("set_previous_command clips to the velocity limits and anchors the acceleration limits", "[WalkMPC]") {
    const Config cfg{};
    WalkMPC mpc{cfg};
    mpc.set_previous_command(Eigen::Vector3d(5.0, -5.0, 5.0));
    CHECK(mpc.previous_command().isApprox(
        Eigen::Vector3d(cfg.max_velocity.x(), -cfg.max_velocity.y(), cfg.max_velocity.z())));

    // From full speed forward, a target behind can only be reached by slowing at the acceleration limit
    mpc.set_previous_command(Eigen::Vector3d(cfg.max_velocity.x(), 0.0, 0.0));
    const auto solution = mpc.solve(Eigen::Vector3d(-2.0, 0.0, 0.0), {});
    REQUIRE(solution.success);
    CHECK(solution.command.x() >= cfg.max_velocity.x() - cfg.max_acceleration.x() * WalkMPC::DT - 1e-9);
}
