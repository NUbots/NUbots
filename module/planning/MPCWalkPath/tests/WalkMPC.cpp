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
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "DynamicInverse.hpp"

using Catch::Approx;
using module::planning::walk_mpc::Config;
using module::planning::walk_mpc::DynamicInverse;
using module::planning::walk_mpc::WalkMPC;
using utility::planning::HammersteinModel;

namespace {

    /// @brief The K1's model in the module's configuration (tests run in the build directory, where it is copied), and
    /// the inverse as the module builds it: out to the identified range, every 0.01, one planner step ahead
    HammersteinModel k1_model() {
        return HammersteinModel::from_yaml(YAML::LoadFile("config/MPCWalkPathModel.yaml"));
    }
    DynamicInverse k1_inverse() {
        return DynamicInverse(k1_model(), Eigen::Vector3d(1.75, 1.75, 1.5), 0.01, WalkMPC::DT);
    }

    /// @brief The defaults (the module's configuration's tuning) with the velocity limits as the module sets them: the configuration's, within what the model settles to
    Config k1_config() {
        const YAML::Node config      = YAML::LoadFile("config/MPCWalkPath.yaml");
        const DynamicInverse inverse = k1_inverse();
        const auto max_velocity      = config["max_velocity"].as<std::vector<double>>();
        const Eigen::Vector3d upper(max_velocity[0], max_velocity[1], max_velocity[2]);
        const Eigen::Vector3d lower(-config["max_backward_velocity"].as<double>(), -upper.y(), -upper.z());
        Config cfg{};
        cfg.lower_velocity = lower.cwiseMax(inverse.lower());
        cfg.upper_velocity = upper.cwiseMin(inverse.upper());
        return cfg;
    }

    /// @brief A walk to a pose: the robot starts at the origin facing +x; target and obstacles are in the world
    struct Scenario {
        std::string name;
        Eigen::Vector3d target;
        std::vector<Eigen::Vector2d> obstacles{};
    };

    struct Run {
        std::optional<double> arrival_time{};
        double min_clearance            = std::numeric_limits<double>::infinity();
        Eigen::Vector3d peak_velocity   = Eigen::Vector3d::Zero();
        Eigen::Vector3d lowest_velocity = Eigen::Vector3d::Zero();
        int failures                    = 0;
    };

    /// @brief Integration steps per planner step (the model's sample time)
    constexpr int SUBSTEPS = 5;

    /// @brief What the robot does with the planned velocity: the velocity it delivers in each integration step
    using Robot = std::function<std::vector<Eigen::Vector3d>(const Eigen::Vector3d&)>;

    /// @brief An ideal robot: it delivers the planned velocity exactly, at once
    std::vector<Eigen::Vector3d> ideal(const Eigen::Vector3d& velocity) {
        return std::vector<Eigen::Vector3d>(SUBSTEPS, velocity);
    }

    /// @brief Closed loop: the planner at 1/DT, the robot integrated at 50 Hz. Arrival is within 5 cm and 0.05 rad,
    /// staying there for 0.5 s.
    Run simulate(WalkMPC& mpc,
                 const Scenario& scenario,
                 const Robot& robot = ideal,
                 double duration    = 30.0) {
        Run run{};
        Eigen::Vector3d pose = Eigen::Vector3d::Zero();
        double inside_since  = -1.0;  // when the robot last entered the target, -1 while outside
        const double dt      = WalkMPC::DT / SUBSTEPS;

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

            const auto solution      = mpc.solve(target, obstacles);
            Eigen::Vector3d velocity = solution.velocity;
            if (!solution.success) {
                ++run.failures;
                velocity.setZero();
                mpc.set_previous_velocity(velocity);
            }
            run.peak_velocity   = run.peak_velocity.cwiseMax(velocity);
            run.lowest_velocity = run.lowest_velocity.cwiseMin(velocity);

            const std::vector<Eigen::Vector3d> deliveries = robot(velocity);
            for (const Eigen::Vector3d& delivered : deliveries) {
                const double cc = std::cos(pose.z());
                const double ss = std::sin(pose.z());
                pose += dt
                        * Eigen::Vector3d(cc * delivered.x() - ss * delivered.y(),
                                          ss * delivered.x() + cc * delivered.y(),
                                          delivered.z());
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
        // Backwards and sideways at once
        {"back_and_left", {-2.0, 2.0, 0.0}},
    };

}  // namespace

TEST_CASE("Arrives at every basic goal on an ideal robot, within the limits", "[WalkMPC]") {
    const Config cfg = k1_config();
    WalkMPC mpc{cfg};
    for (const auto& scenario : BASIC) {
        INFO(scenario.name);
        mpc.reset();
        const Run run = simulate(mpc, scenario);
        CHECK(run.arrival_time.has_value());
        CHECK(run.failures == 0);
        CHECK((run.peak_velocity.array() <= cfg.upper_velocity.array() + 1e-9).all());
        CHECK((run.lowest_velocity.array() >= cfg.lower_velocity.array() - 1e-9).all());
        if (!scenario.obstacles.empty()) {
            CHECK(run.min_clearance >= cfg.obstacle_radius - 0.05);
        }
    }
}

TEST_CASE("Arrives at small and large moves through the model's lag and dead zones", "[WalkMPC]") {
    // The robot is the model, sample by sample: it lags the command and stands still for small commands. The command
    // is the inverse's, from its estimate of the robot, as the module finds it.
    const HammersteinModel model = k1_model();
    DynamicInverse inverse       = k1_inverse();
    HammersteinModel::Lags lags  = HammersteinModel::Lags::Zero();
    const Eigen::Vector3d zero_tolerance(0.02, 0.02, 0.02);
    const Robot modelled = [&](const Eigen::Vector3d& velocity) {
        const Eigen::Vector3d wanted  = (velocity.array().abs() < zero_tolerance.array()).select(0.0, velocity);
        const Eigen::Vector3d command = inverse.inverse(wanted);
        inverse.record(command);
        inverse.advance(WalkMPC::DT);
        std::vector<Eigen::Vector3d> deliveries{};
        for (int k = 0; k < SUBSTEPS; ++k) {
            lags = model.step(lags, command);
            deliveries.push_back(HammersteinModel::delivered(lags));
        }
        return deliveries;
    };
    WalkMPC mpc{k1_config()};
    for (const Scenario& scenario : std::vector<Scenario>{
             {"ahead_2m", {2.0, 0.0, 0.0}},
             {"sidestep_0.3m", {0.0, 0.3, 0.0}},
             {"ahead_0.15m", {0.15, 0.0, 0.0}},
             {"back_0.3m", {-0.3, 0.0, 0.0}},
             {"turn_in_place", {0.0, 0.0, M_PI / 2}},
             {"lateral_turned", {0.5, 1.5, M_PI / 2}},
         }) {
        INFO(scenario.name);
        mpc.reset();
        inverse.reset();
        lags.setZero();
        const Run run = simulate(mpc, scenario, modelled);
        CHECK(run.arrival_time.has_value());
        CHECK(run.failures == 0);
    }
}

TEST_CASE("Non-finite input fails without changing the state, and the next solve recovers", "[WalkMPC]") {
    WalkMPC mpc{k1_config()};
    mpc.solve(Eigen::Vector3d(2.0, 0.0, 0.0), {});
    const Eigen::Vector3d before = mpc.previous_velocity();

    const auto bad = mpc.solve(Eigen::Vector3d(std::nan(""), 0.0, 0.0), {});
    CHECK_FALSE(bad.success);
    CHECK(bad.status == -1);
    CHECK(mpc.previous_velocity() == before);

    const auto bad_obstacle = mpc.solve(Eigen::Vector3d(2.0, 0.0, 0.0), {Eigen::Vector2d(INFINITY, 0.0)});
    CHECK_FALSE(bad_obstacle.success);

    const auto good = mpc.solve(Eigen::Vector3d(2.0, 0.0, 0.0), {});
    CHECK(good.success);
    CHECK(good.velocity.allFinite());
}

TEST_CASE("set_previous_velocity clips to the velocity limits and anchors the rate cost", "[WalkMPC]") {
    const Config cfg = k1_config();
    WalkMPC mpc{cfg};
    mpc.set_previous_velocity(Eigen::Vector3d(5.0, -5.0, 5.0));
    CHECK(mpc.previous_velocity().isApprox(
        Eigen::Vector3d(cfg.upper_velocity.x(), cfg.lower_velocity.y(), cfg.upper_velocity.z())));

    // The rate cost is measured from the previous velocity, so from full speed forward the robot slows towards a
    // target behind it more gently than it starts towards it from standing
    mpc.set_previous_velocity(Eigen::Vector3d(cfg.upper_velocity.x(), 0.0, 0.0));
    const auto from_full_speed = mpc.solve(Eigen::Vector3d(-2.0, 0.0, 0.0), {});
    REQUIRE(from_full_speed.success);
    mpc.reset();
    const auto from_standing = mpc.solve(Eigen::Vector3d(-2.0, 0.0, 0.0), {});
    REQUIRE(from_standing.success);
    CHECK(from_full_speed.velocity.x() > from_standing.velocity.x() + 0.1);
}
