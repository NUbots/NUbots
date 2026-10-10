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
#include "ModelledWalkMPC.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

using Catch::Approx;
using module::planning::modelled_walk_mpc::Config;
using module::planning::modelled_walk_mpc::HammersteinModel;
using module::planning::modelled_walk_mpc::ModelledWalkMPC;

namespace {

    /// @brief The defaults (the module's configuration's tuning) with the K1 model from the module's configuration
    /// (tests run in the build directory, where it is copied)
    Config k1_config() {
        Config cfg{};
        cfg.model = HammersteinModel::from_yaml(YAML::LoadFile("config/ModelledMPCWalkPath.yaml")["model"]);
        return cfg;
    }

    /// @brief A walk to a pose: the robot starts at the origin facing +x; target and obstacles are in the world
    struct Scenario {
        std::string name;
        Eigen::Vector3d target;
        std::vector<Eigen::Vector2d> obstacles{};
    };

    struct Run {
        double arrival_time          = -1.0;  // -1 if it never arrived
        double min_clearance         = std::numeric_limits<double>::infinity();
        Eigen::Vector3d peak_rate    = Eigen::Vector3d::Zero();
        Eigen::Vector3d peak_command = Eigen::Vector3d::Zero();
        double min_vx                = 0.0;
        double max_estimate_error    = 0.0;
        int failures                 = 0;
    };

    /// @brief Closed loop against the model itself as the robot (unsmoothed, at its own sample time): the planner at
    /// 1/DT, advancing its estimate by DT before each solve. Arrival is within 5 cm and 0.05 rad, staying there for
    /// 0.5 s.
    Run simulate(ModelledWalkMPC& mpc,
                 const HammersteinModel& robot,
                 const Scenario& scenario,
                 double duration = 30.0) {
        Run run{};
        Eigen::Vector3d pose     = Eigen::Vector3d::Zero();
        HammersteinModel::Lags z = HammersteinModel::Lags::Zero();
        Eigen::Vector3d sent     = Eigen::Vector3d::Zero();
        double inside_since      = -1.0;  // when the robot last entered the target, -1 while outside
        const int substeps       = int(std::round(ModelledWalkMPC::DT / robot.sample_time));

        for (int tick = 0; tick * ModelledWalkMPC::DT < duration; ++tick) {
            const double t = tick * ModelledWalkMPC::DT;
            if (tick > 0) {
                mpc.advance(ModelledWalkMPC::DT);
            }
            run.max_estimate_error =
                std::max(run.max_estimate_error,
                         (mpc.delivered_velocity() - HammersteinModel::delivered(z)).cwiseAbs().maxCoeff());

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
            run.peak_rate    = run.peak_rate.cwiseMax((command - sent).cwiseAbs() / ModelledWalkMPC::DT);
            run.peak_command = run.peak_command.cwiseMax(command.cwiseAbs());
            run.min_vx       = std::min(run.min_vx, command.x());
            sent             = command;

            for (int k = 0; k < substeps; ++k) {
                const Eigen::Vector3d v = HammersteinModel::delivered(z);
                const double heading    = pose.z() + v.z() * robot.sample_time / 2;
                pose += robot.sample_time
                        * Eigen::Vector3d(std::cos(heading) * v.x() - std::sin(heading) * v.y(),
                                          std::sin(heading) * v.x() + std::cos(heading) * v.y(),
                                          v.z());
                z = robot.step(z, command);
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

    const std::vector<Scenario> SCENARIOS = {
        {"ahead_2m", {2.0, 0.0, 0.0}},
        {"fine_approach", {0.3, 0.1, 20.0 * M_PI / 180.0}},
        {"lateral_turned", {0.5, 1.5, M_PI / 2}},
        {"behind_reversed", {-3.0, 0.0, M_PI}},
        // Turns through the turn map's fitted notch near -0.47 rad/s, which stalled the solver before kink_smoothing
        {"far_diagonal", {4.0, 3.0, -M_PI / 2}},
        {"turn_in_place", {0.0, 0.0, M_PI / 2}},
        {"obstacle_in_path", {4.0, 0.0, 0.0}, {{2.0, 0.1}}},
        {"two_obstacles", {5.0, 0.0, 0.0}, {{1.8, 0.3}, {3.4, -0.4}}},
        // Moves smaller than the policy's dead zones: the MPC has to step past them and back
        {"sidestep_0.3m", {0.0, 0.3, 0.0}},
        {"ahead_0.15m", {0.15, 0.0, 0.0}},
    };

}  // namespace

TEST_CASE("The first command matches the prototype's", "[ModelledWalkMPC]") {
    // The same problem solved through acados's Python interface (acados_template, from codegen/generate_solver.py's
    // build_ocp) from standing, with the same initial guess: [0.0999999982, 0.0999999633, -0.1332770503], at the
    // iteration cap. Also checks that the configured limits and model survive the solver's reset.
    ModelledWalkMPC mpc{k1_config()};
    const auto solution = mpc.solve(Eigen::Vector3d(2.0, 0.5, 0.3), {});
    REQUIRE(solution.success);
    CHECK(solution.command.x() == Approx(0.0999999982).margin(1e-4));
    CHECK(solution.command.y() == Approx(0.0999999633).margin(1e-4));
    CHECK(solution.command.z() == Approx(-0.1332770503).margin(1e-4));
}

TEST_CASE("Arrives at every goal against the model, within the limits", "[ModelledWalkMPC]") {
    const Config cfg = k1_config();
    ModelledWalkMPC mpc{cfg};
    for (const auto& scenario : SCENARIOS) {
        INFO(scenario.name);
        mpc.reset();
        const Run run = simulate(mpc, cfg.model, scenario);
        CHECK(run.arrival_time >= 0.0);
        CHECK(run.failures == 0);
        CHECK(run.peak_command.x() <= cfg.max_velocity.x() + 1e-9);
        CHECK(run.peak_command.y() <= cfg.max_velocity.y() + 1e-9);
        CHECK(run.peak_command.z() <= cfg.max_velocity.z() + 1e-9);
        CHECK(run.min_vx >= -cfg.max_backward_velocity - 1e-9);
        CHECK(run.peak_rate.x() <= cfg.max_acceleration.x() + 1e-6);
        CHECK(run.peak_rate.y() <= cfg.max_acceleration.y() + 1e-6);
        CHECK(run.peak_rate.z() <= cfg.max_acceleration.z() + 1e-6);
        // The estimate runs the same model on the same commands, so it is the robot's state
        CHECK(run.max_estimate_error < 1e-9);
        if (!scenario.obstacles.empty()) {
            CHECK(run.min_clearance >= cfg.obstacle_radius - 0.05);
        }
    }
}

TEST_CASE("The estimate advances in whole model samples and carries the remainder", "[ModelledWalkMPC]") {
    const Config cfg = k1_config();
    ModelledWalkMPC mpc{cfg};
    mpc.set_previous_command(Eigen::Vector3d(0.6, 0.0, 0.0));

    HammersteinModel::Lags z = HammersteinModel::Lags::Zero();
    for (int k = 0; k < 3; ++k) {
        z = cfg.model.step(z, Eigen::Vector3d(0.6, 0.0, 0.0));
    }
    mpc.advance(0.03);  // one sample, 0.01 s left over
    mpc.advance(0.03);  // two more
    CHECK((mpc.delivered_velocity() - HammersteinModel::delivered(z)).norm() < 1e-12);

    // Held long enough, it settles to the steady state (the slowest path, ω←vy, has a 15 s time constant)
    mpc.advance(1000.0);
    CHECK((mpc.delivered_velocity() - cfg.model.steady_state(Eigen::Vector3d(0.6, 0.0, 0.0))).norm() < 1e-6);

    mpc.reset();
    CHECK(mpc.delivered_velocity().isZero());
    CHECK(mpc.previous_command().isZero());
}

TEST_CASE("A model with another sample time is refused", "[ModelledWalkMPC]") {
    Config cfg = k1_config();
    ModelledWalkMPC mpc{cfg};
    cfg.model.sample_time = 0.01;
    CHECK_THROWS_AS(mpc.configure(cfg), std::runtime_error);
}

TEST_CASE("Non-finite input fails without changing the state, and the next solve recovers", "[ModelledWalkMPC]") {
    ModelledWalkMPC mpc{k1_config()};
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
