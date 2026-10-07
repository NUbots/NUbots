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
#include "WalkPathBenchmark.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <fmt/format.h>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/booster/NUSimGroundTruth.hpp"
#include "message/eye/DataPoint.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/math/euler.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::tools {

    using extension::Configuration;

    using message::booster::NUSimBallCommand;
    using message::booster::NUSimRobotGroundTruth;
    using message::eye::DataPoint;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::skill::Walk;
    using message::strategy::FallRecovery;
    using message::strategy::WalkToFieldPosition;

    using utility::math::euler::pos_rpy_to_transform;
    using utility::support::Expression;

    namespace {

        /// @brief Planar pose [x, y, yaw] of an isometry's ground projection
        Eigen::Vector3d planar(const Eigen::Isometry3d& H) {
            return {H.translation().x(), H.translation().y(), std::atan2(H.linear()(1, 0), H.linear()(0, 0))};
        }

        /// @brief Yaw-only isometry at the ground projection of a planar pose
        Eigen::Isometry3d flat(const Eigen::Vector3d& pose) {
            return pos_rpy_to_transform(Eigen::Vector3d(pose.x(), pose.y(), 0.0), Eigen::Vector3d(0.0, 0.0, pose.z()));
        }

        double position_error(const Eigen::Vector3d& pose, const Eigen::Vector3d& target) {
            return (pose.head<2>() - target.head<2>()).norm();
        }

        double heading_error(const Eigen::Vector3d& pose, const Eigen::Vector3d& target) {
            return std::abs(std::remainder(target.z() - pose.z(), 2.0 * M_PI));
        }

    }  // namespace

    WalkPathBenchmark::WalkPathBenchmark(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("WalkPathBenchmark.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.start_delay = config["start_delay"].as<double>();
            cfg.targets.clear();
            for (const auto& target : config["targets"]) {
                cfg.targets.emplace_back(target.as<Expression>());
            }
            cfg.sweep_commands.clear();
            for (const auto& command : config["sweep"]["commands"]) {
                cfg.sweep_commands.emplace_back(command.as<Expression>());
            }
            cfg.sweep_duration   = config["sweep"]["duration"].as<double>();
            cfg.sweep_measure    = config["sweep"]["measure"].as<double>();
            cfg.relative_targets = config["relative_targets"].as<bool>();
            cfg.park_ball.clear();
            for (const auto& v : config["park_ball"]) {
                cfg.park_ball.push_back(v.as<double>());
            }
            cfg.trial_timeout          = config["trial_timeout"].as<double>();
            cfg.reach_position_error   = config["reach_position_error"].as<double>();
            cfg.reach_heading_error    = config["reach_heading_error"].as<Expression>();
            cfg.settle_time            = config["settle_time"].as<double>();
            cfg.rest_speed             = config["rest_speed"].as<double>();
            cfg.rest_turn_rate         = config["rest_turn_rate"].as<double>();
            cfg.fall_height            = config["fall_height"].as<double>();
            cfg.ground_truth_field     = config["ground_truth_field"].as<bool>();
            cfg.fall_recovery_priority = config["fall_recovery_priority"].as<int>();
            cfg.shutdown_when_done     = config["shutdown_when_done"].as<bool>();
            cfg.csv_path               = config["csv_path"].as<std::string>();
        });

        on<Startup>().then([this] {
            startup_time = NUClear::clock::now();
            if (!cfg.csv_path.empty()) {
                csv.open(cfg.csv_path);
                csv << "t,trial,x,y,yaw,z,target_x,target_y,target_yaw,cmd_vx,cmd_vy,cmd_wz\n";
            }
        });

        // The commanded walk after PlanWalkPath's smoothing and dead zone
        on<Trigger<DataPoint>>().then([this](const DataPoint& point) {
            if (point.label == "Walk Command" && point.value.size() == 3) {
                walk_command = Eigen::Vector3d(point.value[0], point.value[1], point.value[2]);
            }
        });

        // Ground-truth localisation: the field frame is NUSim's world, so Hfw = Hfr * Hrw with Hfr from ground truth
        on<Trigger<Sensors>, With<NUSimRobotGroundTruth>>().then(
            [this](const Sensors& sensors, const NUSimRobotGroundTruth& gt) {
                if (!cfg.ground_truth_field) {
                    return;
                }
                auto field       = std::make_unique<Field>();
                field->Hfw       = flat(planar(Eigen::Isometry3d(gt.Hst))) * Eigen::Isometry3d(sensors.Hrw);
                field->localised = true;
                emit(field);
            });

        on<Every<10, Per<std::chrono::seconds>>, With<NUSimRobotGroundTruth>, Sync<WalkPathBenchmark>>().then(
            [this](const NUSimRobotGroundTruth& gt) {
                const auto now              = NUClear::clock::now();
                const Eigen::Isometry3d Hst = Eigen::Isometry3d(gt.Hst);
                const Eigen::Vector3d pose  = planar(Hst);
                const double height         = Hst.translation().z();

                emit<Task>(std::make_unique<FallRecovery>(), cfg.fall_recovery_priority);

                if (state == State::WAITING) {
                    if (std::chrono::duration<double>(now - startup_time).count() < cfg.start_delay) {
                        return;
                    }
                    // Move the ball out of the robot's way
                    if (cfg.park_ball.size() == 2) {
                        auto command      = std::make_unique<NUSimBallCommand>();
                        command->frame    = NUSimBallCommand::Frame::WORLD;
                        command->position = Eigen::Vector3d(cfg.park_ball[0], cfg.park_ball[1], -1.0);
                        emit(command);
                    }
                    if (!cfg.sweep_commands.empty()) {
                        state = State::SWEEP;
                        start_sweep();
                    }
                    else if (!cfg.targets.empty()) {
                        begin_trials(pose);
                    }
                    else {
                        log<ERROR>("No sweep commands or targets configured");
                        state = State::DONE;
                        return;
                    }
                }
                if (state == State::DONE) {
                    return;
                }

                if (state == State::SWEEP) {
                    emit<Task>(std::make_unique<Walk>(cfg.sweep_commands[sweep_index]), 1);

                    const double t = std::chrono::duration<double>(now - sweep.start).count();
                    if (height < cfg.fall_height && !sweep.fallen) {
                        ++sweep.falls;
                    }
                    sweep.fallen = height < cfg.fall_height;

                    // Average the body-frame velocity over the end of the hold, once the gait has settled
                    if (t >= cfg.sweep_duration - cfg.sweep_measure && !sweep.fallen) {
                        const Eigen::Vector2d vTs = Eigen::Vector3d(gt.vTs).head<2>();
                        sweep.velocity += Eigen::Vector3d(std::cos(pose.z()) * vTs.x() + std::sin(pose.z()) * vTs.y(),
                                                          -std::sin(pose.z()) * vTs.x() + std::cos(pose.z()) * vTs.y(),
                                                          Eigen::Vector3d(gt.omegaTs).z());
                        ++sweep.samples;
                    }

                    if (t >= cfg.sweep_duration) {
                        const Eigen::Vector3d& command = cfg.sweep_commands[sweep_index];
                        const Eigen::Vector3d measured = sweep.velocity / std::max(sweep.samples, 1);
                        log<INFO>(
                            fmt::format("SWEEP cmd=({:.2f}, {:.2f}, {:.2f}) meas=({:.3f}, {:.3f}, {:.3f}) falls={}",
                                        command.x(),
                                        command.y(),
                                        command.z(),
                                        measured.x(),
                                        measured.y(),
                                        measured.z(),
                                        sweep.falls));
                        if (++sweep_index < cfg.sweep_commands.size()) {
                            start_sweep();
                        }
                        else if (!cfg.targets.empty()) {
                            begin_trials(pose);
                        }
                        else {
                            state = State::DONE;
                            if (cfg.shutdown_when_done) {
                                powerplant.shutdown();
                            }
                        }
                    }
                    return;
                }

                const Eigen::Vector3d& target = targets[trial_index];
                emit<Task>(std::make_unique<WalkToFieldPosition>(flat(target), true), 1);

                const double t       = std::chrono::duration<double>(now - trial.start).count();
                const double pos_err = position_error(pose, target);
                const double yaw_err = heading_error(pose, target);

                if (csv.is_open()) {
                    csv << fmt::format(
                        "{:.2f},{},{:.4f},{:.4f},{:.4f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n",
                        t,
                        trial_index,
                        pose.x(),
                        pose.y(),
                        pose.z(),
                        height,
                        target.x(),
                        target.y(),
                        target.z(),
                        walk_command.x(),
                        walk_command.y(),
                        walk_command.z());
                }

                // Track the path walked and falls
                trial.path_length += (pose.head<2>() - trial.last_position).norm();
                trial.last_position = pose.head<2>();
                const bool fallen   = height < cfg.fall_height;
                if (fallen && !trial.fallen) {
                    ++trial.falls;
                    log<WARN>("Fell during trial", trial_index);
                }
                trial.fallen = fallen;

                const bool at_target =
                    !fallen && pos_err < cfg.reach_position_error && yaw_err < cfg.reach_heading_error;
                if (at_target && trial.reach_time < 0.0) {
                    trial.reach_time = t;
                }
                if (trial.reach_time >= 0.0) {
                    trial.max_error_after_reach = std::max(trial.max_error_after_reach, pos_err);
                }
                trial.settle_start = at_target ? (trial.settle_start < 0.0 ? t : trial.settle_start) : -1.0;

                // At rest: the walk has stopped (or never started) wherever the robot is
                const bool at_rest = !fallen && Eigen::Vector3d(gt.vTs).head<2>().norm() < cfg.rest_speed
                                     && std::abs(Eigen::Vector3d(gt.omegaTs).z()) < cfg.rest_turn_rate;
                trial.rest_start   = at_rest ? (trial.rest_start < 0.0 ? t : trial.rest_start) : -1.0;

                if (trial.settle_start >= 0.0 && t - trial.settle_start >= cfg.settle_time) {
                    finish_trial(pose, Outcome::OK);
                }
                else if (trial.rest_start >= 0.0 && t - trial.rest_start >= cfg.settle_time) {
                    finish_trial(pose, Outcome::SHORT);
                }
                else if (t > cfg.trial_timeout) {
                    finish_trial(pose, Outcome::TIMEOUT);
                }
            });
    }

    void WalkPathBenchmark::start_sweep() {
        sweep       = Sweep{};
        sweep.start = NUClear::clock::now();
    }

    void WalkPathBenchmark::begin_trials(const Eigen::Vector3d& pose) {
        // Resolve the targets in the field frame
        targets.clear();
        for (const auto& target : cfg.targets) {
            targets.push_back(cfg.relative_targets ? planar(flat(pose) * flat(target)) : target);
        }
        state = State::RUNNING;
        start_trial(pose);
    }

    void WalkPathBenchmark::start_trial(const Eigen::Vector3d& pose) {
        trial               = Trial{};
        trial.start         = NUClear::clock::now();
        trial.start_pose    = pose;
        trial.last_position = pose.head<2>();
        const auto& target  = targets[trial_index];
        log<INFO>(fmt::format("Trial {} start ({:.2f}, {:.2f}, {:.2f}) -> target ({:.2f}, {:.2f}, {:.2f})",
                              trial_index,
                              pose.x(),
                              pose.y(),
                              pose.z(),
                              target.x(),
                              target.y(),
                              target.z()));
    }

    void WalkPathBenchmark::finish_trial(const Eigen::Vector3d& pose, const Outcome outcome) {
        const auto& target    = targets[trial_index];
        const double duration = outcome == Outcome::OK ? trial.settle_start
                                : outcome == Outcome::SHORT
                                    ? trial.rest_start
                                    : std::chrono::duration<double>(NUClear::clock::now() - trial.start).count();
        const double distance = position_error(trial.start_pose, target);
        const double pos_err  = position_error(pose, target);
        const double yaw_err  = heading_error(pose, target);

        log<INFO>(
            fmt::format("TRIAL {} {} dist={:.2f} turn={:.2f} time={:.2f} reach={:.2f} pos_err={:.3f} "
                        "yaw_err={:.3f} path={:.2f} path_ratio={:.2f} max_err_after_reach={:.3f} falls={}",
                        trial_index,
                        outcome == Outcome::OK      ? "OK"
                        : outcome == Outcome::SHORT ? "SHORT"
                                                    : "TIMEOUT",
                        distance,
                        heading_error(trial.start_pose, target),
                        duration,
                        trial.reach_time,
                        pos_err,
                        yaw_err,
                        trial.path_length,
                        distance > 0.05 ? trial.path_length / distance : 0.0,
                        trial.max_error_after_reach,
                        trial.falls));

        total_time += duration;
        total_pos_error += pos_err;
        total_head_error += yaw_err;
        total_falls += trial.falls;
        failures += outcome == Outcome::OK ? 0 : 1;

        if (++trial_index < targets.size()) {
            start_trial(pose);
            return;
        }

        state        = State::DONE;
        const auto n = static_cast<double>(targets.size());
        log<INFO>(
            fmt::format("SUMMARY trials={} failures={} falls={} total_time={:.2f} mean_pos_err={:.3f} "
                        "mean_yaw_err={:.3f}",
                        targets.size(),
                        failures,
                        total_falls,
                        total_time,
                        total_pos_error / n,
                        total_head_error / n));
        if (csv.is_open()) {
            csv.close();
        }
        if (cfg.shutdown_when_done) {
            powerplant.shutdown();
        }
    }

}  // namespace module::tools
