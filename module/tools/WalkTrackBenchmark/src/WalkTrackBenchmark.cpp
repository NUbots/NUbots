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
#include "WalkTrackBenchmark.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fmt/format.h>
#include <iomanip>
#include <numeric>
#include <sstream>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/booster/NUSimGroundTruth.hpp"
#include "message/eye/DataPoint.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
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
    using message::strategy::FallRecovery;
    using message::strategy::WalkToFieldPosition;

    using utility::math::euler::pos_rpy_to_transform;
    using utility::support::Expression;

    namespace {

        /// @brief Local time as a file name, as DataLogging names its recordings
        std::string formatted_time() {
            std::time_t now     = time(nullptr);
            std::tm system_time = *localtime(&now);
            std::stringstream time;
            time << std::put_time(&system_time, "%Y%m%dT%H_%M_%S");
            return time.str();
        }

        /// @brief Planar pose [x, y, yaw] of an isometry's ground projection
        Eigen::Vector3d planar(const Eigen::Isometry3d& H) {
            return {H.translation().x(), H.translation().y(), std::atan2(H.linear()(1, 0), H.linear()(0, 0))};
        }

        /// @brief Yaw-only isometry at the ground projection of a planar pose
        Eigen::Isometry3d flat(const Eigen::Vector3d& pose) {
            return pos_rpy_to_transform(Eigen::Vector3d(pose.x(), pose.y(), 0.0), Eigen::Vector3d(0.0, 0.0, pose.z()));
        }

        double heading_error(const Eigen::Vector3d& pose, const Eigen::Vector3d& target) {
            return std::abs(std::remainder(target.z() - pose.z(), 2.0 * M_PI));
        }

        double mean(const std::vector<double>& values) {
            return values.empty() ? 0.0 : std::accumulate(values.begin(), values.end(), 0.0) / values.size();
        }

        /// @brief The value a fraction p of the values are at or below (nearest rank)
        double percentile(std::vector<double> values, const double p) {
            if (values.empty()) {
                return 0.0;
            }
            const auto rank = static_cast<size_t>(std::ceil(p * values.size())) - 1;
            std::nth_element(values.begin(), values.begin() + rank, values.end());
            return values[rank];
        }

        double share_above(const std::vector<double>& values, const double limit) {
            return values.empty()
                       ? 0.0
                       : static_cast<double>(std::count_if(values.begin(), values.end(), [&](const double v) {
                             return v > limit;
                         })) / values.size();
        }

        /// @brief An optional number in a YAML map, which may be an expression such as `pi / 2`
        double optional(const YAML::Node& node, const std::string& key, const double fallback = 0.0) {
            return node[key] ? static_cast<double>(node[key].as<Expression>()) : fallback;
        }

    }  // namespace

    WalkTrackBenchmark::WalkTrackBenchmark(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("WalkTrackBenchmark.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.start_delay      = config["start_delay"].as<double>();
            cfg.relative_targets = config["relative_targets"].as<bool>();
            cfg.park_ball.clear();
            for (const auto& v : config["park_ball"]) {
                cfg.park_ball.push_back(v.as<double>());
            }
            cfg.tracks.clear();
            for (const auto& node : config["tracks"]) {
                TrackConfig track;
                track.name           = node["name"].as<std::string>();
                track.start          = Eigen::Vector3d(node["start"].as<Expression>());
                track.heading_offset = optional(node, "heading_offset");
                for (const auto& s : node["segments"]) {
                    track.segments.push_back(walk_track::Segment{optional(s, "speed"),
                                                                 optional(s, "acceleration"),
                                                                 optional(s, "curvature"),
                                                                 optional(s, "turn"),
                                                                 s["duration"].as<Expression>()});
                }
                cfg.tracks.push_back(track);
            }
            cfg.speed_scales            = config["speed_scales"].as<std::vector<double>>();
            cfg.approach_position_error = config["approach"]["position_error"].as<double>();
            cfg.approach_heading_error  = config["approach"]["heading_error"].as<Expression>();
            cfg.approach_settle         = config["approach"]["settle_time"].as<double>();
            cfg.approach_timeout        = config["approach"]["timeout"].as<double>();
            cfg.lost_distance           = config["lost_distance"].as<double>();
            cfg.fall_height             = config["fall_height"].as<double>();
            cfg.ground_truth_field      = config["ground_truth_field"].as<bool>();
            cfg.fall_recovery_priority  = config["fall_recovery_priority"].as<int>();
            cfg.shutdown_when_done      = config["shutdown_when_done"].as<bool>();
            cfg.csv_directory           = config["csv_directory"].as<std::string>();
        });

        on<Startup>().then([this] {
            startup_time = NUClear::clock::now();
            if (!cfg.csv_directory.empty()) {
                // One CSV per run, named by its start time
                std::filesystem::create_directories(cfg.csv_directory);
                const std::filesystem::path path = cfg.csv_directory / fmt::format("{}.csv", formatted_time());
                csv.open(path);
                log<INFO>("Writing the trajectory to", path.string());
                csv << "t,run,phase,x,y,yaw,z,target_x,target_y,target_yaw,target_vx,target_vy,cmd_vx,cmd_vy,cmd_wz\n";
            }
        });

        // The commanded walk after the planner's smoothing and dead zone
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

        on<Every<10, Per<std::chrono::seconds>>, With<NUSimRobotGroundTruth>, Sync<WalkTrackBenchmark>>().then(
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
                    if (cfg.tracks.empty() || cfg.speed_scales.empty()) {
                        log<ERROR>("No tracks or speed scales configured");
                        state = State::DONE;
                        return;
                    }
                    begin(pose);
                }
                if (state == State::DONE) {
                    return;
                }

                const Run& run = runs[run_index];
                const double t = std::chrono::duration<double>(now - progress.phase_start).count();
                // The target holds its start pose during the approach, then moves
                const walk_track::TargetState target = run.track.at(state == State::TRACKING ? t : 0.0);

                // Never stop at the target: the planner is given it every tick, moving or not
                emit<Task>(std::make_unique<WalkToFieldPosition>(flat(target.pose), false), 1);

                const Eigen::Vector2d error = pose.head<2>() - target.pose.head<2>();
                const double pos_err        = error.norm();
                const double yaw_err        = heading_error(pose, target.pose);

                const bool fallen = height < cfg.fall_height;
                if (fallen && !progress.fallen) {
                    ++progress.falls;
                    log<WARN>("Fell during", run.name);
                }
                progress.fallen = fallen;

                if (csv.is_open()) {
                    csv << fmt::format(
                        "{:.2f},{},{},{:.4f},{:.4f},{:.4f},{:.3f},{:.4f},{:.4f},{:.4f},{:.3f},{:.3f},"
                        "{:.3f},{:.3f},{:.3f}\n",
                        t,
                        run_index,
                        state == State::TRACKING ? 1 : 0,
                        pose.x(),
                        pose.y(),
                        pose.z(),
                        height,
                        target.pose.x(),
                        target.pose.y(),
                        target.pose.z(),
                        target.velocity.x(),
                        target.velocity.y(),
                        walk_command.x(),
                        walk_command.y(),
                        walk_command.z());
                }

                if (state == State::APPROACH) {
                    const bool at_start =
                        !fallen && pos_err < cfg.approach_position_error && yaw_err < cfg.approach_heading_error;
                    progress.settle_start = at_start ? (progress.settle_start < 0.0 ? t : progress.settle_start) : -1.0;
                    const bool settled =
                        progress.settle_start >= 0.0 && t - progress.settle_start >= cfg.approach_settle;
                    if (settled || t > cfg.approach_timeout) {
                        progress.approach_time = t;
                        progress.approached    = settled;
                        progress.phase_start   = now;
                        state                  = State::TRACKING;
                        log<INFO>(fmt::format("Run {} {} x{:.2f} moving after {:.1f} s{}",
                                              run_index,
                                              run.name,
                                              run.speed_scale,
                                              t,
                                              settled ? "" : " (approach timed out)"));
                    }
                    return;
                }

                // Lag along the travel direction (positive behind the target) and offset across it
                const Eigen::Vector2d direction(std::cos(target.direction), std::sin(target.direction));
                progress.position_errors.push_back(pos_err);
                progress.heading_errors.push_back(yaw_err);
                progress.along_errors.push_back(-error.dot(direction));
                progress.cross_errors.push_back(std::abs(error.x() * direction.y() - error.y() * direction.x()));

                if (t >= run.track.duration()) {
                    finish_run();
                }
            });
    }

    void WalkTrackBenchmark::begin(const Eigen::Vector3d& pose) {
        // Place every track in the field frame, once per speed scale
        runs.clear();
        for (const double scale : cfg.speed_scales) {
            for (const auto& track : cfg.tracks) {
                const Eigen::Vector3d start =
                    cfg.relative_targets ? planar(flat(pose) * flat(track.start)) : track.start;
                runs.push_back(
                    Run{track.name, scale, walk_track::Track(start, track.heading_offset, track.segments, scale)});
            }
        }
        start_run();
    }

    void WalkTrackBenchmark::start_run() {
        progress             = Progress{};
        progress.phase_start = NUClear::clock::now();
        state                = State::APPROACH;
        const Run& run       = runs[run_index];
        const auto start     = run.track.at(0.0).pose;
        log<INFO>(fmt::format("Run {} {} x{:.2f}: approaching ({:.2f}, {:.2f}, {:.2f}), then {:.1f} s moving",
                              run_index,
                              run.name,
                              run.speed_scale,
                              start.x(),
                              start.y(),
                              start.z(),
                              run.track.duration()));
    }

    void WalkTrackBenchmark::finish_run() {
        const Run& run = runs[run_index];
        const auto& p  = progress;
        log<INFO>(fmt::format(
            "TRACK {} {} x{:.2f} approach={:.1f}{} duration={:.1f} pos_mean={:.3f} pos_p90={:.3f} "
            "pos_max={:.3f} yaw_mean={:.3f} yaw_p90={:.3f} lag={:.3f} cross={:.3f} lost={:.3f} falls={}",
            run_index,
            run.name,
            run.speed_scale,
            p.approach_time,
            p.approached ? "" : "(timeout)",
            run.track.duration(),
            mean(p.position_errors),
            percentile(p.position_errors, 0.9),
            p.position_errors.empty() ? 0.0 : *std::max_element(p.position_errors.begin(), p.position_errors.end()),
            mean(p.heading_errors),
            percentile(p.heading_errors, 0.9),
            mean(p.along_errors),
            mean(p.cross_errors),
            share_above(p.position_errors, cfg.lost_distance),
            p.falls));

        all_position_errors.insert(all_position_errors.end(), p.position_errors.begin(), p.position_errors.end());
        all_heading_errors.insert(all_heading_errors.end(), p.heading_errors.begin(), p.heading_errors.end());
        total_falls += p.falls;

        if (++run_index < runs.size()) {
            start_run();
            return;
        }

        state = State::DONE;
        log<INFO>(
            fmt::format("SUMMARY tracks={} falls={} pos_mean={:.3f} pos_p90={:.3f} yaw_mean={:.3f} "
                        "yaw_p90={:.3f} lost={:.3f}",
                        runs.size(),
                        total_falls,
                        mean(all_position_errors),
                        percentile(all_position_errors, 0.9),
                        mean(all_heading_errors),
                        percentile(all_heading_errors, 0.9),
                        share_above(all_position_errors, cfg.lost_distance)));
        if (csv.is_open()) {
            csv.close();
        }
        if (cfg.shutdown_when_done) {
            powerplant.shutdown();
        }
    }

}  // namespace module::tools
