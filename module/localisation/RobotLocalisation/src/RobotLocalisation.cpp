/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#include "RobotLocalisation.hpp"

#include <algorithm>
#include <cmath>
#include <fmt/format.h>
#include <limits>

#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/input/Image.hpp"
#include "message/input/RoboCup.hpp"
#include "message/localisation/Robot.hpp"
#include "message/vision/Robot.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/projection.hpp"

namespace module::localisation {

    using extension::Configuration;

    using LocalisationRobot  = message::localisation::Robot;
    using LocalisationRobots = message::localisation::Robots;
    using VisionRobot        = message::vision::Robot;
    using VisionRobots       = message::vision::Robots;
    using PenaltyState       = message::input::State;

    using message::eye::DataPoint;
    using message::input::GameState;
    using message::input::Image;
    using message::input::RoboCup;
    using message::localisation::Field;
    using message::purpose::Purpose;
    using message::purpose::SoccerPosition;
    using message::support::FieldDescription;

    using utility::nusight::graph;
    using utility::support::Expression;
    using utility::vision::project;

    RobotLocalisation::RobotLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("RobotLocalisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RobotLocalisation.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Set our UKF filter parameters
            cfg.ukf.noise.measurement.position =
                Eigen::Vector2d(config["ukf"]["noise"]["measurement"]["robot_position"].as<Expression>()).asDiagonal();
            cfg.ukf.noise.process.position      = config["ukf"]["noise"]["process"]["position"].as<Expression>();
            cfg.ukf.noise.process.velocity      = config["ukf"]["noise"]["process"]["velocity"].as<Expression>();
            cfg.ukf.initial_covariance.position = config["ukf"]["initial_covariance"]["position"].as<Expression>();
            cfg.ukf.initial_covariance.velocity = config["ukf"]["initial_covariance"]["velocity"].as<Expression>();
            cfg.ukf.initial_covariance.vision_velocity =
                config["ukf"]["initial_covariance"]["vision_velocity"].as<Expression>();
            cfg.measurement.range_base        = config["measurement"]["range_base"].as<double>();
            cfg.measurement.range_quad        = config["measurement"]["range_quad"].as<double>();
            cfg.measurement.bearing_per_metre = config["measurement"]["bearing_per_metre"].as<double>();
            cfg.measurement.pose_std          = config["measurement"]["pose_std"].as<double>();
            cfg.gate_chi2                     = config["gate_chi2"].as<double>();
            cfg.max_track_age                 = config["max_track_age"].as<double>();
            cfg.merge_distance                = config["merge_distance"].as<double>();
            cfg.max_robot_tracks              = config["max_robot_tracks"].as<int>();
            cfg.max_missed_count              = config["max_missed_count"].as<int>();
            cfg.max_distance_from_field       = config["max_distance_from_field"].as<double>();
            cfg.max_localisation_cost         = config["max_localisation_cost"].as<double>();
            cfg.teammate_timeout              = config["teammate_timeout"].as<double>();
        });

        on<Every<UPDATE_RATE, Per<std::chrono::seconds>>,
           With<Image>,
           With<Field>,
           With<FieldDescription>,
           Sync<RobotLocalisation>>()
            .then([this](const Image& image, const Field& field, const FieldDescription& field_desc) {
                // **Run maintenance step**
                maintenance(image, field, field_desc);

                // **Debugging output**
                debug_info();

                // **Emit the localisation of the robots**
                auto localisation_robots = std::make_unique<LocalisationRobots>();
                for (const auto& tracked_robot : tracked_robots) {
                    auto state = RobotModel<double>::StateVec(tracked_robot.ukf.get_state());
                    LocalisationRobot localisation_robot;
                    localisation_robot.id                  = tracked_robot.id;
                    localisation_robot.rRWw                = Eigen::Vector3d(state.rRWw.x(), state.rRWw.y(), 0);
                    localisation_robot.vRw                 = Eigen::Vector3d(state.vRw.x(), state.vRw.y(), 0);
                    localisation_robot.covariance          = tracked_robot.ukf.get_covariance();
                    localisation_robot.time_of_measurement = tracked_robot.last_time_update;

                    localisation_robot.teammate = tracked_robot.teammate;
                    localisation_robot.purpose  = tracked_robot.purpose;

                    localisation_robots->robots.push_back(localisation_robot);
                }

                emit(std::move(localisation_robots));
            });

        on<Trigger<RoboCup>, With<Field>, Sync<RobotLocalisation>>().then(
            [this](const RoboCup& robocup, const Field& field) {
                // **Run prediction step**
                prediction();

                // **Data association**
                // RoboCup messages come from teammates. Their position is (x, y, yaw) in the mixed-team protocol's
                // field frame, which is our field frame rotated 180 degrees about the centre (RobotCommunication
                // negates x and y before sending), so undo that and convert to world.
                const Eigen::Vector3d rRFf(-robocup.current_pose.position.x(), -robocup.current_pose.position.y(), 0.0);
                std::vector<Eigen::Vector3d> robots_rRWw{field.Hfw.inverse() * rRFf};
                auto purpose = std::make_unique<Purpose>();
                // Set the purpose message with the teammate's player id and whether they are going for the ball
                purpose->player_id = robocup.current_pose.player_id;
                purpose->purpose   = robocup.going_for_ball ? SoccerPosition::ATTACK : SoccerPosition::SUPPORT;
                // Teammates only count in strategy decisions if active, so mark them active unless penalised
                purpose->active = robocup.state != PenaltyState::PENALISED;

                // Run data association step
                data_association(robots_rRWw, purpose);
            });

        on<Trigger<VisionRobots>, Sync<RobotLocalisation>>().then([this](const VisionRobots& vision_robots) {
            // **Run prediction step**
            prediction();

            // **Data association**
            // Transform the robot positions from camera coordinates to world coordinates
            Eigen::Isometry3d Hwc = Eigen::Isometry3d(vision_robots.Hcw).inverse();
            // Make vectors with the transformed positions and their measurement covariances
            std::vector<Eigen::Vector3d> robots_rRWw{};
            std::vector<Eigen::Matrix2d> covariances{};
            for (const auto& vision_robot : vision_robots.robots) {
                Eigen::Vector3d rRWw = Hwc * vision_robot.rRCc;
                robots_rRWw.push_back(rRWw);
                covariances.push_back(vision_covariance(rRWw, Hwc.translation()));
            }
            // Run data association step
            vision_association(robots_rRWw, covariances);
        });
    }

    void RobotLocalisation::prediction() {
        auto now = NUClear::clock::now();

        for (auto& tracked_robot : tracked_robots) {
            double dt =
                std::chrono::duration_cast<std::chrono::duration<double>>(now - tracked_robot.last_time_update).count();
            tracked_robot.last_time_update = now;
            tracked_robot.ukf.time(dt);

            // Vision runs far more often than teammates broadcast (every camera frame vs 2 Hz), so resetting
            // seen here would almost always clobber it again before the next comms message arrives while the
            // teammate is outside camera view. Teammates are instead kept alive by comms staleness in
            // maintenance(), so leave their seen/missed_count alone here.
            if (!tracked_robot.teammate) {
                tracked_robot.seen = false;
            }
        }
    }

    void RobotLocalisation::data_association(const std::vector<Eigen::Vector3d>& robots_rRWw,
                                             const std::unique_ptr<Purpose>& purpose) {
        for (const auto& rRWw : robots_rRWw) {
            if (tracked_robots.empty()) {
                // If there are no tracked robots, add this as a new robot
                tracked_robots.emplace_back(TrackedRobot(rRWw, cfg.ukf, next_id++, purpose));
                continue;
            }

            // If this is a teammate, find it in the list of tracked robots
            // If it doesn't exist, add it. Purpose is checked but should always be set for teammates.
            if (purpose) {
                // Check if the robot is already in the list of tracked robots
                auto teammate_itr =
                    std::find_if(tracked_robots.begin(), tracked_robots.end(), [&purpose](const TrackedRobot& robot) {
                        return robot.purpose.player_id == purpose->player_id;
                    });

                // If the robot is not in the list, add it
                if (teammate_itr == tracked_robots.end()) {
                    tracked_robots.emplace_back(TrackedRobot(rRWw, cfg.ukf, next_id++, purpose));
                    continue;
                }
                // If the robot is in the list, update it with the new position
                teammate_itr->ukf.measure(Eigen::Vector2d(rRWw.head<2>()),
                                          cfg.ukf.noise.measurement.position,
                                          MeasurementType::ROBOT_POSITION());
                teammate_itr->seen            = true;
                teammate_itr->purpose         = *purpose;
                teammate_itr->last_comms_time = NUClear::clock::now();

                continue;
            }
        }
    }

    Eigen::Matrix2d RobotLocalisation::vision_covariance(const Eigen::Vector3d& rRWw,
                                                         const Eigen::Vector3d& rCWw) const {
        const Eigen::Vector2d d = rRWw.head<2>() - rCWw.head<2>();
        const double r          = std::max(d.norm(), 1e-3);
        const Eigen::Vector2d u = d / r;
        const Eigen::Vector2d t(-u.y(), u.x());

        const double pose_var  = cfg.measurement.pose_std * cfg.measurement.pose_std;
        const double range_std = cfg.measurement.range_base + cfg.measurement.range_quad * r * r;
        const double tang_std  = cfg.measurement.bearing_per_metre * r;

        const double range_var = range_std * range_std + pose_var;
        const double tang_var  = tang_std * tang_std + pose_var;
        return range_var * u * u.transpose() + tang_var * t * t.transpose();
    }

    void RobotLocalisation::vision_association(const std::vector<Eigen::Vector3d>& robots_rRWw,
                                               const std::vector<Eigen::Matrix2d>& covariances) {
        const size_t n_tracks = tracked_robots.size();
        const size_t n_dets   = robots_rRWw.size();
        const double inf      = std::numeric_limits<double>::infinity();

        // Squared Mahalanobis distance of every detection to every track, infinite if outside the gate
        std::vector<std::vector<double>> cost(n_tracks, std::vector<double>(n_dets, inf));
        for (size_t i = 0; i < n_tracks; ++i) {
            const Eigen::Vector2d mean = tracked_robots[i].get_rRWw();
            const Eigen::Matrix2d P    = tracked_robots[i].ukf.get_covariance().topLeftCorner<2, 2>();
            for (size_t j = 0; j < n_dets; ++j) {
                const Eigen::Vector2d nu = robots_rRWw[j].head<2>() - mean;
                const double d2          = nu.dot((P + covariances[j]).ldlt().solve(nu));
                if (d2 <= cfg.gate_chi2) {
                    cost[i][j] = d2;
                }
            }
        }

        // Greedy one-to-one assignment, lowest cost first
        std::vector<bool> detection_used(n_dets, false);
        while (true) {
            double best = inf;
            size_t bi = 0, bj = 0;
            for (size_t i = 0; i < n_tracks; ++i) {
                for (size_t j = 0; j < n_dets; ++j) {
                    if (cost[i][j] < best) {
                        best = cost[i][j];
                        bi   = i;
                        bj   = j;
                    }
                }
            }
            if (!std::isfinite(best)) {
                break;
            }
            auto& track = tracked_robots[bi];
            track.ukf.measure(Eigen::Vector2d(robots_rRWw[bj].head<2>()),
                              covariances[bj],
                              MeasurementType::ROBOT_POSITION());
            track.seen             = true;
            track.last_vision_time = NUClear::clock::now();
            track.vision_updates++;
            detection_used[bj] = true;
            for (size_t k = 0; k < n_tracks; ++k) {
                cost[k][bj] = inf;
            }
            std::fill(cost[bi].begin(), cost[bi].end(), inf);
        }

        // Detections that matched no track are new robots
        for (size_t j = 0; j < n_dets; ++j) {
            if (!detection_used[j]) {
                tracked_robots.emplace_back(TrackedRobot(robots_rRWw[j], cfg.ukf, next_id++, nullptr, &covariances[j]));
            }
        }
    }

    void RobotLocalisation::maintenance(const Image& image, const Field& field, const FieldDescription& field_desc) {
        std::vector<TrackedRobot> new_tracked_robots{};
        const auto now = NUClear::clock::now();

        // Sort tracked_robots so that teammates are at the front, then oldest first. A track is dropped as a
        // duplicate of a track earlier in this order, so teammates and older tracks win.
        std::sort(tracked_robots.begin(), tracked_robots.end(), [](const TrackedRobot& a, const TrackedRobot& b) {
            if (a.purpose.player_id != b.purpose.player_id) {
                return a.purpose.player_id > b.purpose.player_id;
            }
            return a.id < b.id;
        });

        for (auto& tracked_robot : tracked_robots) {
            auto state = RobotModel<double>::StateVec(tracked_robot.ukf.get_state());

            if (tracked_robot.teammate) {
                // If a teammate has moved outside of the camera's field of view, keep it as seen so we don't
                // lose it. Project its world position into the camera's pixel space and check whether it
                // falls outside the image bounds.
                Eigen::Vector3d rRCc = image.Hcw * Eigen::Vector3d(state.rRWw.x(), state.rRWw.y(), 0);

                bool in_view = false;
                if (rRCc.x() > 0.0) {  // in front of the camera
                    Eigen::Vector2d dimensions      = Eigen::Vector2d(image.dimensions.x(), image.dimensions.y());
                    Eigen::Vector2d norm_dimensions = dimensions / dimensions.x();
                    Eigen::Vector2d pixel           = project(rRCc.normalized(), image.lens, norm_dimensions);
                    in_view = pixel.x() >= 0.0 && pixel.x() < norm_dimensions.x() && pixel.y() >= 0.0
                              && pixel.y() < norm_dimensions.y();
                }

                if (!in_view) {
                    tracked_robot.seen = true;
                }

                // If the tracked robot has not been seen, increment the consecutively missed count
                tracked_robot.missed_count = tracked_robot.seen ? 0 : tracked_robot.missed_count + 1;

                // Don't keep this robot if it has been missed too many times
                if (tracked_robot.missed_count > cfg.max_missed_count) {
                    log<DEBUG>(fmt::format("Removing robot {} due to missed count", tracked_robot.id));
                    continue;
                }
            }
            else {
                // Opponents (and unidentified robots) are removed when they have not had a vision update recently
                double age =
                    std::chrono::duration_cast<std::chrono::duration<double>>(now - tracked_robot.last_vision_time)
                        .count();
                if (age > cfg.max_track_age) {
                    log<DEBUG>(fmt::format("Removing robot {} due to age without vision update", tracked_robot.id));
                    continue;
                }
            }

            // Remove duplicates: a robot that is on top of one already kept is the same robot
            bool duplicate = false;
            for (const auto& other_robot : new_tracked_robots) {
                if ((tracked_robot.get_rRWw() - other_robot.get_rRWw()).norm() < cfg.merge_distance) {
                    log<DEBUG>(fmt::format("Removing robot {} due to proximity to robot {}",
                                           tracked_robot.id,
                                           other_robot.id));
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) {
                continue;
            }

            Eigen::Vector3d rRFf = field.Hfw * Eigen::Vector3d(state.rRWw.x(), state.rRWw.y(), 0);

            if (rRFf.x() < (-field_desc.dimensions.field_length / 2) - cfg.max_distance_from_field
                || rRFf.x() > (field_desc.dimensions.field_length / 2) + cfg.max_distance_from_field
                || rRFf.y() < (-field_desc.dimensions.field_width / 2) - cfg.max_distance_from_field
                || rRFf.y() > (field_desc.dimensions.field_width / 2) + cfg.max_distance_from_field) {
                log<DEBUG>(fmt::format("Removing robot {} due to location (outside field)", tracked_robot.id));
                continue;
            }

            // If removal conditions not met, keep the robot
            new_tracked_robots.push_back(tracked_robot);
        }

        // Keep only the best confirmed non-teammate tracks when the number of robots on the field is limited
        if (cfg.max_robot_tracks > 0) {
            std::vector<size_t> candidates{};
            for (size_t i = 0; i < new_tracked_robots.size(); ++i) {
                if (!new_tracked_robots[i].teammate) {
                    candidates.push_back(i);
                }
            }
            if (candidates.size() > static_cast<size_t>(cfg.max_robot_tracks)) {
                std::stable_sort(candidates.begin(), candidates.end(), [&](size_t a, size_t b) {
                    return new_tracked_robots[a].vision_updates > new_tracked_robots[b].vision_updates;
                });
                std::vector<bool> remove(new_tracked_robots.size(), false);
                for (size_t k = cfg.max_robot_tracks; k < candidates.size(); ++k) {
                    remove[candidates[k]] = true;
                }
                std::vector<TrackedRobot> limited{};
                for (size_t i = 0; i < new_tracked_robots.size(); ++i) {
                    if (!remove[i]) {
                        limited.push_back(new_tracked_robots[i]);
                    }
                }
                new_tracked_robots = std::move(limited);
            }
        }

        tracked_robots = std::move(new_tracked_robots);
    }

    void RobotLocalisation::debug_info() const {
        // Print tracked_robots ids
        log<DEBUG>("Robots tracked:");
        for (const auto& tracked_robot : tracked_robots) {
            log<DEBUG>("\tID: ",
                       tracked_robot.id,
                       ": ",
                       tracked_robot.teammate ? "Teammate" : "Opponent",
                       "teammate ID: ",
                       tracked_robot.purpose.player_id,
                       "position: ",
                       RobotModel<double>::StateVec(tracked_robot.ukf.get_state()).rRWw.transpose());
        }
    }

}  // namespace module::localisation
