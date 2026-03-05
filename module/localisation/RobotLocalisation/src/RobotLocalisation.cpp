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

#include <fmt/format.h>

#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/input/RoboCup.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/Robot.hpp"
#include "message/support/GlobalConfig.hpp"
#include "message/vision/Robot.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

namespace module::localisation {

    using extension::Configuration;

    using LocalisationRobot  = message::localisation::Robot;
    using LocalisationRobots = message::localisation::Robots;
    using VisionRobot        = message::vision::Robot;
    using VisionRobots       = message::vision::Robots;
    using PenaltyState       = message::input::State;

    using message::eye::DataPoint;
    using message::input::GameState;
    using message::input::RoboCup;
    using message::localisation::Field;
    using message::localisation::TeammateObservedSelf;
    using message::localisation::TeammateVisionLandmark;
    using message::purpose::Purpose;
    using message::purpose::SoccerPosition;
    using message::support::FieldDescription;
    using message::support::GlobalConfig;
    using message::vision::GreenHorizon;

    using utility::math::geometry::point_in_convex_hull;
    using utility::nusight::graph;
    using utility::support::Expression;

    RobotLocalisation::RobotLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("RobotLocalisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RobotLocalisation.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Set our UKF filter parameters
            cfg.ukf.noise.measurement.position =
                Eigen::Vector2d(config["ukf"]["noise"]["measurement"]["robot_position"].as<Expression>()).asDiagonal();
            cfg.ukf.noise.measurement.position_robocup =
                Eigen::Vector2d(config["ukf"]["noise"]["measurement"]["robot_position_robocup"].as<Expression>())
                    .asDiagonal();
            cfg.ukf.noise.measurement.position_indirect =
                Eigen::Vector2d(config["ukf"]["noise"]["measurement"]["robot_position_indirect"].as<Expression>())
                    .asDiagonal();
            cfg.ukf.noise.process.position      = config["ukf"]["noise"]["process"]["position"].as<Expression>();
            cfg.ukf.noise.process.velocity      = config["ukf"]["noise"]["process"]["velocity"].as<Expression>();
            cfg.ukf.initial_covariance.position = config["ukf"]["initial_covariance"]["position"].as<Expression>();
            cfg.ukf.initial_covariance.velocity = config["ukf"]["initial_covariance"]["velocity"].as<Expression>();
            cfg.association_distance             = config["association_distance"].as<double>();
            cfg.landmark_association_distance    = config["landmark_association_distance"].as<double>();
            cfg.max_missed_count                = config["max_missed_count"].as<int>();
            cfg.max_distance_from_field         = config["max_distance_from_field"].as<double>();
            cfg.max_localisation_cost           = config["max_localisation_cost"].as<double>();
            cfg.max_opponent_count              = config["max_opponent_count"].as<int>();
        });

        on<Every<UPDATE_RATE, Per<std::chrono::seconds>>,
           With<GreenHorizon>,
           With<Field>,
           With<FieldDescription>,
           Sync<RobotLocalisation>>()
            .then([this](const GreenHorizon& horizon, const Field& field, const FieldDescription& field_desc) {
                // **Run maintenance step**
                maintenance(horizon, field, field_desc);

                // **Debugging output**
                debug_info();

                // **Emit the localisation of the robots**
                auto localisation_robots = std::make_unique<LocalisationRobots>();
                for (const auto& tracked_robot : tracked_robots) {
                    auto state = RobotModel<double>::StateVec(tracked_robot.ukf.get_state());
                    LocalisationRobot localisation_robot;
                    localisation_robot.id                  = tracked_robot.canonical_id;
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

        on<Trigger<RoboCup>, With<Field>, With<GlobalConfig>, Sync<RobotLocalisation>>().then(
            [this](const RoboCup& robocup, const Field& field, const GlobalConfig& global_config) {
                // Do not consider a teammate's localisation if their cost is too high
                if (robocup.current_pose.cost > cfg.max_localisation_cost) {
                    log<DEBUG>("Teammate's localisation cost is too high, not processing.");
                    return;
                }

                // **Run prediction step**
                prediction();

                // **Data association — sender's own position**
                // RoboCup messages come from teammates. Their position is in field space, so convert to world.
                Eigen::Vector3d sender_rRFf = robocup.current_pose.position.cast<double>();
                Eigen::Vector3d sender_rRWw = field.Hfw.inverse() * sender_rRFf;
                std::vector<Eigen::Vector3d> robots_rRWw{sender_rRWw};
                auto purpose = std::make_unique<Purpose>(robocup.purpose);
                data_association(robots_rRWw, purpose);

                // Store the sender's broadcast field position on their track for use as a vision landmark
                auto sender_itr = std::find_if(tracked_robots.begin(),
                                               tracked_robots.end(),
                                               [&purpose](const TrackedRobot& r) {
                                                   return r.purpose.player_id == purpose->player_id;
                                               });
                if (sender_itr != tracked_robots.end()) {
                    sender_itr->last_broadcast_rRFf = sender_rRFf;
                    sender_itr->has_broadcast_rRFf  = true;
                }

                // **Data association — robots the teammate has detected**
                // Each entry is a robot the sender has seen; positions are in field space.
                for (const auto& rc_robot : robocup.others) {
                    // If this entry refers to ourselves, use it to constrain our own field localisation
                    if (rc_robot.player_id > 0 && rc_robot.player_id == global_config.player_id) {
                        auto obs          = std::make_unique<TeammateObservedSelf>();
                        obs->position     = rc_robot.position;
                        obs->sender_cost  = static_cast<float>(robocup.current_pose.cost);
                        emit(std::move(obs));
                        continue;
                    }
                    Eigen::Vector3d rRWw = field.Hfw.inverse() * rc_robot.position.cast<double>();

                    std::unique_ptr<Purpose> p = nullptr;
                    if (rc_robot.player_id > 0) {
                        p              = std::make_unique<Purpose>();
                        p->player_id   = rc_robot.player_id;
                    }

                    data_association({rRWw}, p, cfg.ukf.noise.measurement.position_indirect);

                    // For opponent sightings, propagate the sender's tracking_id as a canonical display ID.
                    // "First reporter wins": only set if this robot hasn't been given a canonical_id yet.
                    if (rc_robot.tracking_id > 0 && rc_robot.player_id == 0) {
                        auto itr = std::min_element(tracked_robots.begin(),
                                                    tracked_robots.end(),
                                                    [&rRWw](const TrackedRobot& a, const TrackedRobot& b) {
                                                        return (rRWw.head<2>() - a.get_rRWw()).norm()
                                                               < (rRWw.head<2>() - b.get_rRWw()).norm();
                                                    });
                        if (itr != tracked_robots.end() && !itr->teammate
                            && itr->canonical_id == static_cast<uint32_t>(itr->id)) {
                            itr->canonical_id = rc_robot.tracking_id;
                        }
                    }
                }
            });

        on<Trigger<VisionRobots>, Sync<RobotLocalisation>>().then([this](const VisionRobots& vision_robots) {
            // **Run prediction step**
            prediction();

            // **Data association**
            // Transform the robot positions from camera coordinates to world coordinates
            Eigen::Isometry3d Hwc = Eigen::Isometry3d(vision_robots.Hcw).inverse();
            std::vector<Eigen::Vector3d> robots_rRWw{};
            for (const auto& vision_robot : vision_robots.robots) {
                Eigen::Vector3d rRWw = Hwc * vision_robot.rRCc;
                robots_rRWw.push_back(rRWw);

                // If this vision detection matches a known teammate with a broadcast position,
                // emit a landmark message so FieldLocalisationNLopt can use it to constrain our Hfw
                if (!tracked_robots.empty()) {
                    auto closest =
                        std::min_element(tracked_robots.begin(),
                                         tracked_robots.end(),
                                         [&rRWw](const TrackedRobot& a, const TrackedRobot& b) {
                                             return (rRWw.head<2>() - a.get_rRWw()).norm()
                                                    < (rRWw.head<2>() - b.get_rRWw()).norm();
                                         });
                    if (closest != tracked_robots.end() && closest->teammate && closest->has_broadcast_rRFf
                        && (rRWw.head<2>() - closest->get_rRWw()).norm() < cfg.landmark_association_distance) {
                        auto landmark  = std::make_unique<TeammateVisionLandmark>();
                        landmark->rRCc = vision_robot.rRCc.cast<float>();
                        landmark->rRFf = closest->last_broadcast_rRFf.cast<float>();
                        landmark->Hcw  = vision_robots.Hcw;
                        emit(std::move(landmark));
                    }
                }
            }
            // Run data association step
            data_association(robots_rRWw);
        });
    }

    void RobotLocalisation::prediction() {
        auto now = NUClear::clock::now();

        for (auto& tracked_robot : tracked_robots) {
            double dt =
                std::chrono::duration_cast<std::chrono::duration<double>>(now - tracked_robot.last_time_update).count();
            tracked_robot.last_time_update = now;
            tracked_robot.ukf.time(dt);
            tracked_robot.seen = false;
        }
    }

    void RobotLocalisation::data_association(const std::vector<Eigen::Vector3d>& robots_rRWw,
                                             const std::unique_ptr<Purpose>& purpose,
                                             const Eigen::Matrix2d& measurement_noise) {
        const bool use_noise_override = !measurement_noise.isZero();
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

                // If the robot is not in the list, check if a vision-detected robot is nearby to promote
                if (teammate_itr == tracked_robots.end()) {
                    // Find the closest non-teammate robot to the broadcast position
                    auto closest_itr =
                        std::min_element(tracked_robots.begin(),
                                         tracked_robots.end(),
                                         [&rRWw](const TrackedRobot& a, const TrackedRobot& b) {
                                             return (rRWw.head<2>() - a.get_rRWw()).norm()
                                                    < (rRWw.head<2>() - b.get_rRWw()).norm();
                                         });

                    // If a vision-detected robot is close enough, promote it to teammate
                    if (closest_itr != tracked_robots.end() && !closest_itr->teammate
                        && (rRWw.head<2>() - closest_itr->get_rRWw()).norm() < cfg.association_distance) {
                        closest_itr->teammate = true;
                        closest_itr->purpose  = *purpose;
                        closest_itr->seen     = true;
                    }
                    else {
                        tracked_robots.emplace_back(TrackedRobot(rRWw, cfg.ukf, next_id++, purpose));
                    }
                    continue;
                }
                // If the robot is in the list, update it with the new position
                teammate_itr->ukf.measure(
                    Eigen::Vector2d(rRWw.head<2>()),
                    use_noise_override ? measurement_noise : cfg.ukf.noise.measurement.position_robocup,
                    MeasurementType::ROBOT_POSITION());
                teammate_itr->seen    = true;
                teammate_itr->purpose = *purpose;

                continue;
            }

            // Get the closest robot we have to the given vision measurement
            auto closest_robot_itr =
                std::min_element(tracked_robots.begin(),
                                 tracked_robots.end(),
                                 [&rRWw](const TrackedRobot& a, const TrackedRobot& b) {
                                     // Get each robot's x-y 2D position in the world
                                     auto a_rRWw = a.get_rRWw();
                                     auto b_rRWw = b.get_rRWw();
                                     // Compare to see which is closer to the robot vision measurement position
                                     return (rRWw.head<2>() - a_rRWw).norm() < (rRWw.head<2>() - b_rRWw).norm();
                                 });
            double closest_distance = (rRWw.head<2>() - closest_robot_itr->get_rRWw()).norm();

            // If the closest robot is far enough away, add this as a new robot.
            // Indirect sightings (noise override) never create new robots — they only update existing ones.
            if (closest_distance > cfg.association_distance) {
                if (!use_noise_override) {
                    tracked_robots.emplace_back(TrackedRobot(rRWw, cfg.ukf, next_id++, purpose));
                }
                continue;
            }

            // For known teammates, skip the vision UKF update — their own broadcast is more accurate
            // than our visual sighting (which has our localisation error compounded in). Just mark
            // as seen so the track stays alive until the next broadcast arrives.
            if (closest_robot_itr->teammate && !use_noise_override) {
                closest_robot_itr->seen = true;
                continue;
            }

            // Otherwise update matched robot with the vision measurement
            closest_robot_itr->ukf.measure(
                Eigen::Vector2d(rRWw.head<2>()),
                use_noise_override ? measurement_noise : cfg.ukf.noise.measurement.position,
                MeasurementType::ROBOT_POSITION());
            closest_robot_itr->seen = true;
        }
    }

    void RobotLocalisation::maintenance(const GreenHorizon& horizon,
                                        const Field& field,
                                        const FieldDescription& field_desc) {
        std::vector<TrackedRobot> new_tracked_robots{};

        // Sort tracked_robots so that robots that are teammates are at the front to prevent teammates being removed
        std::sort(tracked_robots.begin(), tracked_robots.end(), [](const TrackedRobot& a, const TrackedRobot& b) {
            return a.purpose.player_id > b.purpose.player_id;
        });

        for (auto& tracked_robot : tracked_robots) {
            auto state = RobotModel<double>::StateVec(tracked_robot.ukf.get_state());

            // If a tracked robot has moved outside of view, keep it as seen so we don't lose it
            // A robot is outside of view if it is not within the green horizon
            // TODO (tom): It may be better to use fov and image size to determine if a robot should be seen
            if (!point_in_convex_hull(horizon.horizon, Eigen::Vector3d(state.rRWw.x(), state.rRWw.y(), 0))) {
                tracked_robot.seen = true;
            }

            // If the tracked robot has not been seen, increment the consecutively missed count
            tracked_robot.missed_count = tracked_robot.seen ? 0 : tracked_robot.missed_count + 1;

            // Don't add this robot if it has been missed too many times
            if (tracked_robot.missed_count > cfg.max_missed_count) {
                log<DEBUG>(fmt::format("Removing robot {} due to missed count", tracked_robot.id));
                continue;
            }

            // Check if this robot is too close to any kept robot
            if (std::any_of(new_tracked_robots.begin(), new_tracked_robots.end(), [&](const auto& other_robot) {
                    return (tracked_robot.get_rRWw() - other_robot.get_rRWw()).norm() < cfg.association_distance;
                })) {
                log<DEBUG>(fmt::format("Removing robot {} due to proximity", tracked_robot.id));
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

        // Deduplicate teammates by player_id — if two tracks share the same player_id, keep the stronger one
        std::vector<TrackedRobot> deduplicated{};
        for (auto& robot : new_tracked_robots) {
            if (!robot.teammate || robot.purpose.player_id == 0) {
                deduplicated.push_back(std::move(robot));
                continue;
            }
            auto existing = std::find_if(deduplicated.begin(), deduplicated.end(), [&robot](const TrackedRobot& r) {
                return r.teammate && r.purpose.player_id == robot.purpose.player_id;
            });
            if (existing == deduplicated.end()) {
                deduplicated.push_back(std::move(robot));
            }
            else if (robot.missed_count < existing->missed_count) {
                *existing = std::move(robot);
            }
        }
        new_tracked_robots = std::move(deduplicated);

        // Enforce max opponent count — sort opponents by missed_count ascending, remove weakest from tail
        int opponent_count =
            std::count_if(new_tracked_robots.begin(), new_tracked_robots.end(), [](const TrackedRobot& r) {
                return !r.teammate;
            });
        if (opponent_count > cfg.max_opponent_count) {
            // Stable sort: teammates first, then opponents strongest (lowest missed_count) first
            std::stable_sort(new_tracked_robots.begin(),
                             new_tracked_robots.end(),
                             [](const TrackedRobot& a, const TrackedRobot& b) {
                                 if (a.teammate != b.teammate) {
                                     return a.teammate > b.teammate;
                                 }
                                 return a.missed_count < b.missed_count;
                             });
            int excess = opponent_count - cfg.max_opponent_count;
            while (excess-- > 0 && !new_tracked_robots.empty() && !new_tracked_robots.back().teammate) {
                log<DEBUG>(fmt::format("Removing robot {} due to opponent count limit", new_tracked_robots.back().id));
                new_tracked_robots.pop_back();
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
