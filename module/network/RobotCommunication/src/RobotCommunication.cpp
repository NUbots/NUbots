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
#include "RobotCommunication.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "extension/Configuration.hpp"

#include "message/behaviour/state/WalkState.hpp"
#include "message/input/GameState.hpp"
#include "message/input/RoboCup.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/Robot.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/purpose/Purpose.hpp"
#include "message/skill/Kick.hpp"
#include "message/support/GlobalConfig.hpp"

#include "utility/math/euler.hpp"
namespace module::network {

    using extension::Configuration;
    using message::behaviour::state::WalkState;
    using message::input::GameState;
    using message::input::RoboCup;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::Robots;
    using message::planning::WalkTo;
    using message::purpose::Purpose;
    using message::purpose::SoccerPosition;
    using message::skill::Kick;
    using message::support::GlobalConfig;
    using utility::math::euler::mat_to_rpy_intrinsic;

    struct StartupDelay {};

    RobotCommunication::RobotCommunication(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration, Trigger<GlobalConfig>>("RobotCommunication.yaml")
            .then([this](const Configuration& config, const GlobalConfig& global_config) {
                // Use configuration here from file RobotCommunication.yaml
                log_level = config["log_level"].as<NUClear::LogLevel>();
                // Delay before sending messages
                cfg.startup_delay = config["startup_delay"].as<int>();
                // Ball timeout to use the ball position
                cfg.ball_timeout = std::chrono::seconds(config["ball_timeout"].as<int>());

                // Need to determine send and receive ports
                cfg.send_port = config["send_port"].as<uint>();
                // Need to determine broadcast ip
                cfg.broadcast_ip = config["broadcast_ip"].as<std::string>("");
                // Need to determine optional filtering packets
                cfg.udp_filter_address = config["udp_filter_address"].as<std::string>("");

                // If we are changing ports (the port starts at 0 so this should start it the first time)
                if (config["receive_port"].as<uint>() != cfg.receive_port) {
                    // If we have an old binding, then unbind it
                    // The port starts at 0 so this should work
                    if (cfg.receive_port != 0) {
                        listen_handle.unbind();
                    }

                    cfg.receive_port = config["receive_port"].as<uint>();

                    // Bind our new handle
                    std::tie(listen_handle, std::ignore, std::ignore) =
                        on<UDP::Broadcast, Optional<With<GameState>>, Single>(cfg.receive_port)
                            .then([this, &global_config](const UDP::Packet& p,
                                                         const std::shared_ptr<const GameState>& game_state) {
                                std::string remote_addr = p.remote.address;

                                // Apply filtering of packets if udp_filter_address is set in config
                                if (!cfg.udp_filter_address.empty() && remote_addr != cfg.udp_filter_address) {
                                    if (std::find(ignored_ip_addresses.begin(), ignored_ip_addresses.end(), remote_addr)
                                        == ignored_ip_addresses.end()) {
                                        ignored_ip_addresses.insert(remote_addr);
                                        log<INFO>("Ignoring UDP packet from",
                                                  remote_addr,
                                                  "as it doesn't match configured filter address",
                                                  cfg.udp_filter_address);
                                    }

                                    return;
                                }

                                // Deserialise the incoming RoboCup message
                                const std::vector<unsigned char>& payload = p.payload;
                                RoboCup incoming_msg =
                                    NUClear::util::serialise::Serialise<RoboCup>::deserialise(payload);

                                // Check if the incoming message is from the same player
                                bool own_player_message =
                                    global_config.player_id == incoming_msg.current_pose.player_id;

                                // If there is game state information, get the colour
                                message::input::Team team_colour = message::input::Team::UNKNOWN_TEAM;
                                if (game_state) {
                                    switch (int(game_state->team.team_colour)) {
                                        case GameState::TeamColour::BLUE:
                                            team_colour = message::input::Team::BLUE;
                                            break;
                                        case GameState::TeamColour::RED: team_colour = message::input::Team::RED; break;
                                        default: team_colour = message::input::Team::UNKNOWN_TEAM;
                                    }
                                }

                                // Check if the incoming message is from the same team
                                bool own_team_message = team_colour == incoming_msg.current_pose.team;

                                // Filter out messages from ourselves and from other teams
                                if (!own_player_message && own_team_message) {
                                    emit(std::make_unique<RoboCup>(std::move(incoming_msg)));
                                }
                            });
                }
            });

        on<Startup>().then([this] {
            // Delay the robot sending messages, to allow the robot to collect data and send reasonable information
            emit<Scope::DELAY>(std::make_unique<StartupDelay>(), std::chrono::seconds(cfg.startup_delay));
        });

        on<Every<2, Per<std::chrono::seconds>>,
           With<StartupDelay>,
           Optional<With<Ball>>,
           Optional<With<WalkState>>,
           Optional<With<Kick>>,
           Optional<With<Sensors>>,
           Optional<With<Field>>,
           Optional<With<GameState>>,
           Optional<With<Purpose>>,
           Optional<With<WalkTo>>,
           Optional<With<Robots>>,
           With<GlobalConfig>>()
            .then([this](const std::shared_ptr<const Ball>& loc_ball,
                         const std::shared_ptr<const WalkState>& walk_state,
                         const std::shared_ptr<const Kick>& kick,
                         const std::shared_ptr<const Sensors>& sensors,
                         const std::shared_ptr<const Field>& field,
                         const std::shared_ptr<const GameState>& game_state,
                         const std::shared_ptr<const Purpose>& purpose,
                         const std::shared_ptr<const WalkTo>& walk_to,
                         const std::shared_ptr<const Robots>& robot_localisation,
                         const GlobalConfig& config) {
                auto msg = std::make_unique<RoboCup>();

                // Timestamp
                msg->timestamp = NUClear::clock::now();

                // State
                // If there is game state information, then process
                if (game_state) {
                    int penalty_reason = game_state->self.penalty_reason;
                    switch (penalty_reason) {
                        case 0: msg->state = 0; break;
                        case 1: msg->state = 1; break;
                        default: msg->state = 2; break;
                    }

                    // Team colour
                    switch (int(game_state->team.team_colour)) {
                        case GameState::TeamColour::BLUE: msg->current_pose.team = message::input::Team::BLUE; break;
                        case GameState::TeamColour::RED: msg->current_pose.team = message::input::Team::RED; break;
                        default: msg->current_pose.team = message::input::Team::UNKNOWN_TEAM;
                    }
                }

                // Current pose (Position, orientation, and covariance of the player on the field)
                msg->current_pose.player_id = config.player_id;

                if (sensors) {
                    // Get our world transform
                    Eigen::Isometry3d Htw(sensors->Htw);

                    // If we have field information
                    if (field) {
                        // Transform the field state into Hfw
                        Eigen::Isometry3d Hfw = Eigen::Isometry3d(field->Hfw);

                        // Get our torso in field space
                        Eigen::Isometry3d Hft = Hfw * Htw.inverse();
                        Eigen::Vector3d rTFf  = Hft.translation();

                        // Store our position from field to torso
                        msg->current_pose.position     = rTFf.cast<float>();
                        msg->current_pose.position.z() = mat_to_rpy_intrinsic(Hft.rotation()).z();

                        msg->current_pose.covariance = field->covariance.cast<float>();
                        msg->current_pose.cost       = field->cost;
                    }
                }


                // Walk command
                if (walk_state != nullptr) {
                    msg->walk_command = walk_state->velocity_target.cast<float>();
                }

                // Target pose (Position and orientation of the players target on the field specified)
                if (walk_to && sensors && field) {
                    // Get target pose in field coords (Hfr * Hrd = Hfd)
                    Eigen::Isometry3d Hfd = (field->Hfw * sensors->Hrw.inverse()) * walk_to->Hrd;

                    // Extract 3d translation
                    Eigen::Vector3d rDFf = Hfd.translation();
                    // Store position
                    msg->target_pose.position = rDFf.cast<float>();
                    // Extract yaw from rotation matrix
                    msg->target_pose.position.z() = mat_to_rpy_intrinsic(Hfd.rotation()).z();
                    // Copy team and player ID to target pose
                    msg->target_pose.team      = msg->current_pose.team;
                    msg->target_pose.player_id = config.player_id;
                }

                // Kick target
                if (kick) {
                    // take x and y components of vec3 to convert to fvec2
                    msg->kick_target.x() = kick->target.x();
                    msg->kick_target.y() = kick->target.y();
                }

                // Ball information
                // Check if the ball has been seen recently, if it hasn't the message fields will 0
                // and the confidence will be 0
                if (loc_ball && (NUClear::clock::now() - loc_ball->time_of_measurement < cfg.ball_timeout)) {
                    // Convert position of ball from world to field space
                    if (field) {
                        Eigen::Vector3d rBWw = loc_ball->rBWw;
                        // Transform the field state into Hfw
                        Eigen::Isometry3d Hfw = Eigen::Isometry3d(field->Hfw);
                        Eigen::Vector3d rBFf  = Hfw * rBWw;
                        // Store our position from field to ball
                        msg->ball.position = rBFf.cast<float>();
                    }
                    // Confidence - our own estimates are 1.0, while if it's from a teammate, we have no confidence
                    // This it to prevent everyone echoing
                    msg->ball.confidence = loc_ball->confidence;
                    msg->ball.covariance = loc_ball->covariance.block(0, 0, 3, 3).cast<float>();

                    msg->ball.velocity = (loc_ball->vBw).cast<float>();
                }

                // Where the robot thinks the other robots are.
                if (robot_localisation && field) {

                    // Iterate through robots detected by localisation
                    for (const auto& local_bot : robot_localisation->robots) {
                        // Create new robot message
                        message::input::Robot rc_robot;

                        // Assign player ID from purpose
                        rc_robot.player_id = local_bot.purpose.player_id;

                        // Convert world to field coords
                        Eigen::Vector3d rRFf = field->Hfw * local_bot.rRWw;

                        // Store 2D position (x, y) and set (z) to 0
                        rc_robot.position     = rRFf.cast<float>();
                        rc_robot.position.z() = 0.0f;

                        // Check if covariance matrix is valid
                        // Initialise with zeros
                        rc_robot.covariance = Eigen::Matrix3f::Zero();
                        if (local_bot.covariance.rows() >= 2 && local_bot.covariance.cols() >= 2) {
                            rc_robot.covariance.block<2, 2>(0, 0) =
                                local_bot.covariance.block<2, 2>(0, 0).cast<float>();
                        }

                        if (local_bot.teammate) {
                            rc_robot.team = msg->current_pose.team;
                        }
                        else {
                            rc_robot.team = msg->current_pose.team == message::input::Team::BLUE
                                                ? message::input::Team::RED
                                                : message::input::Team::BLUE;
                        }

                        // Add robot information to list of other robots in message
                        msg->others.push_back(std::move(rc_robot));
                    }
                }


                // Current purpose (soccer position) of the Robot
                if (purpose) {
                    msg->purpose = *purpose;
                }
                // Override the player ID in the purpose message to be consistent, and in case purpose is not set
                msg->purpose.player_id = config.player_id;

                emit<Scope::UDP>(msg, cfg.broadcast_ip, cfg.send_port);
            });
    }
}  // namespace module::network
