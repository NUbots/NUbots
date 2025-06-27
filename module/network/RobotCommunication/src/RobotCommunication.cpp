/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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
           With<GlobalConfig>>()
            .then([this](const std::shared_ptr<const Ball>& loc_ball,
                         const std::shared_ptr<const WalkState>& walk_state,
                         const std::shared_ptr<const Kick>& kick,
                         const std::shared_ptr<const Sensors>& sensors,
                         const std::shared_ptr<const Field>& field,
                         const std::shared_ptr<const GameState>& game_state,
                         const std::shared_ptr<const Purpose>& purpose,
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
                        // msg->current_pose.cost       = field->cost;
                    }
                }


                // Walk command
                if (walk_state != nullptr) {
                    msg->walk_command = walk_state->velocity_target.cast<float>();
                }

                // Target pose (Position and orientation of the players target on the field specified)
                // Flag (true for using lab field, false for robocup field)
                bool use_lab = false;

                // Robocup field dimensions
                const float robocup_field_length = 9.0f;
                const float robocup_field_width  = 6.0f;

                // Lab field dimensions
                const float lab_field_length = 6.8f;
                const float lab_field_width  = 5.0f;

                // Pick field size to use
                const float field_length = use_lab ? lab_field_length : robocup_field_length;
                const float field_width  = use_lab ? lab_field_width : robocup_field_width;


                // Goal positions x-axis (centered on y=0)
                Eigen::Vector2f own_goal(-field_length / 2.0f, 0.0f);
                Eigen::Vector2f opponent_goal(field_length / 2.0f, 0.0f);
                // Extract the current robot and ball positions
                Eigen::Vector2f robot_position(msg->current_pose.position.x(), msg->current_pose.position.y());
                Eigen::Vector2f ball_position(msg->ball.position.x(), msg->ball.position.y());
                // Initialize target position and angle
                Eigen::Vector2f target_position = Eigen::Vector2f::Zero();
                float target_theta              = 0.0f;

                // Compute angle from point 'from' to point 'to'. Useful to set robot's orientation facing
                // toward some position
                auto calculate_angle = [](const Eigen::Vector2f& from, const Eigen::Vector2f& to) {
                    return std::atan2(to.y() - from.y(), to.x() - from.x());
                };

                // Determines target position (where the robot should move) and target theta (what direction
                // the robot should face) based on assigned role
                if (purpose) {
                    // Extract SoccerPosition enum value once
                    SoccerPosition soccer_position = static_cast<SoccerPosition>(msg->purpose.purpose);

                    switch (static_cast<int>(soccer_position)) {  // cast to int to avoid ambiguity
                        case static_cast<int>(SoccerPosition::ATTACK):
                            target_position = ball_position - Eigen::Vector2f(0.5f, 0.0f);
                            target_theta    = calculate_angle(target_position, opponent_goal);
                            break;

                        case static_cast<int>(SoccerPosition::DEFEND):
                            target_position = (ball_position + own_goal) / 2.0f;
                            target_theta    = calculate_angle(target_position, ball_position);
                            break;

                        case static_cast<int>(SoccerPosition::GOALIE):
                            target_position = own_goal + Eigen::Vector2f(0.0f, 0.5f * ball_position.y());
                            target_theta    = calculate_angle(target_position, ball_position);
                            break;

                        case static_cast<int>(SoccerPosition::ALL_ROUNDER): {
                            target_position     = ball_position - Eigen::Vector2f(0.3f, 0.0f);
                            float distance_goal = (target_position - opponent_goal).norm();
                            float distance_ball = (target_position - ball_position).norm();
                            target_theta        = (distance_goal < distance_ball)
                                                      ? calculate_angle(target_position, opponent_goal)
                                                      : calculate_angle(target_position, ball_position);
                            break;
                        }

                        default:
                            target_position = robot_position;
                            target_theta    = calculate_angle(robot_position, ball_position);
                            break;
                    }  // end switch
                }
                // Update message with target pose
                msg->target_pose.player_id    = config.player_id;
                msg->target_pose.position.x() = target_position.x();
                msg->target_pose.position.y() = target_position.y();
                msg->target_pose.position.z() = target_theta;
                msg->target_pose.team         = msg->current_pose.team;

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


                    // No confidence field. msg->ball.confidence = loc_ball->confidence;


                    msg->ball.covariance = loc_ball->covariance.block(0, 0, 3, 3).cast<float>();

                    msg->ball.velocity = (loc_ball->vBw).cast<float>();
                }

                // TODO: Robots. Where the robot thinks the other robots are. This doesn't exist yet.
                // Get own robot pose transform (world to robot)
                // my_pose_transform = Sensors.Hrw

                // Get field transform
                // field_transform = Field.Hfw

                // Get detected robots (from vision system)
                // detected_robots = getDetectedRobots()

                // Convert detected robot positions
                // other_robots_list = empty list
                // for each detected_robot in detected_robots
                // detected_robot.position is in sensor/camera coordinates

                // Convert sensor coordinates to world coordinates
                // world_position = my_pose_transform * detected_robot.position

                // Convert world coordinates to field coordinates
                // field_position = field_transform * world_position

                // Create new robot message
                // robot_msg = new Robot()

                // Set detected robot ID or 0 if unknown
                // robot_msg.player_id = detected_robot.id or 0

                // Set position (x, y, theta) in field drame
                // robot_msg.position = field_position

                // Set team or UNKNOWN
                // robot_msg.team = detected_robot.team or UNKNOWN

                // Add robot message to list
                // other_robots_list.append(robot_msg)


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
