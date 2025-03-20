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
#include "message/purpose/Purpose.hpp"
#include "message/skill/Kick.hpp"
#include "message/strategy/TeamMates.hpp"
#include "message/support/GlobalConfig.hpp"

#include "utility/math/euler.hpp"
#include "utility/strategy/soccer_strategy.hpp"


namespace module::network {

    // add usings
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
    using message::strategy::TeamMate;
    using message::strategy::TeamMates;
    using message::support::GlobalConfig;
    using utility::math::euler::mat_to_rpy_intrinsic;

    RobotCommunication::RobotCommunication(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration, Trigger<GlobalConfig>>("RobotCommunication.yaml")
            .then([this](const Configuration& config, const GlobalConfig& global_config) {
                // Use configuration here from file RobotCommunication.yaml
                log_level = config["log_level"].as<NUClear::LogLevel>();

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
                        on<UDP::Broadcast, Single>(cfg.receive_port).then([this, &global_config](const UDP::Packet& p) {
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

                            const std::vector<unsigned char>& payload = p.payload;
                            RoboCup incoming_msg = NUClear::util::serialise::Serialise<RoboCup>::deserialise(payload);

                            // filter out own messages
                            if (global_config.player_id != incoming_msg.current_pose.player_id) {
                                emit(std::make_unique<RoboCup>(std::move(incoming_msg)));
                            }
                        });
                }

                emit(std::unique_ptr<TeamMates>());
            });

        // include teammates and add a using teammates
        on<Trigger<RoboCup>, With<TeamMates>>().then([this](const RoboCup& robocup, const TeamMates& old_teammates) {
            // Get the id of this robot
            int id = robocup.current_pose.player_id;

            // Make new TeamMates message using data from the old message
            TeamMates teammates = old_teammates;

            // Find if the teammates vector (repeated TeamMate) has ID
            //...
            // for (mate in teammates) if mate.id==id then has id
            //...
            bool found = false;

            for (auto& mate : teammates.teammates) {  // Loop through each teammate.
                if (mate.id == id) {  // If the ID of a teammate is found to be the same, update the position.
                    mate.rRFf = robocup.current_pose.position;  // Update position
                    found     = true;
                    break;  // Break out of the loop.
                }
            }
            // Else this robot is not in the list of team mates already, add it in
            if (!found) {
                // Make the mate
                TeamMate mate;
                // Set the variables
                mate.id   = id;
                mate.rRFf = robocup.current_pose.position;
                // Add it to the teammates list
                teammates.teammates.emplace_back(mate);
            }

            emit(std::make_unique<TeamMates>(std::move(teammates)));
        });

        on<Every<2, Per<std::chrono::seconds>>,
           Optional<With<Ball>>,
           Optional<With<WalkState>>,
           Optional<With<Kick>>,
           Optional<With<Sensors>>,
           Optional<With<Field>>,
           Optional<With<GameState>>,
           Optional<With<Purpose>>,
           Optional<With<GlobalConfig>>>()
            .then([this](const std::shared_ptr<const Ball>& loc_ball,
                         const std::shared_ptr<const WalkState>& walk_state,
                         const std::shared_ptr<const Kick>& kick,
                         const std::shared_ptr<const Sensors>& sensors,
                         const std::shared_ptr<const Field>& field,
                         const std::shared_ptr<const GameState>& game_state,
                         const std::shared_ptr<const Purpose>& purpose,
                         const std::shared_ptr<const GlobalConfig>& config) {
                auto msg = std::make_unique<RoboCup>();

                // Timestamp
                msg->timestamp = NUClear::clock::now();

                // State
                // If there is game state information, then process
                if (game_state) {
                    int penalty_reason = game_state->data.self.penalty_reason;
                    switch (penalty_reason) {
                        case 0: msg->state = 0; break;
                        case 1: msg->state = 1; break;
                        default: msg->state = 2; break;
                    }
                }

                // Current pose (Position, orientation, and covariance of the player on the field)
                if (config) {
                    msg->current_pose.player_id = config->player_id;

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

                            msg->current_pose.team = config->team_id;
                        }
                    }
                }

                // Walk command
                if (walk_state != nullptr) {
                    msg->walk_command = walk_state->velocity_target.cast<float>();
                }

                // TODO: target pose (Position and orientation of the players target on the field specified)

                // Kick target
                if (kick) {
                    // take x and y components of vec3 to convert to fvec2
                    msg->kick_target.x() = kick->target.x();
                    msg->kick_target.y() = kick->target.y();
                }

                // Ball
                if (loc_ball) {
                    // convert position of ball from world to field space
                    if (field) {
                        Eigen::Vector3d rBWw = loc_ball->rBWw;
                        // Transform the field state into Hfw
                        Eigen::Isometry3d Hfw = Eigen::Isometry3d(field->Hfw);
                        Eigen::Vector3d rBFf  = Hfw * rBWw;
                        // Store our position from field to ball
                        msg->ball.position = rBFf.cast<float>();
                    }

                    msg->ball.covariance = loc_ball->covariance.block(0, 0, 3, 3).cast<float>();

                    msg->ball.velocity = (loc_ball->vBw).cast<float>();
                }

                // TODO: Robots. Where the robot thinks the other robots are. This doesn't exist yet.

                // Current purpose (soccer position) of the Robot
                if (purpose) {
                    msg->purpose = *purpose;
                }

                emit<Scope::UDP>(msg, cfg.broadcast_ip, cfg.send_port);
            });
    }
}  // namespace module::network
