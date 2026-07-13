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
#include "RobotCommunication.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/format.h>

#include "extension/Configuration.hpp"

#include "message/behaviour/state/WalkState.hpp"
#include "message/input/GameState.hpp"
#include "message/input/Robocup.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/purpose/Purpose.hpp"
#include "message/skill/Kick.hpp"
#include "message/support/GlobalConfig.hpp"

#include "utility/math/euler.hpp"
namespace module::network {

    using extension::Configuration;
    using message::behaviour::state::WalkState;
    using message::input::GameState;
    using message::input::Message;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::planning::WalkToDebug;
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
                // Maximum message size and count
                cfg.max_message_size      = config["max_message_size"].as<uint>();
                cfg.max_messages_per_game = config["max_messages_per_game"].as<uint>();

                // Compute ports as 10000 + team_id unless overridden in config
                const uint configured_send_port = config["send_port"].as<uint>();
                cfg.send_port = (configured_send_port != 0) ? configured_send_port : 10000 + global_config.team_id;
                const uint configured_receive_port = config["receive_port"].as<uint>();
                const uint new_receive_port =
                    (configured_receive_port != 0) ? configured_receive_port : 10000 + global_config.team_id;

                // Need to determine broadcast ip
                cfg.broadcast_ip = config["broadcast_ip"].as<std::string>("");
                // Need to determine optional filtering packets
                cfg.udp_filter_address = config["udp_filter_address"].as<std::string>("");

                // If we are changing ports (the port starts at 0 so this should start it the first time)
                if (new_receive_port != cfg.receive_port) {
                    // If we have an old binding, then unbind it
                    // The port starts at 0 so this should work
                    if (cfg.receive_port != 0) {
                        listen_handle.unbind();
                    }

                    cfg.receive_port = new_receive_port;

                    // Bind our new handle
                    std::tie(listen_handle, std::ignore, std::ignore) =
                        on<UDP::Broadcast, Single>(cfg.receive_port).then([this, &global_config](const UDP::Packet& p) {
                            std::string remote_addr = p.remote.address;

                            // Apply filtering of packets if udp_filter_address is set in config
                            if (!cfg.udp_filter_address.empty() && remote_addr != cfg.udp_filter_address) {
                                if (std::find(ignored_ip_addresses.begin(), ignored_ip_addresses.end(), remote_addr)
                                    == ignored_ip_addresses.end()) {
                                    ignored_ip_addresses.insert(remote_addr);
                                    log<DEBUG>("Ignoring UDP packet from",
                                              remote_addr,
                                              "as it doesn't match configured filter address",
                                              cfg.udp_filter_address);
                                }

                                return;
                            }

                            // Deserialise the incoming RoboCup message
                            const std::vector<unsigned char>& payload = p.payload;
                            Message incoming_msg = NUClear::util::serialise::Serialise<Message>::deserialise(payload);

                            // Check if the incoming message is from the same player
                            bool own_player_message = global_config.player_id == incoming_msg.current_pose.player_id;

                            // Port-per-team ensures only teammates broadcast on this port
                            // Filter out messages from ourselves only
                            if (!own_player_message) {
                                log<DEBUG>("Message received from teammate ID",
                                          incoming_msg.current_pose.player_id,
                                          "position:",
                                          incoming_msg.current_pose.position.x(),
                                          incoming_msg.current_pose.position.y(),
                                          incoming_msg.current_pose.position.z(),
                                          "going for ball:",
                                          incoming_msg.going_for_ball);
                                emit(std::make_unique<Message>(std::move(incoming_msg)));
                            }
                        });
                }
            });

        on<Startup>().then([this] {
            // Delay the robot sending messages, to allow the robot to collect data and send reasonable information
            emit<Scope::DELAY>(std::make_unique<StartupDelay>(), std::chrono::seconds(cfg.startup_delay));
        });

        on<Trigger<GameState>>().then([this](const GameState& game_state) {
            // Reset message counter when a new game ends
            if (game_state.phase == GameState::Phase::FINISHED) {
                if (messages_sent > 0) {
                    log<INFO>("Game ended. Total messages sent:", messages_sent);
                    messages_sent = 0;
                }
            }
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
           Optional<With<WalkToDebug>>,
           With<GlobalConfig>>()
            .then([this](const std::shared_ptr<const Ball>& loc_ball,
                         const std::shared_ptr<const WalkState>& walk_state,
                         const std::shared_ptr<const Kick>& kick,
                         const std::shared_ptr<const Sensors>& sensors,
                         const std::shared_ptr<const Field>& field,
                         const std::shared_ptr<const GameState>& game_state,
                         const std::shared_ptr<const Purpose>& purpose,
                         const std::shared_ptr<const WalkToDebug>& walk_to,
                         const GlobalConfig& config) {
                auto msg = std::make_unique<Message>();

                // Timestamp
                msg->timestamp = NUClear::clock::now();

                // State
                // If there is game state information, then process
                if (game_state) {
                    // Penalty
                    if (game_state->self.penalty_reason == GameState::PenaltyReason::UNPENALISED) {
                        msg->state = message::input::State::UNPENALISED;
                    }
                    else {
                        msg->state = message::input::State::PENALISED;
                    }
                    // Team
                    msg->current_pose.team = message::input::Team::NUBOTS;
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
                        msg->current_pose.position = rTFf.cast<float>();
                        // The mixed-team protocol expects the other team's field x-axis convention, which is
                        // mirrored relative to ours, so flip x before sending
                        msg->current_pose.position.x() *= -1.0F;
                        msg->current_pose.position.z() = mat_to_rpy_intrinsic(Hft.rotation()).z();

                        msg->current_pose.covariance = field->covariance.cast<float>();
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
                    // The mixed-team protocol expects the other team's field x-axis convention, which is
                    // mirrored relative to ours, so flip x before sending
                    msg->target_pose.position.x() *= -1.0F;
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
                    // Convert position and velocity of ball from world to field space
                    if (field) {
                        Eigen::Isometry3d Hfw = Eigen::Isometry3d(field->Hfw);

                        // Store our position from field to ball
                        Eigen::Vector3d rBFf = Hfw * loc_ball->rBWw;
                        msg->ball.position   = rBFf.cast<float>();

                        // Rotate the ball's velocity from world frame into field frame (translation doesn't
                        // apply to a velocity, so only use the rotation part of Hfw)
                        Eigen::Vector3d vBFf = Hfw.linear() * loc_ball->vBw;
                        msg->ball.velocity   = vBFf.cast<float>();

                        // The mixed-team protocol expects the other team's field x-axis convention, which is
                        // mirrored relative to ours, so flip x before sending
                        msg->ball.position.x() *= -1.0F;
                        msg->ball.velocity.x() *= -1.0F;
                    }
                    msg->ball.covariance = loc_ball->covariance.block(0, 0, 3, 3).cast<float>();
                    // Age of the ball observation in seconds (-1 indicates invalid / do not rebroadcast teammate
                    // guesses)
                    if (loc_ball->confidence > 0.0) {
                        msg->ball.age =
                            std::chrono::duration<float>(NUClear::clock::now() - loc_ball->time_of_measurement).count();
                    }
                }

                // Purpose information, simple a bool in the new proto
                msg->going_for_ball = (purpose && purpose->purpose.value == SoccerPosition::ATTACK);

                // Single-line summary of the outgoing broadcast
                log<DEBUG>(fmt::format(
                    "Broadcast: id={} {} pose=({:.2f}, {:.2f}, {:.2f}) walk=({:.2f}, {:.2f}, {:.2f}) "
                    "target=({:.2f}, {:.2f}, {:.2f}) kick=({:.2f}, {:.2f}) ball=({:.2f}, {:.2f}) age={:.1f}s "
                    "going_for_ball={}",
                    msg->current_pose.player_id,
                    msg->state == message::input::State::PENALISED ? "PENALISED" : "UNPENALISED",
                    msg->current_pose.position.x(),
                    msg->current_pose.position.y(),
                    msg->current_pose.position.z(),
                    msg->walk_command.x(),
                    msg->walk_command.y(),
                    msg->walk_command.z(),
                    msg->target_pose.position.x(),
                    msg->target_pose.position.y(),
                    msg->target_pose.position.z(),
                    msg->kick_target.x(),
                    msg->kick_target.y(),
                    msg->ball.position.x(),
                    msg->ball.position.y(),
                    msg->ball.age,
                    msg->going_for_ball));

                // Check serialised size before sending
                auto payload = NUClear::util::serialise::Serialise<Message>::serialise(*msg);
                if (payload.size() > cfg.max_message_size) {
                    log<WARN>("RoboCup message size",
                              payload.size(),
                              "bytes exceeds",
                              cfg.max_message_size,
                              "byte limit");
                }
                else {
                    log<DEBUG>("RoboCup message size:", payload.size(), "bytes");
                }

                // Only count messages during Ready, Set, and Playing states
                if (game_state) {
                    const bool is_counting_state = game_state->phase == GameState::Phase::READY
                                                   || game_state->phase == GameState::Phase::SET
                                                   || game_state->phase == GameState::Phase::PLAYING;
                    if (is_counting_state) {
                        if (messages_sent >= cfg.max_messages_per_game) {
                            log<WARN>("Message budget exhausted, dropping packet");
                            return;
                        }
                        messages_sent++;
                        log<DEBUG>("Messages sent this game:", messages_sent, "/ ", cfg.max_messages_per_game);
                    }
                }


                emit<Scope::UDP>(msg, cfg.broadcast_ip, cfg.send_port);
            });
    }
}  // namespace module::network
