/*
 * MIT License
 *
 * Copyright (c) 2014 NUbots
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

#include "GameController.hpp"

#include <atomic>

#include "extension/Configuration.hpp"

#include "message/platform/RawSensors.hpp"
#include "message/support/GlobalConfig.hpp"

namespace module::input {

    using extension::Configuration;
    using gamecontroller::GameControllerPacket;
    using gamecontroller::GameControllerReplyPacket;
    using gamecontroller::ReplyMessage;
    using gamecontroller::Team;
    using message::input::GameEvents;
    using message::input::GameState;
    using message::support::GlobalConfig;
    using TeamColour      = message::input::GameEvents::TeamColour::Value;
    using Score           = GameEvents::Score;
    using GoalScored      = GameEvents::GoalScored;
    using Penalisation    = GameEvents::Penalisation;
    using Unpenalisation  = GameEvents::Unpenalisation;
    using Coach_message   = GameEvents::CoachMessage;
    using HalfTime        = GameEvents::HalfTime;
    using BallKickedOut   = GameEvents::BallKickedOut;
    using KickOffTeam     = GameEvents::KickOffTeam;
    using GamePhase       = GameEvents::GamePhase;
    using GameMode        = GameEvents::GameMode;
    using PenaltyReason   = GameState::Data::PenaltyReason;
    using TeamColourEvent = message::input::GameEvents::TeamColour;

    GameController::GameController(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), receive_port(0), send_port(0), TEAM_ID(0), PLAYER_ID(0), packet(), mode() {

        // Configure
        on<Configuration, Trigger<GlobalConfig>>("GameController.yaml")
            .then("GameController Configuration",
                  [this](const Configuration& config, const GlobalConfig& global_config) {
                      log_level = config["log_level"].as<NUClear::LogLevel>();

                      PLAYER_ID = global_config.player_id;
                      TEAM_ID   = global_config.team_id;
                      send_port = config["send_port"].as<uint>();

                      udp_filter_address = config["udp_filter_address"].as<std::string>("");

                      // If we are changing ports (the port starts at 0 so this should start it the first time)
                      if (config["receive_port"].as<uint>() != receive_port) {

                          // If we have an old binding, then unbind it
                          // The port starts at 0 so this should work
                          if (receive_port != 0) {
                              listen_handle.unbind();
                          }

                          // Update our port
                          receive_port = config["receive_port"].as<uint>();

                          // Bind our new handle
                          std::tie(listen_handle, std::ignore, std::ignore) =
                              on<UDP::Broadcast, With<GameState>, Single>(receive_port)
                                  .then([this](const UDP::Packet& p, const GameState& game_state) {
                                      std::string remote_addr = p.remote.address;

                                      // Apply filtering of packets if udp_filter_address is set in config
                                      if (!udp_filter_address.empty() && remote_addr != udp_filter_address) {
                                          if (std::find(ignored_ip_addresses.begin(),
                                                        ignored_ip_addresses.end(),
                                                        remote_addr)
                                              == ignored_ip_addresses.end()) {
                                              ignored_ip_addresses.insert(remote_addr);
                                              log<INFO>("Ignoring UDP packet from",
                                                        remote_addr,
                                                        "as it doesn't match configured filter address",
                                                        udp_filter_address);
                                          }

                                          return;
                                      }

                                      // Get our packet contents
                                      const GameControllerPacket& new_packet =
                                          *reinterpret_cast<const GameControllerPacket*>(p.payload.data());

                                      // Get the IP we are getting this packet from
                                      // Store it and use it to send back to the game controller using emit UDP
                                      BROADCAST_IP = p.local.address;

                                      if (new_packet.version == SUPPORTED_VERSION) {
                                          try {
                                              process(game_state, packet, new_packet);
                                          }
                                          catch (std::runtime_error& err) {
                                              log(err.what());
                                          }

                                          packet = new_packet;
                                      }
                                  });
                      }

                      reset_state();
                  });

        on<Every<2, Per<std::chrono::seconds>>>().then("GameController Reply",
                                                       [this] { send_reply_message(ReplyMessage::ALIVE); });
    }

    void GameController::send_reply_message(const ReplyMessage& message) {
        auto packet     = std::make_unique<GameControllerReplyPacket>();
        packet->header  = {gamecontroller::RETURN_HEADER[0],
                           gamecontroller::RETURN_HEADER[1],
                           gamecontroller::RETURN_HEADER[2],
                           gamecontroller::RETURN_HEADER[3]};
        packet->version = gamecontroller::RETURN_VERSION;
        packet->team    = TEAM_ID;
        packet->player  = PLAYER_ID;
        packet->message = message;

        emit<Scope::UDP>(packet, BROADCAST_IP, send_port);
    }

    void GameController::reset_state() {

        mode = static_cast<gamecontroller::Mode>(-1);

        std::copy(std::begin(gamecontroller::RECEIVE_HEADER),
                  std::end(gamecontroller::RECEIVE_HEADER),
                  std::begin(packet.header));
        packet.version          = SUPPORTED_VERSION;
        packet.packet_number    = 0;
        packet.players_per_team = PLAYERS_PER_TEAM;
        packet.state            = static_cast<gamecontroller::State>(-1);
        packet.first_half       = true;
        packet.kick_off_team    = -1;
        packet.mode             = static_cast<gamecontroller::Mode>(-1);
        packet.drop_in_team     = static_cast<gamecontroller::TeamColour>(-1);
        packet.drop_in_time     = -1;
        packet.secs_remaining   = 0;
        packet.secondary_time   = 0;
        for (uint i = 0; i < NUM_TEAMS; i++) {
            auto& own_team        = packet.teams[i];
            own_team.team_id      = (i == 0 ? TEAM_ID : 0);
            own_team.team_colour  = static_cast<gamecontroller::TeamColour>(-1);
            own_team.score        = 0;
            own_team.penalty_shot = 0;
            own_team.single_shots = 0;
            own_team.coach_message.fill(0);
            auto& coach               = own_team.coach;
            coach.penalty_state       = gamecontroller::PenaltyState::UNPENALISED;
            coach.penalised_time_left = 0;
            for (uint i = 0; i < gamecontroller::MAX_NUM_PLAYERS; i++) {
                auto& player = own_team.players[i];
                if (i <= ACTIVE_PLAYERS_PER_TEAM) {
                    player.penalty_state       = gamecontroller::PenaltyState::UNPENALISED;
                    player.penalised_time_left = 0;
                }
                else {
                    player.penalty_state       = gamecontroller::PenaltyState::SUBSTITUTE;
                    player.penalised_time_left = 0;
                }
            }
        }

        auto initial_state = std::make_unique<GameState>();
        // default to reasonable values for initial state
        initial_state->data.phase            = GameState::Data::Phase::INITIAL;
        initial_state->data.mode             = GameState::Data::Mode::NORMAL;
        initial_state->data.first_half       = true;
        initial_state->data.kicked_out_by_us = false;
        initial_state->data.our_kick_off     = false;

        initial_state->data.team.team_id     = TEAM_ID;
        initial_state->data.opponent.team_id = 0;

        emit(std::move(initial_state));
        emit(std::make_unique<GameState::Data::Phase>(GameState::Data::Phase::INITIAL));
    }

    void GameController::process(const GameState& old_game_state,
                                 const GameControllerPacket& old_packet,
                                 const GameControllerPacket& new_packet) {

        auto state = std::make_unique<GameState>(old_game_state);

        std::vector<std::function<void()>> state_changes;

        // game score
        const auto& old_own_team = get_own_team(old_packet);
        const auto& new_own_team = get_own_team(new_packet);

        const auto& old_opponent_team = get_opponent_team(old_packet);
        const auto& new_opponent_team = get_opponent_team(new_packet);

        /*******************************************************************************************
         * Process score updates
         ******************************************************************************************/
        if (old_own_team.score != new_own_team.score || old_opponent_team.score != new_opponent_team.score) {
            // score update
            emit(std::make_unique<GameEvents::Score>(GameEvents::Score{new_own_team.score, new_opponent_team.score}));


            if (old_own_team.score < new_own_team.score) {
                // we scored! :D

                // Set the team scores in the state packet
                state->data.team.score = new_own_team.score;
                state_changes.emplace_back([this, new_own_team] {
                    emit(
                        std::make_unique<GoalScored>(GoalScored{GameEvents::Context::Value::TEAM, new_own_team.score}));
                });
            }

            if (old_opponent_team.score < new_opponent_team.score) {
                // they scored :( boo

                // Set the team scores in the state packet
                state->data.opponent.score = new_opponent_team.score;
                state_changes.emplace_back([this, new_opponent_team] {
                    emit(std::make_unique<GoalScored>(
                        GoalScored{GameEvents::Context::Value::OPPONENT, new_opponent_team.score}));
                });
            }
        }

        /*******************************************************************************************
         * Process penalty updates
         ******************************************************************************************/
        // Clear our player state (easier to just rebuild)
        state->data.team.players.clear();
        state->data.opponent.players.clear();

        // Note: assumes players_per_team never changes
        for (uint i = 0; i < new_packet.players_per_team; i++) {
            uint player_id             = i + 1;
            const auto& old_own_player = old_own_team.players[i];
            const auto& new_own_player = new_own_team.players[i];

            const auto& old_opponent_player = old_opponent_team.players[i];
            const auto& new_opponent_player = new_opponent_team.players[i];

            // Update our state
            GameState::Data::Robot own_player = GameState::Data::Robot(
                player_id,
                get_penalty_reason(new_own_player.penalty_state),
                NUClear::clock::now() + std::chrono::seconds(new_own_player.penalised_time_left));
            state->data.team.players.push_back(own_player);
            if (player_id == PLAYER_ID) {
                state->data.self = own_player;
            }

            state->data.opponent.players.emplace_back(
                player_id,
                get_penalty_reason(new_opponent_player.penalty_state),
                NUClear::clock::now() + std::chrono::seconds(new_opponent_player.penalised_time_left));

            // Check if player on own team is penalised
            if (new_own_player.penalty_state != old_own_player.penalty_state
                && new_own_player.penalty_state != gamecontroller::PenaltyState::UNPENALISED) {

                auto unpenalised_time =
                    NUClear::clock::now() + std::chrono::seconds(new_own_player.penalised_time_left);
                auto reason = get_penalty_reason(new_own_player.penalty_state);
                state_changes.emplace_back([this, player_id, unpenalised_time, reason] {
                    if (player_id == PLAYER_ID) {
                        // Self penalised :@
                        emit(std::make_unique<Penalisation>(
                            Penalisation{GameEvents::Context::Value::SELF, player_id, unpenalised_time, reason}));
                        send_reply_message(ReplyMessage::PENALISED);
                        self_penalised = true;
                    }
                    else {
                        // Team mate penalised :'(
                        emit(std::make_unique<Penalisation>(
                            Penalisation{GameEvents::Context::Value::TEAM, player_id, unpenalised_time, reason}));
                    }
                });
            }
            else if (new_own_player.penalty_state == gamecontroller::PenaltyState::UNPENALISED
                     && old_own_player.penalty_state != gamecontroller::PenaltyState::UNPENALISED) {
                state_changes.emplace_back([this, player_id] {
                    if (player_id == PLAYER_ID) {
                        // Self unpenalised :)
                        emit(std::make_unique<Unpenalisation>(
                            Unpenalisation{GameEvents::Context::Value::SELF, player_id}));
                        send_reply_message(ReplyMessage::UNPENALISED);
                        self_penalised = false;
                    }
                    else {
                        // Team mate unpenalised :)
                        emit(std::make_unique<Unpenalisation>(
                            Unpenalisation{GameEvents::Context::Value::TEAM, player_id}));
                    }
                });
            }

            if (new_opponent_player.penalty_state != old_opponent_player.penalty_state
                && new_opponent_player.penalty_state != gamecontroller::PenaltyState::UNPENALISED) {
                auto unpenalised_time =
                    NUClear::clock::now() + std::chrono::seconds(new_opponent_player.penalised_time_left);
                auto reason = get_penalty_reason(new_opponent_player.penalty_state);
                // Opponent penalised :D
                state_changes.emplace_back([this, player_id, unpenalised_time, reason] {
                    emit(std::make_unique<Penalisation>(
                        Penalisation{GameEvents::Context::Value::OPPONENT, player_id, unpenalised_time, reason}));
                });
            }
            else if (new_opponent_player.penalty_state == gamecontroller::PenaltyState::UNPENALISED
                     && old_opponent_player.penalty_state != gamecontroller::PenaltyState::UNPENALISED) {
                // Opponent unpenalised D:
                state_changes.emplace_back([this, player_id] {
                    emit(std::make_unique<Unpenalisation>(
                        Unpenalisation{GameEvents::Context::Value::OPPONENT, player_id}));
                });
            }
        }


        /*******************************************************************************************
         * Process coach messages
         ******************************************************************************************/
        if (old_own_team.coach_message != new_own_team.coach_message) {

            // Update the coach message in the state
            state->data.team.coach_message = new_own_team.coach_message.data();

            // Listen to the coach? o_O
            state_changes.emplace_back([this, new_own_team] {
                emit(std::make_unique<Coach_message>(
                    Coach_message{GameEvents::Context::Value::TEAM, new_own_team.coach_message.data()}));
            });
        }

        if (old_opponent_team.coach_message != new_opponent_team.coach_message) {

            // Update the opponent coach message in the state
            state->data.opponent.coach_message = new_opponent_team.coach_message.data();

            // Listen in on the enemy! >:D
            state_changes.emplace_back([this, new_opponent_team] {
                emit(std::make_unique<Coach_message>(
                    Coach_message{GameEvents::Context::Value::OPPONENT, new_opponent_team.coach_message.data()}));
            });
        }


        /*******************************************************************************************
         * Process half changes
         ******************************************************************************************/
        if (old_packet.first_half != new_packet.first_half) {

            // Update the half time in the state
            state->data.first_half = new_packet.first_half;

            // Half time
            state_changes.emplace_back(
                [this, new_packet] { emit(std::make_unique<HalfTime>(HalfTime{new_packet.first_half})); });
        }

        /*******************************************************************************************
         * Process ball kicked out
         ******************************************************************************************/
        // Woo boolean algebra!
        if ((new_packet.drop_in_time != -1
             && ((new_packet.drop_in_time < old_packet.drop_in_time) == (old_packet.drop_in_time != -1)))
            || new_packet.drop_in_team != old_packet.drop_in_team) {

            // Ball was kicked out by drop_in_team
            auto time = NUClear::clock::now() - std::chrono::seconds(new_packet.drop_in_time);

            // Update the ball kicked out time and player in the state
            state->data.kicked_out_by_us = new_packet.drop_in_team == new_own_team.team_colour;
            state->data.kicked_out_time  = time;

            if (new_packet.drop_in_team == new_own_team.team_colour) {
                // We kicked the ball out :S
                state_changes.emplace_back([this, time] {
                    emit(std::make_unique<BallKickedOut>(BallKickedOut{GameEvents::Context::Value::TEAM, time}));
                });
            }
            else {
                // They kicked the ball out! ^_^
                state_changes.emplace_back([this, time] {
                    emit(std::make_unique<BallKickedOut>(BallKickedOut{GameEvents::Context::Value::OPPONENT, time}));
                });
            }
        }


        /*******************************************************************************************
         * Process kick off team
         ******************************************************************************************/
        if (old_packet.kick_off_team != new_packet.kick_off_team) {

            // Update the kickoff team (us or them)
            state->data.our_kick_off = new_packet.kick_off_team == new_own_team.team_id;

            // Get the new kick off team to emit the team change
            GameEvents::Context team = new_packet.kick_off_team == new_own_team.team_id
                                           ? GameEvents::Context::Value::TEAM
                                           : GameEvents::Context::Value::OPPONENT;

            state_changes.emplace_back([this, team] { emit(std::make_unique<KickOffTeam>(KickOffTeam{team})); });
        }


        /*******************************************************************************************
         * Process our team colour
         ******************************************************************************************/
        if (old_own_team.team_colour != new_own_team.team_colour) {
            TeamColour colour =
                new_own_team.team_colour == gamecontroller::TeamColour::CYAN ? TeamColour::CYAN : TeamColour::MAGENTA;
            state_changes.emplace_back([this, colour] { emit(std::make_unique<TeamColour>(colour)); });
        }


        /*******************************************************************************************
         * Process state/mode changes
         ******************************************************************************************/

        if (new_packet.mode != mode && old_packet.mode != gamecontroller::Mode::TIMEOUT
            && new_packet.mode != gamecontroller::Mode::TIMEOUT) {

            mode = new_packet.mode;

            // Changed modes but not to timeout
            switch (new_packet.mode) {
                case gamecontroller::Mode::NORMAL:
                    state->data.mode = GameState::Data::Mode::NORMAL;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Data::Mode::Value::NORMAL)); });
                    break;
                case gamecontroller::Mode::PENALTY_SHOOTOUT:
                    state->data.mode = GameState::Data::Mode::PENALTY_SHOOTOUT;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Data::Mode::Value::PENALTY_SHOOTOUT)); });
                    break;
                case gamecontroller::Mode::OVERTIME:
                    state->data.mode = GameState::Data::Mode::OVERTIME;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Data::Mode::Value::OVERTIME)); });
                    break;
                case gamecontroller::Mode::DIRECT_FREEKICK:
                    state->data.mode = GameState::Data::Mode::DIRECT_FREEKICK;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Data::Mode::Value::DIRECT_FREEKICK)); });
                    break;
                case gamecontroller::Mode::INDIRECT_FREEKICK:
                    state->data.mode = GameState::Data::Mode::INDIRECT_FREEKICK;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Data::Mode::Value::INDIRECT_FREEKICK)); });
                    break;
                case gamecontroller::Mode::PENALTYKICK:
                    state->data.mode = GameState::Data::Mode::PENALTYKICK;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Data::Mode::Value::PENALTYKICK)); });
                    break;
                case gamecontroller::Mode::CORNER_KICK:
                    state->data.mode = GameState::Data::Mode::CORNER_KICK;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Data::Mode::Value::CORNER_KICK)); });
                    break;
                case gamecontroller::Mode::GOAL_KICK:
                    state->data.mode = GameState::Data::Mode::GOAL_KICK;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Data::Mode::Value::GOAL_KICK)); });
                    break;
                case gamecontroller::Mode::THROW_IN:
                    state->data.mode = GameState::Data::Mode::THROW_IN;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Data::Mode::Value::THROW_IN)); });
                    break;
                default:
                    throw std::runtime_error("Invalid mode change");
                    emit(std::make_unique<GameState::Data::Mode>(state->data.mode));
            }
        }

        if ((state->data.mode != GameState::Data::Mode::NORMAL
             && state->data.mode != GameState::Data::Mode::PENALTY_SHOOTOUT
             && state->data.mode != GameState::Data::Mode::OVERTIME)
            && (new_packet.secondary_state_info[0] != state->data.secondary_state.team_performing
                || new_packet.secondary_state_info[1] != state->data.secondary_state.sub_mode)
            && new_packet.secondary_state_info[0] != 0) {
            if (new_packet.secondary_state_info[1] != state->data.secondary_state.sub_mode) {
                state_changes.emplace_back([] {});
            }
            state->data.secondary_state.team_performing = new_packet.secondary_state_info[0];
            state->data.secondary_state.sub_mode        = new_packet.secondary_state_info[1];
        }

        if (old_packet.mode != gamecontroller::Mode::TIMEOUT && new_packet.mode == gamecontroller::Mode::TIMEOUT) {

            // Change the game state to timeout
            auto time = NUClear::clock::now() + std::chrono::seconds(new_packet.secondary_time);

            state->data.phase          = GameState::Data::Phase::Value::TIMEOUT;
            state->data.secondary_time = time;

            state_changes.emplace_back([this, time] {
                auto msg   = std::make_unique<GamePhase>();
                msg->phase = GameState::Data::Phase::Value::TIMEOUT;
                msg->ends  = time;
                emit(msg);
            });
        }
        else if (old_packet.state != new_packet.state
                 || (old_packet.mode == gamecontroller::Mode::TIMEOUT
                     && new_packet.mode != gamecontroller::Mode::TIMEOUT)) {

            // State has changed, process it
            switch (new_packet.state) {
                case gamecontroller::State::INITIAL: {

                    state->data.phase = GameState::Data::Phase::Value::INITIAL;

                    state_changes.emplace_back([this] {
                        auto msg   = std::make_unique<GamePhase>();
                        msg->phase = GameState::Data::Phase::Value::INITIAL;
                        emit(msg);
                    });
                    break;
                }
                case gamecontroller::State::READY: {
                    /* Note: It was concluded that a specific 'dropped ball' event
                     * was not needed, but in the case that it is, this is how and
                     * where you would emit it:
                     */
                    // if (old_packet.state == State::PLAYING) {
                    //     // DROPPED BALL
                    // }
                    auto time = NUClear::clock::now() + std::chrono::seconds(new_packet.secondary_time);

                    state->data.phase          = GameState::Data::Phase::Value::READY;
                    state->data.secondary_time = time;

                    state_changes.emplace_back([this, time] {
                        auto msg        = std::make_unique<GamePhase>();
                        msg->phase      = GameState::Data::Phase::Value::READY;
                        msg->ready_time = time;
                        emit(msg);
                    });
                    break;
                }
                case gamecontroller::State::SET: {

                    state->data.phase = GameState::Data::Phase::Value::SET;

                    state_changes.emplace_back([this] {
                        auto msg   = std::make_unique<GamePhase>();
                        msg->phase = GameState::Data::Phase::Value::SET;
                        emit(msg);
                    });
                    break;
                }
                case gamecontroller::State::PLAYING: {
                    auto end_half  = NUClear::clock::now() + std::chrono::seconds(new_packet.secs_remaining);
                    auto ball_free = NUClear::clock::now() + std::chrono::seconds(new_packet.secondary_time);

                    state->data.primary_time   = end_half;
                    state->data.secondary_time = ball_free;
                    state->data.phase          = GameState::Data::Phase::Value::PLAYING;

                    state_changes.emplace_back([this, end_half, ball_free] {
                        auto msg       = std::make_unique<GamePhase>();
                        msg->phase     = GameState::Data::Phase::Value::PLAYING;
                        msg->end_half  = end_half;
                        msg->ball_free = ball_free;
                        emit(msg);
                    });
                    break;
                }
                case gamecontroller::State::FINISHED: {

                    auto next_half = NUClear::clock::now() + std::chrono::seconds(new_packet.secs_remaining);

                    state->data.primary_time = next_half;
                    state->data.phase        = GameState::Data::Phase::Value::FINISHED;

                    state_changes.emplace_back([this, next_half] {
                        auto msg       = std::make_unique<GamePhase>();
                        msg->phase     = GameState::Data::Phase::Value::FINISHED;
                        msg->next_half = next_half;
                        emit(msg);
                    });
                    break;
                }
            }

            emit(std::make_unique<GameState::Data::Phase>(state->data.phase));
        }

        if (!state_changes.empty()) {
            // emit new state
            emit(std::move(state));

            // emit individual state change events
            for (auto& change : state_changes) {
                change();
            }
        }
    }

    PenaltyReason GameController::get_penalty_reason(const gamecontroller::PenaltyState& penalty_state) {
        // ugly incoming
        switch (penalty_state) {
            case gamecontroller::PenaltyState::UNPENALISED: return PenaltyReason::UNPENALISED;
            case gamecontroller::PenaltyState::BALL_MANIPULATION: return PenaltyReason::BALL_MANIPULATION;
            case gamecontroller::PenaltyState::PHYSICAL_CONTACT: return PenaltyReason::PHYSICAL_CONTACT;
            case gamecontroller::PenaltyState::ILLEGAL_ATTACK: return PenaltyReason::ILLEGAL_ATTACK;
            case gamecontroller::PenaltyState::ILLEGAL_DEFENSE: return PenaltyReason::ILLEGAL_DEFENSE;
            case gamecontroller::PenaltyState::REQUEST_FOR_PICKUP: return PenaltyReason::REQUEST_FOR_PICKUP;
            case gamecontroller::PenaltyState::REQUEST_FOR_SERVICE: return PenaltyReason::REQUEST_FOR_SERVICE;
            case gamecontroller::PenaltyState::SUBSTITUTE: return PenaltyReason::SUBSTITUTE;
            case gamecontroller::PenaltyState::MANUAL: return PenaltyReason::MANUAL;
            case gamecontroller::PenaltyState::PLAYER_PUSHING: return PenaltyReason::PLAYER_PUSHING;
            default: throw std::runtime_error("Invalid Penalty State");
        }
    }

    const Team& GameController::get_own_team(const GameControllerPacket& state) const {

        for (const auto& team : state.teams) {
            if (team.team_id == TEAM_ID) {
                return team;
            }
        }

        throw std::runtime_error("Own team not found");
    }

    const Team& GameController::get_opponent_team(const GameControllerPacket& state) const {
        for (const auto& team : state.teams) {
            if (team.team_id != TEAM_ID) {
                return team;
            }
        }

        throw std::runtime_error("No opponent teams not found");  // should never happen!
    }

    std::string GameController::ip_address_int_to_string(const uint32_t& ip_addr) {
        uint32_t ip_addr_n = htonl(ip_addr);

        char c[255];
        std::memset(c, 0, sizeof(c));

        std::string addr = inet_ntop(AF_INET, &ip_addr_n, c, sizeof(c));

        return addr;
    }
}  // namespace module::input
