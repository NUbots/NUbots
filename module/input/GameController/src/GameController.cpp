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

#include "message/support/GlobalConfig.hpp"

namespace module::input {

    using extension::Configuration;
    using gamecontroller::GameControllerPacket;
    using gamecontroller::GameControllerReplyPacket;
    using gamecontroller::Team;
    using message::input::GameEvents;
    using message::input::GameState;
    using message::support::GlobalConfig;
    using Score          = GameEvents::Score;
    using GoalScored     = GameEvents::GoalScored;
    using Penalisation   = GameEvents::Penalisation;
    using Unpenalisation = GameEvents::Unpenalisation;
    using HalfTime       = GameEvents::HalfTime;
    using KickOffTeam    = GameEvents::KickOffTeam;
    using GameEventPhase = GameEvents::GamePhase;
    using GameMode       = GameEvents::GameMode;
    using PenaltyReason  = GameState::PenaltyReason;
    using TeamColour     = GameState::TeamColour::Value;

    GameController::GameController(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        // Configure
        on<Configuration, Trigger<GlobalConfig>>("GameController.yaml")
            .then("GameController Configuration",
                  [this](const Configuration& config, const GlobalConfig& global_config) {
                      log_level = config["log_level"].as<NUClear::LogLevel>();

                      PLAYER_ID = global_config.player_id;
                      TEAM_ID   = global_config.team_id;
                      send_port = config["send_port"].as<uint>();

                      udp_filter_address = config["udp_filter_address"].as<std::string>("");

                      if (config["use_set_gc_address"].as<bool>(false)) {
                          game_controller_address = config["gc_address"].as<std::string>("");
                          log<INFO>("Using configured game controller address", game_controller_address);
                      }

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

                                      if (game_controller_address != remote_addr) {
                                          game_controller_address = remote_addr;
                                          log<INFO>("Game controller address", game_controller_address);
                                      }

                                      // Get our packet contents
                                      const GameControllerPacket& new_packet =
                                          *reinterpret_cast<const GameControllerPacket*>(p.payload.data());

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

        on<Every<2, Per<std::chrono::seconds>>>().then("GameController Reply", [this] { send_reply_message(); });
    }

    void GameController::send_reply_message() {
        auto packet     = std::make_unique<GameControllerReplyPacket>();
        packet->header  = {gamecontroller::RETURN_HEADER[0],
                           gamecontroller::RETURN_HEADER[1],
                           gamecontroller::RETURN_HEADER[2],
                           gamecontroller::RETURN_HEADER[3]};
        packet->version = gamecontroller::RETURN_VERSION;
        packet->team    = TEAM_ID;
        packet->player  = PLAYER_ID;

        // fallen, pose, ball_age and ball should be optional to be sent to the GameController
        // TODO: wire up from localisation if required in the future
        packet->fallen   = self_penalised ? 1 : 0;
        packet->pose[0]  = 0.f;
        packet->pose[1]  = 0.f;
        packet->pose[2]  = 0.f;
        packet->ball_age = -1.f;
        packet->ball[0]  = 0.f;
        packet->ball[1]  = 0.f;

        if (game_controller_address != "") {
            emit<Scope::UDP>(packet, game_controller_address, send_port);
        }
    }

    void GameController::reset_state() {

        game_phase = static_cast<gamecontroller::GamePhase>(-1);
        set_play   = static_cast<gamecontroller::SetPlay>(-1);

        std::copy(std::begin(gamecontroller::RECEIVE_HEADER),
                  std::end(gamecontroller::RECEIVE_HEADER),
                  std::begin(packet.header));
        packet.version          = SUPPORTED_VERSION;
        packet.packet_number    = 0;
        packet.players_per_team = PLAYERS_PER_TEAM;
        packet.state            = static_cast<gamecontroller::State>(-1);
        packet.first_half       = true;
        packet.kicking_team     = 0;
        packet.game_phase       = static_cast<gamecontroller::GamePhase>(-1);
        packet.set_play         = static_cast<gamecontroller::SetPlay>(-1);
        packet.secs_remaining   = 0;
        packet.secondary_time   = 0;
        for (uint i = 0; i < NUM_TEAMS; i++) {
            auto& own_team               = packet.teams[i];
            own_team.team_id             = (i == 0 ? TEAM_ID : 0);
            own_team.field_player_colour = static_cast<gamecontroller::TeamColour>(-1);
            own_team.score               = 0;
            own_team.penalty_shot        = 0;
            own_team.single_shots        = 0;
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
        initial_state->phase            = GameState::Phase::INITIAL;
        initial_state->mode             = GameState::Mode::NORMAL;
        initial_state->first_half       = true;
        initial_state->kicked_out_by_us = false;
        initial_state->our_kick_off     = false;

        initial_state->team.team_id         = TEAM_ID;
        initial_state->team.team_colour     = TeamColour::UNKNOWN;
        initial_state->opponent.team_id     = 0;
        initial_state->opponent.team_colour = TeamColour::UNKNOWN;

        emit(std::move(initial_state));
        emit(std::make_unique<GameState::Phase>(GameState::Phase::INITIAL));
    }

    void GameController::process(const GameState& old_game_state,
                                 const GameControllerPacket& old_packet,
                                 const GameControllerPacket& new_packet) {

        auto state = std::make_unique<GameState>(old_game_state);

        std::vector<std::function<void()>> state_changes;

        // Stopped play
        if (old_packet.stopped != new_packet.stopped) {
            state_changes.emplace_back(
                [this, new_packet] { log<DEBUG>("Stopped state changed:", uint(new_packet.stopped)); });
        }
        state->stopped = new_packet.stopped;

        // Game score
        const auto& old_own_team = get_own_team(old_packet);
        const auto& new_own_team = get_own_team(new_packet);

        const auto& old_opponent_team = get_opponent_team(old_packet);
        const auto& new_opponent_team = get_opponent_team(new_packet);

        // Team colours
        state->team.team_colour     = get_team_colour(new_own_team.field_player_colour);
        state->opponent.team_colour = get_team_colour(new_opponent_team.field_player_colour);

        // Log message budget for cross-checking against local counter
        log<DEBUG>("GameController message budget remaining:", new_own_team.message_budget);

        /*******************************************************************************************
         * Process score updates
         ******************************************************************************************/
        if (old_own_team.score != new_own_team.score || old_opponent_team.score != new_opponent_team.score) {
            // score update
            emit(std::make_unique<GameEvents::Score>(GameEvents::Score{new_own_team.score, new_opponent_team.score}));


            if (old_own_team.score < new_own_team.score) {
                // we scored! :D

                // Set the team scores in the state packet
                state->team.score = new_own_team.score;
                state_changes.emplace_back([this, new_own_team] {
                    emit(
                        std::make_unique<GoalScored>(GoalScored{GameEvents::Context::Value::TEAM, new_own_team.score}));
                });
            }

            if (old_opponent_team.score < new_opponent_team.score) {
                // they scored :( boo

                // Set the team scores in the state packet
                state->opponent.score = new_opponent_team.score;
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
        state->team.players.clear();
        state->opponent.players.clear();

        // Note: assumes players_per_team never changes
        for (uint i = 0; i < new_packet.players_per_team; i++) {
            uint player_id             = i + 1;
            const auto& old_own_player = old_own_team.players[i];
            const auto& new_own_player = new_own_team.players[i];

            const auto& old_opponent_player = old_opponent_team.players[i];
            const auto& new_opponent_player = new_opponent_team.players[i];

            // Update our state
            GameState::Robot own_player =
                GameState::Robot(player_id,
                                 get_penalty_reason(new_own_player.penalty_state),
                                 NUClear::clock::now() + std::chrono::seconds(new_own_player.penalised_time_left),
                                 new_own_team.goalkeeper == player_id);
            state->team.players.push_back(own_player);
            if (player_id == PLAYER_ID) {
                state->self = own_player;
            }

            state->opponent.players.emplace_back(
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
                        send_reply_message();
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
                        send_reply_message();
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
         * Process half changes
         ******************************************************************************************/
        if (old_packet.first_half != new_packet.first_half) {

            // Update the half time in the state
            state->first_half = new_packet.first_half;

            // Half time
            state_changes.emplace_back(
                [this, new_packet] { emit(std::make_unique<HalfTime>(HalfTime{new_packet.first_half})); });
        }

        /*******************************************************************************************
         * Process kick off team
         ******************************************************************************************/
        if (old_packet.kicking_team != new_packet.kicking_team) {

            // Update the kickoff team (us or them)
            state->our_kick_off = new_packet.kicking_team == new_own_team.team_id;

            // Get the new kick off team to emit the team change
            GameEvents::Context team = new_packet.kicking_team == new_own_team.team_id
                                           ? GameEvents::Context::Value::TEAM
                                           : GameEvents::Context::Value::OPPONENT;

            state_changes.emplace_back([this, team] { emit(std::make_unique<KickOffTeam>(KickOffTeam{team})); });
        }


        /*******************************************************************************************
         * Process our team colour
         ******************************************************************************************/
        if (old_own_team.field_player_colour != new_own_team.field_player_colour) {
            TeamColour colour = get_team_colour(new_own_team.field_player_colour);
            state_changes.emplace_back([this, colour] { emit(std::make_unique<TeamColour>(colour)); });
        }


        /*******************************************************************************************
         * Process game phase changes
         ******************************************************************************************/

        if (old_packet.game_phase != gamecontroller::GamePhase::TIMEOUT
            && new_packet.game_phase == gamecontroller::GamePhase::TIMEOUT) {

            // Change the game state to timeout
            auto time = NUClear::clock::now() + std::chrono::seconds(new_packet.secondary_time);

            state->phase          = GameState::Phase::Value::TIMEOUT;
            state->secondary_time = time;

            state_changes.emplace_back([this, time] {
                auto msg   = std::make_unique<GameEventPhase>();
                msg->phase = GameState::Phase::Value::TIMEOUT;
                msg->ends  = time;
                emit(msg);
            });
        }
        else if (old_packet.state != new_packet.state
                 || (old_packet.game_phase == gamecontroller::GamePhase::TIMEOUT
                     && new_packet.game_phase != gamecontroller::GamePhase::TIMEOUT)) {

            // State has changed, process it
            switch (new_packet.state) {
                case gamecontroller::State::INITIAL: {

                    state->phase = GameState::Phase::Value::INITIAL;

                    state_changes.emplace_back([this] {
                        auto msg   = std::make_unique<GameEventPhase>();
                        msg->phase = GameState::Phase::Value::INITIAL;
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

                    state->phase          = GameState::Phase::Value::READY;
                    state->secondary_time = time;

                    state_changes.emplace_back([this, time] {
                        auto msg        = std::make_unique<GameEventPhase>();
                        msg->phase      = GameState::Phase::Value::READY;
                        msg->ready_time = time;
                        emit(msg);
                    });
                    break;
                }
                case gamecontroller::State::SET: {

                    state->phase = GameState::Phase::Value::SET;

                    state_changes.emplace_back([this] {
                        auto msg   = std::make_unique<GameEventPhase>();
                        msg->phase = GameState::Phase::Value::SET;
                        emit(msg);
                    });
                    break;
                }
                case gamecontroller::State::PLAYING: {
                    auto end_half  = NUClear::clock::now() + std::chrono::seconds(new_packet.secs_remaining);
                    auto ball_free = NUClear::clock::now() + std::chrono::seconds(new_packet.secondary_time);

                    state->primary_time   = end_half;
                    state->secondary_time = ball_free;
                    state->phase          = GameState::Phase::Value::PLAYING;

                    state_changes.emplace_back([this, end_half, ball_free] {
                        auto msg       = std::make_unique<GameEventPhase>();
                        msg->phase     = GameState::Phase::Value::PLAYING;
                        msg->end_half  = end_half;
                        msg->ball_free = ball_free;
                        emit(msg);
                    });
                    break;
                }
                case gamecontroller::State::FINISHED: {

                    auto next_half = NUClear::clock::now() + std::chrono::seconds(new_packet.secs_remaining);

                    state->primary_time = next_half;
                    state->phase        = GameState::Phase::Value::FINISHED;

                    state_changes.emplace_back([this, next_half] {
                        auto msg       = std::make_unique<GameEventPhase>();
                        msg->phase     = GameState::Phase::Value::FINISHED;
                        msg->next_half = next_half;
                        emit(msg);
                    });
                    break;
                }
            }

            emit(std::make_unique<GameState::Phase>(state->phase));
        }

        /*******************************************************************************************
         * Process set play changes
         ******************************************************************************************/
        if (old_packet.set_play != new_packet.set_play) {
            set_play = new_packet.set_play;
            switch (new_packet.set_play) {
                case gamecontroller::SetPlay::DIRECT_FREE_KICK:
                    state->mode = GameState::Mode::DIRECT_FREEKICK;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Mode::Value::DIRECT_FREEKICK)); });
                    break;
                case gamecontroller::SetPlay::INDIRECT_FREE_KICK:
                    state->mode = GameState::Mode::INDIRECT_FREEKICK;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Mode::Value::INDIRECT_FREEKICK)); });
                    break;
                case gamecontroller::SetPlay::PENALTY_KICK:
                    state->mode = GameState::Mode::PENALTYKICK;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Mode::Value::PENALTYKICK)); });
                    break;
                case gamecontroller::SetPlay::CORNER_KICK:
                    state->mode = GameState::Mode::CORNER_KICK;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Mode::Value::CORNER_KICK)); });
                    break;
                case gamecontroller::SetPlay::GOAL_KICK:
                    state->mode = GameState::Mode::GOAL_KICK;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Mode::Value::GOAL_KICK)); });
                    break;
                case gamecontroller::SetPlay::THROW_IN:
                    state->mode = GameState::Mode::THROW_IN;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Mode::Value::THROW_IN)); });
                    break;
                case gamecontroller::SetPlay::NONE:
                    state->mode = GameState::Mode::NORMAL;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Mode::Value::NORMAL)); });
                    break;
            }
        }

        if (old_packet.game_phase != new_packet.game_phase) {
            game_phase = new_packet.game_phase;
            switch (new_packet.game_phase) {
                case gamecontroller::GamePhase::PENALTY_SHOOTOUT:
                    state->mode = GameState::Mode::PENALTY_SHOOTOUT;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Mode::Value::PENALTY_SHOOTOUT)); });
                    break;
                case gamecontroller::GamePhase::EXTRA_TIME:
                    state->mode = GameState::Mode::OVERTIME;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Mode::Value::OVERTIME)); });
                    break;
                case gamecontroller::GamePhase::NORMAL:
                    state->mode = GameState::Mode::NORMAL;
                    state_changes.emplace_back(
                        [this] { emit(std::make_unique<GameMode>(GameState::Mode::Value::NORMAL)); });
                    break;
                default: break;
            }
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
        switch (penalty_state) {
            case gamecontroller::PenaltyState::UNPENALISED: return PenaltyReason::UNPENALISED;
            case gamecontroller::PenaltyState::ILLEGAL_POSITIONING: return PenaltyReason::ILLEGAL_POSITIONING;
            case gamecontroller::PenaltyState::MOTION_IN_SET: return PenaltyReason::MOTION_IN_SET;
            case gamecontroller::PenaltyState::MOTION_IN_STOP: return PenaltyReason::MOTION_IN_STOP;
            case gamecontroller::PenaltyState::LOCAL_GAME_STUCK: return PenaltyReason::LOCAL_GAME_STUCK;
            case gamecontroller::PenaltyState::INCAPABLE_ROBOT: return PenaltyReason::INCAPABLE_ROBOT;
            case gamecontroller::PenaltyState::PICK_UP: return PenaltyReason::PICK_UP;
            case gamecontroller::PenaltyState::BALL_HOLDING: return PenaltyReason::BALL_HOLDING;
            case gamecontroller::PenaltyState::LEAVING_THE_FIELD: return PenaltyReason::LEAVING_THE_FIELD;
            case gamecontroller::PenaltyState::PLAYING_WITH_ARMS_HANDS: return PenaltyReason::PLAYING_WITH_ARMS_HANDS;
            case gamecontroller::PenaltyState::PLAYER_PUSHING: return PenaltyReason::PLAYER_PUSHING;
            case gamecontroller::PenaltyState::CAUTIONED: return PenaltyReason::CAUTIONED;
            case gamecontroller::PenaltyState::SENT_OFF: return PenaltyReason::SENT_OFF;
            case gamecontroller::PenaltyState::SUBSTITUTE: return PenaltyReason::SUBSTITUTE;
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

    GameState::TeamColour::Value GameController::get_team_colour(const gamecontroller::TeamColour& colour) {
        switch (colour) {
            case gamecontroller::TeamColour::BLUE: return TeamColour::BLUE;
            case gamecontroller::TeamColour::RED: return TeamColour::RED;
            case gamecontroller::TeamColour::YELLOW: return TeamColour::YELLOW;
            case gamecontroller::TeamColour::BLACK: return TeamColour::BLACK;
            case gamecontroller::TeamColour::WHITE: return TeamColour::WHITE;
            case gamecontroller::TeamColour::GREEN: return TeamColour::GREEN;
            case gamecontroller::TeamColour::ORANGE: return TeamColour::ORANGE;
            case gamecontroller::TeamColour::PURPLE: return TeamColour::PURPLE;
            case gamecontroller::TeamColour::BROWN: return TeamColour::BROWN;
            case gamecontroller::TeamColour::GRAY: return TeamColour::GRAY;
            default: return TeamColour::UNKNOWN;
        }
    }
}  // namespace module::input
