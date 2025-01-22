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
#include "Player.hpp"

#include <cmath>

#include "extension/Configuration.hpp"

#include "message/eye/Rpc.hpp"

namespace module::nbs {

    using message::eye::RpcResponseMeta;
    using message::nbs::player::LoadRequest;
    using message::nbs::player::PauseRequest;
    using message::nbs::player::PlaybackFinished;
    using message::nbs::player::PlaybackState;
    using message::nbs::player::PlayRequest;
    using message::nbs::player::SetModeRequest;
    using message::nbs::player::SetPlaybackSpeedRequest;
    using message::nbs::player::PlaybackMode::FAST;
    using message::nbs::player::PlaybackMode::REALTIME;
    using message::nbs::player::PlaybackMode::SEQUENTIAL;

    using extension::Configuration;

    Player::Player(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Register emitters for all the message types enabled
        register_emitters();

        on<Configuration>("Player.yaml").then([this](const Configuration& cfg) {
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<LoadRequest>>().then([this](const LoadRequest& load_request) {
            if (playback_state != message::nbs::player::PlaybackState::State::ENDED) {
                log<NUClear::ERROR>("Cannot load NBS files while playing.");
                emit(std::make_unique<LoadRequest::Response>(
                    RpcResponseMeta(load_request.rpc.token, false, "Cannot load NBS files while playing.")));
                return;
            }

            if (load_request.files.empty()) {
                // If no NBS files are provided, don't continue
                log<NUClear::ERROR>("No NBS files provided.");
                emit(std::make_unique<LoadRequest::Response>(
                    RpcResponseMeta(load_request.rpc.token, false, "No NBS files provided.")));
                return;
            }

            log<NUClear::INFO>("Loading NBS files:");
            std::vector<std::filesystem::path> file_paths;
            for (const auto& path : load_request.files) {
                std::filesystem::path file_path(path);
                if (std::filesystem::exists(file_path)) {
                    file_paths.push_back(file_path);
                    log<NUClear::INFO>(" - ", path);
                }
                else {
                    log<NUClear::ERROR>("File does not exist: ", path);
                }
            }

            if (file_paths.empty()) {
                log<NUClear::ERROR>("No valid NBS files to load.");
                emit(std::make_unique<LoadRequest::Response>(
                    RpcResponseMeta(load_request.rpc.token, false, "No valid NBS files to load.")));
                return;
            }
            decoder = utility::nbs::Decoder(file_paths, true);
            // Get the total number of messages
            total_messages = std::distance(decoder.begin(), decoder.end());
            // Get the timestamp of the last message
            auto last_message = std::prev(decoder.end());
            end_time = NUClear::clock::time_point(std::chrono::nanoseconds((*last_message).item->item.timestamp));
            // Get the timestamp of the first message
            auto first_message = decoder.begin();
            start_time = NUClear::clock::time_point(std::chrono::nanoseconds((*first_message).item->item.timestamp));
            decoder_iterator = decoder.begin();

            // Update which types we will be playing
            log<INFO>("Enabling messages:");
            for (const auto& message_name : load_request.messages) {
                // Hash our type to work out our type on the wire
                uint64_t hash =
                    NUClear::util::serialise::xxhash64(message_name.c_str(), message_name.size(), 0x4e55436c);
                if (emitters.find(hash) != emitters.end()) {
                    emitters[hash](decoder);
                    log<INFO>(" - ", message_name);
                }
                else {
                    log<NUClear::ERROR>("Message type not found: ", message_name);
                }
            }

            // Synchronise the clock epoch to the first message timestamp
            if (decoder_iterator != decoder.end()) {
                emit<Scope::INLINE>(
                    std::make_unique<NUClear::message::TimeTravel>(start_time,
                                                                   playback_speed,
                                                                   NUClear::message::TimeTravel::Action::RELATIVE));
                // Emit the first message
                emit_next_message();
            }

            // Emit the playback state
            emit_playback_state();

            // Emit RPC response
            emit(std::make_unique<LoadRequest::Response>(RpcResponseMeta(load_request.rpc.token, true)));
        });

        on<Trigger<SetModeRequest>>().then([this](const SetModeRequest& set_mode_request) {
            mode = set_mode_request.mode;
            log<NUClear::INFO>("Playback mode set to: ", set_mode_request.mode);

            // Emit the playback state
            emit_playback_state();

            // Emit RPC response
            emit(std::make_unique<SetModeRequest::Response>(RpcResponseMeta(set_mode_request.rpc.token, true)));
        });

        on<Trigger<PauseRequest>>().then([this](const PauseRequest& pause_request) {
            // Set the clock rtf to 0.0 to pause time
            emit<Scope::INLINE>(
                std::make_unique<NUClear::message::TimeTravel>(NUClear::clock::now(),
                                                               0.0,
                                                               NUClear::message::TimeTravel::Action::RELATIVE));
            // Unbind the player handles
            realtime_player_handle.disable();
            skip_idle_player_handle.disable();

            // Emit the playback state
            playback_state = message::nbs::player::PlaybackState::State::PAUSED;
            emit_playback_state();

            // Emit RPC response
            emit(std::make_unique<PauseRequest::Response>(RpcResponseMeta(pause_request.rpc.token, true)));
        });

        on<Trigger<SetPlaybackSpeedRequest>>().then([this](const SetPlaybackSpeedRequest& set_speed_request) {
            playback_speed = std::pow(2.0, set_speed_request.playback_speed);

            // Emit the playback state
            emit_playback_state();

            // Emit RPC response
            emit(std::make_unique<SetPlaybackSpeedRequest::Response>(
                RpcResponseMeta(set_speed_request.rpc.token, true)));
        });

        on<Trigger<PlayRequest>>().then([this](const PlayRequest& play_request) {
            // Disable the player handles
            realtime_player_handle.disable();
            skip_idle_player_handle.disable();

            switch (mode.value) {
                case FAST:
                    // Set RTF to match the desired playback speed
                    emit<Scope::DIRECT>(
                        std::make_unique<NUClear::message::TimeTravel>(NUClear::clock::now(),
                                                                       playback_speed,
                                                                       NUClear::message::TimeTravel::Action::RELATIVE));
                    enable_skip_idle_player();
                    enable_realtime_player();
                    break;

                case REALTIME:
                    // Set RTF to match the desired playback speed
                    emit<Scope::DIRECT>(
                        std::make_unique<NUClear::message::TimeTravel>(NUClear::clock::now(),
                                                                       playback_speed,
                                                                       NUClear::message::TimeTravel::Action::RELATIVE));
                    enable_realtime_player();
                    break;

                case SEQUENTIAL:
                    // Set RTF to zero to pause time in SEQUENTIAL mode and have IDLE jump between messages/tasks
                    playback_speed = 0.0;
                    emit<Scope::INLINE>(
                        std::make_unique<NUClear::message::TimeTravel>(NUClear::clock::now(),
                                                                       playback_speed,
                                                                       NUClear::message::TimeTravel::Action::RELATIVE));
                    enable_skip_idle_player();
                    break;
                default:
                    log<NUClear::ERROR>("Invalid playback mode selected.");
                    emit(std::make_unique<PlayRequest::Response>(
                        RpcResponseMeta(play_request.rpc.token, false, "Invalid playback mode selected.")));
                    return;
                    break;
            }

            // Emit the playback state
            playback_state = message::nbs::player::PlaybackState::State::PLAYING;
            emit_playback_state();

            // Emit RPC response
            emit(std::make_unique<PlayRequest::Response>(RpcResponseMeta(play_request.rpc.token, true)));
        });
    }

    void Player::enable_skip_idle_player() {
        skip_idle_player_handle = on<Idle<>, Single>().then([this] {
            std::lock_guard<std::mutex> decoder_lock(decoder_mutex);
            if (NUClear::clock::now() < target_emit_time) {
                emit<Scope::INLINE>(
                    std::make_unique<NUClear::message::TimeTravel>(target_emit_time,
                                                                   playback_speed,
                                                                   NUClear::message::TimeTravel::Action::NEAREST));
            }
            emit_next_message();
            cv.notify_all();
        });
        skip_idle_player_handle.enable();
    }

    void Player::enable_realtime_player() {
        realtime_player_handle = on<Always>().then([this] {
            std::unique_lock<std::mutex> decoder_lock(decoder_mutex);
            emit_next_message();
            cv.wait_until(decoder_lock, target_emit_time);
        });
        realtime_player_handle.enable();
    }

    void Player::emit_next_message() {
        if (decoder_iterator != decoder.end() && (*decoder_iterator).has_callback()) {
            target_emit_time =
                NUClear::clock::time_point(std::chrono::nanoseconds((*decoder_iterator).item->item.timestamp));

            // Only emit the message if the target emit time has been reached
            if (NUClear::clock::now() >= target_emit_time) {
                // Emit the message
                (*decoder_iterator)();
                ++decoder_iterator;
                // Emit the playback state
                emit_playback_state();
            }
        }

        // Skip the message if it has no callback
        while (decoder_iterator != decoder.end() && !(*decoder_iterator).has_callback()) {
            ++decoder_iterator;
        }

        if (playback_state != message::nbs::player::PlaybackState::State::ENDED && decoder_iterator == decoder.end()) {
            realtime_player_handle.disable();
            skip_idle_player_handle.disable();
            playback_state = message::nbs::player::PlaybackState::State::ENDED;
            // Emit the playback state
            emit_playback_state();
            emit(std::make_unique<PlaybackFinished>());
        }
    }

    void Player::emit_playback_state() {
        auto playback_state             = std::make_unique<PlaybackState>();
        playback_state->current_message = std::distance(decoder.begin(), decoder_iterator);
        playback_state->total_messages  = total_messages;
        playback_state->timestamp       = NUClear::clock::now();
        playback_state->start           = start_time;
        playback_state->end             = end_time;
        playback_state->playback_state  = message::nbs::player::PlaybackState::State::PLAYING;
        playback_state->playback_speed  = int32_t(std::log2(playback_speed));
        emit(std::move(playback_state));
    }

}  // namespace module::nbs
