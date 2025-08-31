#include "Player.hpp"

#include <cmath>

#include "bind.hpp"

#include "extension/Configuration.hpp"

#include "message/eye/Rpc.hpp"

namespace module::nbs {

    using message::eye::RpcResponseMeta;

    using Load     = message::eye::ScrubberLoadRequest;
    using Pause    = message::eye::ScrubberPauseRequest;
    using Finished = message::eye::ScrubberPlaybackFinished;
    using Play     = message::eye::ScrubberPlayRequest;
    using SetMode  = message::eye::ScrubberSetModeRequest;
    using SetSpeed = message::eye::ScrubberSetPlaybackSpeedRequest;
    using State    = message::eye::ScrubberState;

    template <typename T>
    using Unbind = NUClear::dsl::operation::Unbind<T>;
    using NUClear::dsl::operation::ChronoTask;
    using NUClear::message::TimeTravel;

    using extension::Configuration;

    /// Structure used to emit a message from the decoder iterator
    struct EmitMessage {
        EmitMessage(utility::nbs::Decoder::Iterator decoder_iterator) : decoder_iterator(std::move(decoder_iterator)) {}
        utility::nbs::Decoder::Iterator decoder_iterator;  /// The decoder iterator containing the message to emit
    };

    /// Structure used to emit the current player state
    struct EmitState : message::eye::ScrubberState {
        using message::eye::ScrubberState::ScrubberState;
    };
    /// Structure used to emit the finished state when playback completes
    struct EmitFinished : State {
        using message::eye::ScrubberState::ScrubberState;
    };


    Player::Player(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        // Initialize playback state
        state.playback_state = State::State::ENDED;
        state.playback_speed = 0;  // Playback speed is measured in log increments so 0 = 100 speed (log(0) = 1)

        on<Configuration>("Player.yaml").then([this](const Configuration& cfg) {
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });

        // If the system is idle, then we might need to skip time forward to the next action
        on<Idle<>, Sync<Player>>().then([this] {
            // If playing in fast or playback mode skip to the next action
            if (state.playback_state == State::State::PLAYING
                && (state.playback_mode == State::Mode::FAST || state.playback_mode == State::Mode::SEQUENTIAL)) {
                log<DEBUG>("Skipping time forward to the next action");
                emit<Scope::INLINE>(std::make_unique<TimeTravel>(state.end, rtf(), TimeTravel::Action::NEAREST));
            }
        });

        on<Trigger<Load>, Sync<Player>>().then([this](const Load& load) {
            // Ensure we are unbound from the previous player
            unbind_player();
            // Use the reaction id of the load reaction as the chrono task id
            chrono_task_id = NUClear::threading::ReactionTask::get_current_task()->id;

            log<INFO>("Loading NBS files:");
            std::vector<std::filesystem::path> file_paths;
            for (const auto& path : load.files) {
                std::filesystem::path file_path(path);
                if (std::filesystem::exists(file_path)) {
                    file_paths.push_back(file_path);
                    log<INFO>(" - ", path);
                }
                else {
                    log<ERROR>("File does not exist: ", path);
                }
            }

            if (file_paths.empty()) {
                log<ERROR>("No valid NBS files to load.");
                emit(std::make_unique<Load::Response>(
                    RpcResponseMeta(load.rpc.token, false, "No valid NBS files to load.")));
                return;
            }

            decoder          = utility::nbs::Decoder(file_paths);
            decoder_iterator = decoder.begin();

            // Store the initial player state
            state.end =
                NUClear::clock::time_point(std::chrono::nanoseconds(std::prev(decoder.end())->item->item.timestamp));
            state.start = NUClear::clock::time_point(std::chrono::nanoseconds(decoder.begin()->item->item.timestamp));
            state.timestamp       = state.start;
            state.playback_state  = State::State::PAUSED;
            state.playback_speed  = 0;
            state.playback_repeat = false;
            state.total_messages  = uint32_t(std::distance(decoder.begin(), decoder.end()));
            state.current_message = 0;

            // Synchronise the clock epoch to the first message and set the playback speed to 0.0
            // Future chrono tasks are moved relatively so future everys etc won't be in the distant future
            emit<Scope::INLINE>(std::make_unique<TimeTravel>(state.start, 0.0, TimeTravel::Action::RELATIVE));

            // Update which types we will be playing
            log<INFO>("Enabling messages:");
            for (const auto& message_name : load.messages) {
                log<INFO>(" - ", message_name, "enabled");
                bind(message_name, *this, decoder);
            }

            // Emit the player state
            emit(std::make_unique<State>(state));

            emit(std::make_unique<Load::Response>(RpcResponseMeta(load.rpc.token, true)));
        });

        on<Trigger<SetMode>, Sync<Player>>().then([this](const SetMode& set_mode) {
            state.playback_mode = set_mode.mode;
            update_clock();
            emit(std::make_unique<State>(state));

            log<INFO>("Playback mode set to: ", set_mode.mode);

            emit(std::make_unique<SetMode::Response>(RpcResponseMeta(set_mode.rpc.token, true)));
        });

        on<Trigger<Play>, Sync<Player>>().then([this](const Play& play) {
            state.playback_state = State::State::PLAYING;
            update_clock();
            emit(std::make_unique<State>(state));

            bind_player();  // This starts the player running

            log<INFO>("Playback started");

            emit(std::make_unique<Play::Response>(RpcResponseMeta(play.rpc.token, true)));
        });

        on<Trigger<Pause>, Sync<Player>>().then([this](const Pause& pause) {
            state.playback_state = State::State::PAUSED;
            update_clock();
            emit(std::make_unique<State>(state));

            log<INFO>("Playback paused");

            emit(std::make_unique<Pause::Response>(RpcResponseMeta(pause.rpc.token, true)));
        });

        on<Trigger<SetSpeed>, Sync<Player>>().then([this](const SetSpeed& set_speed) {
            state.playback_speed = set_speed.playback_speed;
            update_clock();
            emit(std::make_unique<State>(state));

            log<INFO>("Setting playback speed to: ", set_speed.playback_speed);

            emit(std::make_unique<SetSpeed::Response>(RpcResponseMeta(set_speed.rpc.token, true)));
        });


        // These trigger is specifically used to break the loop in the chrono task below
        // If that task directly emits the state, it can go to another module which may try to make its own chrono tasks
        // In that case the system will deadlock as ChronoController tries to obtain its own mutex
        // This trigger fixes that by moving the actual emitting to the Player thread
        // It is kept as sync to ensure that the state update messages are always emitted in order
        on<Trigger<EmitMessage>, Inline::NEVER>().then(
            [](const EmitMessage& send_message) { (*send_message.decoder_iterator)(); });
        on<Trigger<EmitState>, Sync<Player>>().then(
            [this](const EmitState& emit_state) { emit(std::make_unique<State>(emit_state)); });
        on<Trigger<EmitFinished>, Sync<Player>, Inline::NEVER>().then([this](const EmitFinished& emit_finished) {
            emit(std::make_unique<TimeTravel>(state.end, 0.0, TimeTravel::Action::ABSOLUTE));
            emit(std::make_unique<State>(emit_finished));
            emit(std::make_unique<Finished>());
        });
    }

    double Player::rtf() const {
        // All states should have the clock frozen except when running in non-sequential mode
        return state.playback_state == State::State::PLAYING && state.playback_mode != State::Mode::SEQUENTIAL
                   ? std::pow(2, state.playback_speed)
                   : 0.0;
    }

    void Player::update_clock() {
        emit<Scope::INLINE>(std::make_unique<TimeTravel>(NUClear::clock::now(), rtf(), TimeTravel::Action::ABSOLUTE));
    }

    void Player::bind_player() {
        // Create the chrono task
        emit<Scope::INLINE>(std::make_unique<ChronoTask>(
            [this](NUClear::clock::time_point& t) {
                // Skip to the next message that has a callback
                while (decoder_iterator != decoder.end() && !decoder_iterator->has_callback()) {
                    ++decoder_iterator;
                }

                if (decoder_iterator != decoder.end()) {
                    auto ts =
                        NUClear::clock::time_point(std::chrono::nanoseconds(decoder_iterator->item->item.timestamp));

                    if (ts <= t) {
                        state.timestamp       = ts;
                        state.current_message = uint32_t(std::distance(decoder.begin(), decoder_iterator));

                        // Emit the message and state, done in a separate trigger to avoid locking up chrono controller
                        emit(std::make_unique<EmitMessage>(decoder_iterator));
                        emit(std::make_unique<EmitState>(state));

                        ++decoder_iterator;

                        // Now skip ahead to the next message that has a callback
                        while (decoder_iterator != decoder.end() && !decoder_iterator->has_callback()) {
                            t = NUClear::clock::time_point(
                                std::chrono::nanoseconds(decoder_iterator->item->item.timestamp));
                            ++decoder_iterator;
                        }
                    }
                    else {
                        t = ts;
                    }
                }
                else {
                    state.timestamp      = state.end;
                    state.playback_state = State::State::ENDED;
                    emit(std::make_unique<EmitFinished>(state));
                    return false;  // Don't run the chrono task again
                }

                return true;
            },
            state.start,
            chrono_task_id));
    }

    void Player::unbind_player() {
        if (chrono_task_id != 0) {
            emit<Scope::INLINE>(std::make_unique<Unbind<ChronoTask>>(chrono_task_id));
        }
    }


}  // namespace module::nbs
