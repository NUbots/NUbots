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
#include "PlayGoalie.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Buttons.hpp"
#include "message/input/GameState.hpp"
#include "message/localisation/Field.hpp"
#include "message/output/Buzzer.hpp"
#include "message/purpose/Player.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"

namespace module::purpose {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::ButtonLeftDown;
    using message::input::ButtonLeftUp;
    using message::input::ButtonMiddleDown;
    using message::input::ButtonMiddleUp;
    using message::input::GameState;
    using message::localisation::ResetFieldLocalisation;
    using message::output::Buzzer;
    using message::purpose::Goalie;
    using message::skill::Look;
    using message::skill::Walk;
    using message::strategy::FallRecovery;

    PlayGoalie::PlayGoalie(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlayGoalie.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.start_delay = config["start_delay"].as<int>();
        });

        on<Startup>().then([this] {
            // There is no GameController, so the game is always in normal play and this robot is always the goalie.
            // purpose::Goalie only plays in PLAYING, and reads whether play is stopped from the GameState
            auto game_state         = std::make_unique<GameState>();
            game_state->phase       = GameState::Phase::PLAYING;
            game_state->mode        = GameState::Mode::NORMAL;
            game_state->first_half  = true;
            game_state->self.goalie = true;
            emit(std::move(game_state));
            emit(std::make_unique<GameState::Phase>(GameState::Phase::PLAYING));

            // Without these emits, modules that need a Stability and WalkState messages may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));
            // Stand idle, looking forward, while nothing else wants the legs or the head
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 0);
            emit<Task>(std::make_unique<Look>(Eigen::Vector3d::UnitX(), true), 0);

            emit<Scope::DELAY>(std::make_unique<StartPlaying>(), std::chrono::seconds(cfg.start_delay));
        });

        on<Trigger<StartPlaying>>().then([this] {
            if (paused) {
                return;
            }
            log<INFO>("Playing goalie");
            // The robot should always try to recover from falling, above playing
            emit<Task>(std::make_unique<FallRecovery>(), 2);
            emit<Task>(std::make_unique<Goalie>(), 1);
        });

        // Left button pauses: the goalie stops and stands where it is
        on<Trigger<ButtonLeftDown>>().then([this] {
            paused = true;
            emit<Task>(std::unique_ptr<Goalie>(nullptr));
            emit<Task>(std::unique_ptr<FallRecovery>(nullptr));
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit<Scope::INLINE>(std::make_unique<Buzzer>(1000));
            log<INFO>("Paused");
        });

        // Middle button resumes after the start delay. Field localisation restarts, so put the robot where
        // FieldLocalisationNLopt's starting_side expects it, as at the start of a game
        on<Trigger<ButtonMiddleDown>>().then([this] {
            paused = false;
            emit<Scope::INLINE>(std::make_unique<ResetFieldLocalisation>());
            emit<Scope::INLINE>(std::make_unique<Buzzer>(1000));
            emit<Scope::DELAY>(std::make_unique<StartPlaying>(), std::chrono::seconds(cfg.start_delay));
        });

        on<Trigger<ButtonLeftUp>>().then([this] { emit<Scope::INLINE>(std::make_unique<Buzzer>(0)); });
        on<Trigger<ButtonMiddleUp>>().then([this] { emit<Scope::INLINE>(std::make_unique<Buzzer>(0)); });
    }

}  // namespace module::purpose
