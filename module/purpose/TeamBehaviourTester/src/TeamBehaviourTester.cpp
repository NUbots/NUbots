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
#include "TeamBehaviourTester.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/purpose/Player.hpp"
#include "message/purpose/SupportPosition.hpp"

namespace module::purpose {

    using extension::Configuration;

    using message::input::GameState;
    using message::purpose::FieldPlayer;
    using message::purpose::Support;
    using message::purpose::SupportPosition;

    TeamBehaviourTester::TeamBehaviourTester(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("TeamBehaviourTester.yaml").then([this](const Configuration& config) {
            // Use configuration here from file TeamBehaviourTester.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        // Continuously pretend we are in the PLAYING phase of a normal game and request the FieldPlayer
        // purpose. This makes FieldPlayer run its real in-game decision logic (based on the ball, teammates
        // and field localisation) and emit the Purpose (role) it would take, along with a SupportPosition
        // when that role is Support. These are visualised in NUsight. The robot is still driven around by
        // KeyboardWalk - FieldPlayer's own locomotion tasks (Attack, Defend, Support, ...) have no providers
        // in this role, so they are inert and the robot never moves on its own.
        on<Every<BEHAVIOUR_UPDATE_RATE, Per<std::chrono::seconds>>>().then([this] {
            // Synthesise a sensible "we are playing a normal game" state so FieldPlayer's PLAYING
            // reaction fires. Without a GameController on the network the fields default to safe values
            // (not stopped, unpenalised, our kick off so we are allowed to attack).
            auto game_state              = std::make_unique<GameState>();
            game_state->phase            = GameState::Phase::PLAYING;
            game_state->mode             = GameState::Mode::NORMAL;
            game_state->our_kick_off     = true;
            game_state->stopped          = false;
            game_state->team.team_colour = GameState::TeamColour::BLUE;
            game_state->self.penalty_reason = GameState::PenaltyReason::UNPENALISED;
            emit(game_state);

            // FieldPlayer's When<Phase, ...> guard reads the standalone Phase message, so emit it too
            emit(std::make_unique<GameState::Phase>(GameState::Phase::PLAYING));

            // Keep the FieldPlayer purpose active so it re-evaluates and reports the role every tick
            emit<Task>(std::make_unique<FieldPlayer>());

            // Also always run the Support calculation so NUsight shows the position this robot *would*
            // walk to if it were supporting, even when its current role is something else (e.g. attack).
            // Its WalkToFieldPosition task has no provider in this role, so it only emits the debug
            // SupportPosition and never actually moves the robot.
            emit<Task>(std::make_unique<Support>());
        });

        // Diagnostic: log the computed support position whenever the Support module emits one. If this
        // never fires, the Support calculation is not running (e.g. field not localised yet, or this
        // robot's player_id has no slot in Formation.yaml). Set log_level to DEBUG to see it.
        on<Trigger<SupportPosition>>().then([this](const SupportPosition& sp) {
            log<DEBUG>("Computed support position: x =", sp.position.x(), "y =", sp.position.y());
        });
    }

}  // namespace module::purpose
