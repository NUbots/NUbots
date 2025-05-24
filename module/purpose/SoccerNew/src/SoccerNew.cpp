/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#include "SoccerNew.hpp"

#include <fmt/format.h>
#include <string>
#include <vector>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Buttons.hpp"
#include "message/input/GameEvents.hpp"
#include "message/input/RoboCup.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/output/Buzzer.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/purpose/FindPurpose.hpp"
#include "message/purpose/Goalie.hpp"
#include "message/purpose/Player.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/StandStill.hpp"

namespace module::purpose {

    using extension::Configuration;

    using Penalisation   = message::input::GameEvents::Penalisation;
    using Unpenalisation = message::input::GameEvents::Unpenalisation;

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::ButtonLeftDown;
    using message::input::ButtonLeftUp;
    using message::input::ButtonMiddleDown;
    using message::input::ButtonMiddleUp;
    using message::input::GameEvents;
    using message::input::RoboCup;
    using message::localisation::ResetFieldLocalisation;
    using message::output::Buzzer;
    using message::platform::ResetWebotsServos;
    using message::purpose::FieldPlayer;
    using message::purpose::FindPurpose;
    using message::purpose::Goalie;
    using message::purpose::Purpose;
    using message::skill::Look;
    using message::skill::Walk;
    using message::strategy::FallRecovery;
    using message::strategy::StandStill;

    SoccerNew::SoccerNew(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("SoccerNew.yaml").then([this](const Configuration& config) {
            this->log_level   = config["log_level"].as<NUClear::LogLevel>();
            cfg.force_playing = config["force_playing"].as<bool>();

            cfg.disable_idle_delay = config["disable_idle_delay"].as<int>();

            cfg.is_goalie = config["is_goalie"].as<bool>();
        });

        // Start the Director graph for the soccer scenario!
        on<Startup>().then([this] {
            // At the start of the program, we should be standing
            // Without these emits, modules that need a Stability and WalkState messages may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));
            // Stand idle
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 0);
            // Idle look forward if the head isn't doing anything else
            emit<Task>(std::make_unique<Look>(Eigen::Vector3d::UnitX(), true), 0);
            // This emit starts the tree to play soccer
            log<INFO>("Start FindPurpose");
            emit<Task>(std::make_unique<FindPurpose>(), 1);
            // The robot should always try to recover from falling, if applicable, regardless of purpose
            emit<Task>(std::make_unique<FallRecovery>(), 2);
        });

        on<Provide<FindPurpose>, Every<1, Per<std::chrono::seconds>>>().then([this] {
            if (cfg.is_goalie) {
                emit<Task>(std::make_unique<Goalie>());
            }
            else {
                emit<Task>(std::make_unique<FieldPlayer>());
            }
        });

        on<Trigger<Penalisation>>().then([this](const Penalisation& self_penalisation) {
            // If the robot is penalised, it must stand still
            if (!cfg.force_playing && self_penalisation.context == GameEvents::Context::SELF) {
                emit(std::make_unique<ResetWebotsServos>());
                emit(std::make_unique<Stability>(Stability::UNKNOWN));
                emit(std::make_unique<ResetFieldLocalisation>());
                emit<Task>(std::unique_ptr<FindPurpose>(nullptr));
            }
        });

        on<Trigger<Unpenalisation>>().then([this](const Unpenalisation& self_unpenalisation) {
            // If the robot is unpenalised, stop standing still and find its purpose
            if (!cfg.force_playing && self_unpenalisation.context == GameEvents::Context::SELF) {
                emit<Task>(std::make_unique<FindPurpose>(), 1);
            }
        });

        // Left button pauses the soccer scenario
        on<Trigger<ButtonLeftDown>>().then([this] {
            emit<Scope::INLINE>(std::make_unique<ResetFieldLocalisation>());
            emit<Scope::INLINE>(std::make_unique<EnableIdle>());
            emit<Scope::INLINE>(std::make_unique<Buzzer>(1000));
            idle = true;
        });

        on<Trigger<ButtonLeftUp>>().then([this] { emit<Scope::INLINE>(std::make_unique<Buzzer>(0)); });

        on<Trigger<EnableIdle>>().then([this] {
            // Stop all tasks and stand still
            emit<Task>(std::unique_ptr<FindPurpose>(nullptr));
            emit<Task>(std::unique_ptr<FallRecovery>(nullptr));
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            log<INFO>("Idle mode enabled");
        });

        // Middle button resumes the soccer scenario
        on<Trigger<ButtonMiddleDown>>().then([this] {
            emit<Scope::INLINE>(std::make_unique<ResetFieldLocalisation>());
            // Restart the Director graph for the soccer scenario after a delay
            emit<Scope::DELAY>(std::make_unique<DisableIdle>(), std::chrono::seconds(cfg.disable_idle_delay));
            emit<Scope::INLINE>(std::make_unique<Buzzer>(1000));
            idle = false;
        });

        on<Trigger<ButtonMiddleUp>>().then([this] { emit<Scope::INLINE>(std::make_unique<Buzzer>(0)); });

        on<Trigger<DisableIdle>>().then([this] {
            // If the robot is not idle, restart the Director graph for the soccer scenario!
            if (!idle) {
                emit<Task>(std::make_unique<FindPurpose>(), 1);
                emit<Task>(std::make_unique<FallRecovery>(), 2);
                log<INFO>("Idle mode disabled");
            }
        });
    }

}  // namespace module::purpose
