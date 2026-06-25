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
#include "Tester.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/localisation/Ball.hpp"
#include "message/planning/KickTo.hpp"
#include "message/planning/LookAround.hpp"
#include "message/skill/GPT.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Say.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/FindBall.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/math/euler.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::purpose {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::localisation::Ball;
    using message::planning::KickTo;
    using message::planning::LookAround;
    using message::skill::GPTAudioRequest;
    using message::skill::GPTChatRequest;
    using message::skill::Look;
    using message::skill::Say;
    using message::skill::Walk;
    using message::strategy::FallRecovery;
    using message::strategy::FindBall;
    using message::strategy::LookAtBall;
    using message::strategy::StandStill;
    using message::strategy::WalkToBall;
    using message::strategy::WalkToFieldPosition;
    using message::strategy::WalkToKickBall;

    using utility::math::euler::pos_rpy_to_transform;
    using utility::support::Expression;

    Tester::Tester(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Tester.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Tester.yaml
            this->log_level                     = config["log_level"].as<NUClear::LogLevel>();
            cfg.find_ball_priority              = config["tasks"]["find_ball_priority"].as<int>();
            cfg.look_at_ball_priority           = config["tasks"]["look_at_ball_priority"].as<int>();
            cfg.walk_to_ball_priority           = config["tasks"]["walk_to_ball_priority"].as<int>();
            cfg.walk_to_kick_ball_priority      = config["tasks"]["walk_to_kick_ball_priority"].as<int>();
            cfg.walk_to_field_position_priority = config["tasks"]["walk_to_field_position_priority"].as<int>();
            cfg.kick_to_priority                = config["tasks"]["kick_to_priority"].as<int>();
            cfg.look_around_priority            = config["tasks"]["look_around_priority"].as<int>();
            cfg.stand_still_priority            = config["tasks"]["stand_still_priority"].as<int>();
            cfg.say_priority                    = config["tasks"]["say_priority"].as<int>();
            cfg.chatgpt_priority                = config["tasks"]["chatgpt_priority"].as<int>();
            cfg.audiogpt_priority               = config["tasks"]["audiogpt_priority"].as<int>();
            cfg.audiogpt_listen_duration        = config["audiogpt_listen_duration"].as<int>();
            cfg.walk_to_field_position_position = config["walk_to_field_position_position"].as<Expression>();
            cfg.say_text                        = config["say_text"].as<std::string>();
            cfg.chatgpt_prompt                  = config["chatgpt_prompt"].as<std::string>();

            cfg.start_delay = config["start_delay"].as<int>();

            // --- DEBUG: print the priorities actually loaded. If editing Tester.yaml does not change
            // these numbers in the log, the config is not being reloaded/applied. If the numbers DO
            // change but behaviour does not, arbitration downstream (e.g. FindBall outranking) is the cause.
            log<INFO>("[Tester] priorities loaded: find_ball=",
                      cfg.find_ball_priority,
                      "look_at_ball=",
                      cfg.look_at_ball_priority,
                      "walk_to_ball=",
                      cfg.walk_to_ball_priority,
                      "walk_to_kick_ball=",
                      cfg.walk_to_kick_ball_priority,
                      "walk_to_field_position=",
                      cfg.walk_to_field_position_priority,
                      "kick_to=",
                      cfg.kick_to_priority,
                      "stand_still=",
                      cfg.stand_still_priority);
        });

        on<Startup>().then([this] {
            // Bootstrap the walk chain exactly like Soccer does. Without an idle Walk task (which fires the
            // Walk skill's on<Start<WalkTask>> -> CurrentWalkTask + WalkState) plus an initial Stability and
            // WalkState, the Director never grants the leg-using providers (WalkToFieldPosition /
            // WalkToKickBall) and the robot never walks. See module/purpose/Soccer/src/Soccer.cpp.
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));
            // Stand idle (priority 0): keeps the legs controlled and bootstraps the Walk skill
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 0);
            // Idle look forward if the head isn't doing anything else
            emit<Task>(std::make_unique<Look>(Eigen::Vector3d::UnitX(), true), 0);
            emit<Scope::DELAY>(std::make_unique<StartTester>(), std::chrono::seconds(cfg.start_delay));
        });

        on<Trigger<StartTester>>().then([this] {
            // Always try to recover from falling, regardless of the test tasks. FallRecovery MUST outrank
            // every configurable test task (find_ball can be 4+), otherwise a search/play task steals the
            // legs mid-getup and the robot abandons getup to search. Soccer can use 2 only because its play
            // tree is 1. Use a high constant (not INT_MAX, to avoid any Director priority-arithmetic overflow).
            constexpr int FALL_RECOVERY_PRIORITY = 1000;
            emit<Task>(std::make_unique<FallRecovery>(), FALL_RECOVERY_PRIORITY);
            on<Every<BEHAVIOUR_UPDATE_RATE, Per<std::chrono::seconds>>, Optional<With<Ball>>>().then(
                [this](const std::shared_ptr<const Ball>& ball) {
                // Ball-dependent approach tasks (WalkToBall/WalkToKickBall) are emitted ONLY once a Ball
                // exists, so the Director grants them (NEW_TASK) with the ball present. Their providers have
                // With<Ball> and run only on grant; emitting them at startup (ball null) means that single run
                // is skipped on the missing With<Ball> and never retried -> robot never approaches. This
                // mirrors how Attack gates WalkToKickBall behind a With<Ball> provider.
                const bool have_ball = ball != nullptr;
                // --- DEBUG: which root tasks are being emitted this cycle (and at what priority)
                log<DEBUG>("[Tester] emitting root tasks: find_ball=",
                           cfg.find_ball_priority,
                           "walk_to_kick_ball=",
                           cfg.walk_to_kick_ball_priority,
                           "look_at_ball=",
                           cfg.look_at_ball_priority);

                // Emit all the tasks with priorities higher than 0
                if (cfg.find_ball_priority > 0) {
                    emit<Task>(std::make_unique<FindBall>(), cfg.find_ball_priority);
                }
                if (cfg.look_at_ball_priority > 0) {
                    emit<Task>(std::make_unique<LookAtBall>(), cfg.look_at_ball_priority);
                }
                if (cfg.walk_to_ball_priority > 0 && have_ball) {
                    emit<Task>(std::make_unique<WalkToBall>(), cfg.walk_to_ball_priority);
                }
                if (cfg.walk_to_kick_ball_priority > 0 && have_ball) {
                    emit<Task>(std::make_unique<WalkToKickBall>(), cfg.walk_to_kick_ball_priority);
                }
                if (cfg.walk_to_field_position_priority > 0) {
                    emit<Task>(std::make_unique<WalkToFieldPosition>(
                                   pos_rpy_to_transform(Eigen::Vector3d(cfg.walk_to_field_position_position.x(),
                                                                        cfg.walk_to_field_position_position.y(),
                                                                        0),
                                                        Eigen::Vector3d(0, 0, cfg.walk_to_field_position_position.z())),
                                   true),
                               cfg.walk_to_field_position_priority);
                }
                if (cfg.kick_to_priority > 0) {
                    emit<Task>(std::make_unique<KickTo>(), cfg.kick_to_priority);
                }
                if (cfg.look_around_priority > 0) {
                    emit<Task>(std::make_unique<LookAround>(), cfg.look_around_priority);
                }
                if (cfg.stand_still_priority > 0) {
                    emit<Task>(std::make_unique<StandStill>(), cfg.stand_still_priority);
                }
                if (cfg.say_priority > 0) {
                    emit<Task>(std::make_unique<Say>(cfg.say_text, true), cfg.say_priority);
                }
                if (cfg.chatgpt_priority > 0) {
                    emit<Task>(std::make_unique<GPTChatRequest>(cfg.chatgpt_prompt, true), cfg.chatgpt_priority);
                }
                if (cfg.audiogpt_priority > 0) {
                    emit<Task>(std::make_unique<GPTAudioRequest>(true, true, cfg.audiogpt_listen_duration),
                               cfg.audiogpt_priority);
                }
            });
        });
    }

}  // namespace module::purpose
