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
#include "message/planning/KickTo.hpp"
#include "message/planning/LookAround.hpp"
#include "message/skill/GPT.hpp"
#include "message/skill/Say.hpp"
#include "message/strategy/AlignBallToGoal.hpp"
#include "message/strategy/FindFeature.hpp"
#include "message/strategy/KickToGoal.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/Ready.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/math/euler.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::purpose {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::planning::KickTo;
    using message::planning::LookAround;
    using message::skill::GPTAudioRequest;
    using message::skill::GPTChatRequest;
    using message::skill::Say;
    using message::strategy::AlignBallToGoal;
    using message::strategy::FindBall;
    using message::strategy::KickToGoal;
    using message::strategy::LookAtBall;
    using message::strategy::Ready;
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
            cfg.align_ball_to_goal_priority     = config["tasks"]["align_ball_to_goal_priority"].as<int>();
            cfg.kick_to_goal_priority           = config["tasks"]["kick_to_goal_priority"].as<int>();
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
        });

        on<Startup>().then([this] {
            emit<Scope::DELAY>(std::make_unique<StartTester>(), std::chrono::seconds(cfg.start_delay));
            emit(std::make_unique<Stability>(Stability::STANDING));
        });

        on<Trigger<StartTester>>().then([this] {
            on<Every<BEHAVIOUR_UPDATE_RATE, Per<std::chrono::seconds>>>().then([this] {
                // Emit all the tasks with priorities higher than 0
                if (cfg.find_ball_priority > 0) {
                    emit<Task>(std::make_unique<FindBall>(), cfg.find_ball_priority);
                }
                if (cfg.look_at_ball_priority > 0) {
                    emit<Task>(std::make_unique<LookAtBall>(), cfg.look_at_ball_priority);
                }
                if (cfg.walk_to_ball_priority > 0) {
                    emit<Task>(std::make_unique<WalkToBall>(), cfg.walk_to_ball_priority);
                }
                if (cfg.walk_to_kick_ball_priority > 0) {
                    log<NUClear::INFO>("Walk to kick ball");
                    emit<Task>(std::make_unique<WalkToKickBall>(), cfg.walk_to_kick_ball_priority);
                }
                if (cfg.align_ball_to_goal_priority > 0) {
                    emit<Task>(std::make_unique<AlignBallToGoal>(), cfg.align_ball_to_goal_priority);
                }
                if (cfg.kick_to_goal_priority > 0) {
                    emit<Task>(std::make_unique<KickToGoal>(), cfg.kick_to_goal_priority);
                }
                if (cfg.walk_to_field_position_priority > 0) {
                    emit<Task>(std::make_unique<WalkToFieldPosition>(pos_rpy_to_transform(
                                   Eigen::Vector3d(cfg.walk_to_field_position_position.x(),
                                                   cfg.walk_to_field_position_position.y(),
                                                   0),
                                   Eigen::Vector3d(0, 0, cfg.walk_to_field_position_position.z()))),
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
