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
#include "K1Walk.hpp"

#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/booster/BoosterMode.hpp"
#include "message/booster/BoosterWalk.hpp"
#include "message/eye/DataPoint.hpp"
#include "message/skill/Walk.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::booster::BoosterMode;
    using message::booster::BoosterWalk;
    using message::booster::K1Mode;
    using WalkTask = message::skill::Walk;

    using utility::nusight::graph;
    using utility::support::Expression;

    K1Walk::K1Walk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("K1Walk.yaml").then([this](const Configuration& config) {
            log_level         = config["log_level"].as<NUClear::LogLevel>();
            cfg.kick_velocity = Eigen::Vector3d(config["kick"]["velocity"].as<Expression>());
            cfg.kick_duration = std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["kick"]["duration"].as<double>()));
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
        });

        on<Start<WalkTask>>().then([this]() {
            // Walking requires movement, so request soccer mode from the Booster hardware.
            auto mode  = std::make_unique<BoosterMode>();
            mode->mode = K1Mode::SOCCER;
            emit(std::move(mode));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
        });

        on<Stop<WalkTask>>().then([this] {
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
            auto msg     = std::make_unique<BoosterWalk>();
            msg->velocity = Eigen::Vector3d::Zero();
            emit(std::move(msg));
        });

        // Every lets the kick branch re-run to check whether kick_duration has elapsed
        on<Provide<WalkTask>, Every<30, Per<std::chrono::seconds>>>().then(
            [this](const WalkTask& walk, const RunReason& run_reason) {
                // The Booster SDK has no in-walk kick primitive, so a kick is emulated as a forward velocity
                // burst held for kick_duration. The gait runs inside the SDK, so walk.leg cannot be honoured.
                if (walk.kick) {
                    if (run_reason == RunReason::NEW_TASK) {
                        kick_start_time = NUClear::clock::now();
                        log<INFO>("K1Walk starting in-walk kick");
                    }

                    auto msg      = std::make_unique<BoosterWalk>();
                    msg->velocity = cfg.kick_velocity;
                    emit(std::move(msg));
                    emit(std::make_unique<WalkState>(WalkState::State::WALKING, cfg.kick_velocity));

                    // Finish the kick once the burst duration has elapsed, otherwise keep it running
                    if (NUClear::clock::now() - kick_start_time > cfg.kick_duration) {
                        log<INFO>("K1Walk in-walk kick complete");
                        emit<Task>(std::make_unique<Done>());
                    }
                    else {
                        emit<Task>(std::make_unique<Continue>());
                    }
                    return;
                }

                // Normal walking: only act on a new task to avoid resending unchanged velocity commands
                if (run_reason != RunReason::NEW_TASK) {
                    return;
                }

                auto msg      = std::make_unique<BoosterWalk>();
                msg->velocity = walk.velocity_target;
                emit(std::move(msg));

                const auto state =
                    walk.velocity_target.isZero() ? WalkState::State::STOPPED : WalkState::State::WALKING;
                emit(std::make_unique<WalkState>(state, walk.velocity_target));

                if (log_level <= NUClear::LogLevel::DEBUG) {
                    emit(graph("Walk velocity target",
                               walk.velocity_target.x(),
                               walk.velocity_target.y(),
                               walk.velocity_target.z()));
                    emit(graph("Walk state", int(state)));
                }
            });
    }

}  // namespace module::skill
