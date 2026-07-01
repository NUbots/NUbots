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

namespace module::skill {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::booster::BoosterMode;
    using message::booster::BoosterWalk;
    using message::booster::K1Mode;
    using WalkTask = message::skill::Walk;

    using utility::nusight::graph;

    K1Walk::K1Walk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("K1Walk.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();
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

        on<Provide<WalkTask>>().then([this](const WalkTask& walk, const RunReason& run_reason) {
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
