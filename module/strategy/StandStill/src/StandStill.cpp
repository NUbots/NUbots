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
#include "StandStill.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/skill/Walk.hpp"
#include "message/strategy/StandStill.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::skill::Walk;
    using StandStillTask = message::strategy::StandStill;

    StandStill::StandStill(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("StandStill.yaml").then([this](const Configuration& config) {
            // Use configuration here from file StandStill.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<StandStillTask>>().then([this](const RunInfo& info) {
            // If we haven't emitted yet, then emit a walk task
            if (info.run_reason == RunReason::NEW_TASK) {
                emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()));
            }
            else {
                emit<Task>(std::make_unique<Idle>());
            }
        });
    }

}  // namespace module::strategy
