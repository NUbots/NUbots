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
#include "FindObject.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/planning/LookAround.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/strategy/FindFeature.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::planning::LookAround;
    using message::planning::TurnOnSpot;
    using message::strategy::FindBall;

    FindObject::FindObject(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("FindObject.yaml").then([this](const Configuration& config) {
            // Use configuration here from file FindObject.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<FindBall>>().then([this] {
            emit<Task>(std::make_unique<LookAround>());
            emit<Task>(std::make_unique<TurnOnSpot>());
        });
    }

}  // namespace module::strategy
