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
#include "Dive.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/BodySide.hpp"
#include "message/actuation/Limbs.hpp"
#include "message/skill/Dive.hpp"

#include "utility/skill/Script.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::actuation::BodySequence;
    using message::actuation::BodySide;
    using utility::skill::load_script;
    using DiveTask = message::skill::Dive;

    Dive::Dive(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Dive.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Dive.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<DiveTask>>().then([this](const DiveTask& dive, const RunReason& run_reason) {
            if (run_reason == RunReason::SUBTASK_DONE) {
                emit<Task>(std::make_unique<Done>());
                return;
            }

            if (dive.side == BodySide::LEFT) {
                emit<Task>(load_script<BodySequence>("DiveLeft.yaml"));
            }
            else {
                emit<Task>(load_script<BodySequence>("DiveRight.yaml"));
            }
        });
    }

}  // namespace module::skill
