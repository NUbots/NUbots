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
#include "FallingRelaxPlanner.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;
    using message::actuation::ArmsSequence;
    using message::actuation::BodySequence;
    using message::actuation::HeadSequence;
    using message::actuation::UpperBodySequence;
    using utility::support::Expression;

    FallingRelaxPlanner::FallingRelaxPlanner(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("FallingRelaxPlanner.yaml").then([this](const Configuration& config) {
            this->log_level        = config["log_level"].as<NUClear::LogLevel>();
            std::string relax_part = config["relax"].as<std::string>();

            if (relax_part == "Body") {
                cfg.fall_script = "Relax.yaml";
                make_relax<BodySequence>();
            }
            else if (relax_part == "UpperBody") {
                cfg.fall_script = "RelaxUpperBody.yaml";
                make_relax<UpperBodySequence>();
            }
            else if (relax_part == "Arms") {
                cfg.fall_script = "RelaxArms.yaml";
                make_relax<ArmsSequence>();
            }
            else if (relax_part == "Head") {
                cfg.fall_script = "RelaxHead.yaml";
                make_relax<HeadSequence>();
            }
            else {
                log<ERROR>("Unknown relax part: ", relax_part);
            }
        });
    }

}  // namespace module::planning
