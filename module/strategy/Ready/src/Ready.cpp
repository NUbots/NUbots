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
#include "Ready.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/skill/Walk.hpp"
#include "message/strategy/Ready.hpp"
// #include "message/strategy/StandStill.hpp"
#include "message/behaviour/state/Stability.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::skill::Walk;
    using ReadyTask = message::strategy::Ready;
    // using message::strategy::StandStill;
    using message::behaviour::state::Stability;

    Ready::Ready(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Ready.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Ready.yaml
            this->log_level        = config["log_level"].as<NUClear::LogLevel>();
            cfg.walk_to_ready_time = std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["walk_to_ready_time"].as<double>()));
            cfg.walk_to_ready_speed_x  = config["walk_to_ready_speed_x"].as<double>();
            cfg.walk_to_ready_speed_y  = config["walk_to_ready_speed_y"].as<double>();
            cfg.walk_to_ready_rotation = config["walk_to_ready_rotation"].as<double>();
        });

        on<Provide<ReadyTask>, With<Stability>>().then([this](const RunReason& info, const Stability& stability) {
            if (info.run_reason == RunReason::NEW_TASK) {
                // Set the timer and emit a walk Task
                start_ready_time = NUClear::clock::now();
                emit<Task>(std::make_unique<Walk>(
                    Eigen::Vector3d(cfg.walk_to_ready_speed_x, cfg.walk_to_ready_speed_y, cfg.walk_to_ready_rotation)));
            }
            // If the time has elapsed to walk to ready, then emit the stand still task
            // Don't emit another stand still task if we already did so
            else if (NUClear::clock::now() - start_ready_time > cfg.walk_to_ready_time
                     && stability != Stability::STANDING) {
                emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()));
            }
            else {  // Otherwise, emit the idle task to keep walking or standing still
                emit<Task>(std::make_unique<Idle>());
            }
        });
    }

}  // namespace module::strategy
