/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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
#include "KickWalk.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/skill/Kick.hpp"
#include "message/skill/Walk.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::skill::Kick;
    using message::skill::Walk;

    KickWalk::KickWalk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {
        on<Configuration>("KickWalk.yaml").then([this](const Configuration& config) {
            // Use configuration here from file KickWalk.yaml
            this->log_level              = config["log_level"].as<NUClear::LogLevel>();
            cfg.kick_approach_velocity_x = config["kick_approach_velocity_x"].as<double>();
            cfg.kick_approach_velocity_y = config["kick_approach_velocity_y"].as<double>();
            cfg.new_kick_wait_time       = config["new_kick_wait_time"].as<double>();
        });

        on<Provide<Kick>, Uses<Walk>>().then(
            [this](const Kick& kick, const RunReason& run_reason, const Uses<Walk>& walk) {
                if (run_reason == RunReason::NEW_TASK) {
                    if (NUClear::clock::now() - last_kick_end < std::chrono::seconds(cfg.new_kick_wait_time)) {
                        log<DEBUG>("KickWalk is waiting for cooldown after last kick.");
                        NUClear::clock::time_point wait_until =
                            last_kick_end + std::chrono::seconds(cfg.new_kick_wait_time);
                        emit<Task>(std::make_unique<Wait>(wait_until));
                        return;
                    }

                    log<DEBUG>("KickWalk module received a kick request for leg: ", kick.leg);

                    // Slow approach
                    Eigen::Vector3d walk_velocity =
                        Eigen::Vector3d(cfg.kick_approach_velocity_x, cfg.kick_approach_velocity_y, 0.0);
                    emit<Task>(std::make_unique<Walk>(walk_velocity, true, kick.leg));
                }
                // The wait triggered the provider
                else if (run_reason == RunReason::SUBTASK_DONE && !walk.done) {
                    // Slow approach
                    log<DEBUG>("KickWalk module received a kick request for leg: ", kick.leg);
                    Eigen::Vector3d walk_velocity =
                        Eigen::Vector3d(cfg.kick_approach_velocity_x, cfg.kick_approach_velocity_y, 0.0);
                    emit<Task>(std::make_unique<Walk>(walk_velocity, true, kick.leg));
                }

                // If the walk says the kick is done, emit a Done task
                if (run_reason == RunReason::SUBTASK_DONE) {
                    log<DEBUG>("KickWalk step completed, kick done for leg: ", kick.leg);
                    last_kick_end = NUClear::clock::now();
                    emit<Task>(std::make_unique<Done>());
                    return;
                }
            });
    }

}  // namespace module::skill
