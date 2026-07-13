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
#include "K1GetUp.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/booster/BoosterFallDownState.hpp"
#include "message/booster/BoosterGetUp.hpp"
#include "message/skill/GetUp.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::behaviour::state::Stability;
    using message::booster::BoosterFallDownState;
    using message::booster::FallDownStateType;
    using message::booster::BoosterGetUp;
    using GetUpTask = message::skill::GetUp;

    K1GetUp::K1GetUp(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("K1GetUp.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.retry_delay = config["retry_delay"].as<double>();
        });

        on<Provide<GetUpTask>, With<BoosterFallDownState>>().then(
            [this](const RunReason& run_reason, const BoosterFallDownState& fall_state) {
                // Re-request on a cooldown, not just NEW_TASK: if the get-up fails (or the robot
                // falls again straight after), fall_down_state returns to HAS_FALLEN while this
                // task is still running — waiting for IS_READY alone deadlocks with no retry.
                // The cooldown guards against spamming the RPC while the firmware/sim is mid
                // get-up (which reports IS_GETTING_UP, not HAS_FALLEN, so it never retriggers).
                if (fall_state.fall_down_state == FallDownStateType::HAS_FALLEN
                    && fall_state.is_recovery_available
                    && (run_reason == RunReason::NEW_TASK
                        || NUClear::clock::now() - last_request
                               > std::chrono::duration_cast<NUClear::clock::duration>(
                                   std::chrono::duration<double>(cfg.retry_delay)))) {
                    log<INFO>("Getting up...");
                    emit(std::make_unique<BoosterGetUp>());
                    last_request = NUClear::clock::now();
                }

                if (fall_state.fall_down_state == FallDownStateType::IS_READY) {
                    log<INFO>("Finished getting up");
                    emit(std::make_unique<Stability>(Stability::STANDING));
                    emit<Task>(std::make_unique<Done>());
                }
                else {
                    emit<Task>(std::make_unique<Continue>());
                }
            });
    }

}  // namespace module::skill
