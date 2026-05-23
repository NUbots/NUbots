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
        });

        on<Provide<GetUpTask>, With<BoosterFallDownState>>().then(
            [this](const RunReason& run_reason, const BoosterFallDownState& fall_state) {
                if (run_reason == RunReason::NEW_TASK
                    && fall_state.fall_down_state == FallDownStateType::HAS_FALLEN
                    && fall_state.is_recovery_available) {
                    log<INFO>("Getting up...");
                    emit(std::make_unique<BoosterGetUp>());
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
