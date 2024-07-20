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
#include "WalkToFieldPosition.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::planning::WalkTo;
    using WalkToFieldPositionTask = message::strategy::WalkToFieldPosition;
    using message::skill::Walk;

    WalkToFieldPosition::WalkToFieldPosition(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("WalkToFieldPosition.yaml").then([this](const Configuration& config) {
            // Use configuration here from file WalkToFieldPosition.yaml
            this->log_level       = config["log_level"].as<NUClear::LogLevel>();
            cfg.stop_threshold    = config["stop_threshold"].as<double>();
            cfg.stopped_threshold = config["stopped_threshold"].as<double>();
            cfg.stand_duration    = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["stand_duration"].as<double>()));
        });
        on<Provide<WalkToFieldPositionTask>, Uses<WalkTo>, With<Field>, With<Sensors>>().then(
            [this](const WalkToFieldPositionTask& walk_to_field_position,
                   const Uses<WalkTo>& walk_to,
                   const Field& field,
                   const Sensors& sensors) {
                if (!waiting && walk_to.run_state == GroupInfo::RunState::NO_TASK) {
                    waiting    = true;
                    start_time = NUClear::clock::now();
                    log<NUClear::DEBUG>("Start timer");
                }

                if (NUClear::clock::now() - start_time > cfg.stand_duration) {
                    waiting = false;
                    // Transform from desired field position into robot space
                    Eigen::Isometry3d Hrd = sensors.Hrw * field.Hfw.inverse() * walk_to_field_position.Hfd;
                    emit<Task>(std::make_unique<WalkTo>(Hrd));
                    log<NUClear::DEBUG>("Walk to field position");
                }
                else {
                    log<NUClear::DEBUG>("Stand still");
                    emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()));
                }
            });
    }

}  // namespace module::strategy
