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
#include "GetUpPlanner.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/planning/GetUpWhenFallen.hpp"
#include "message/skill/GetUp.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;
    using message::input::Sensors;
    using message::planning::GetUpWhenFallen;
    using message::skill::GetUp;
    using utility::support::Expression;

    GetUpPlanner::GetUpPlanner(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("GetUpPlanner.yaml").then([this](const Configuration& config) {
            this->log_level  = config["log_level"].as<NUClear::LogLevel>();
            cfg.fallen_angle = config["fallen_angle"].as<float>();
        });

        on<Provide<GetUpWhenFallen>, Uses<GetUp>, Trigger<Sensors>>().then(
            [this](const Uses<GetUp>& getup, const Sensors& sensors) {
                // Orientation of the robot relative to upright (cheap, computed every tick so the
                // status string below is always current).
                Eigen::Matrix4d Hwt  = sensors.Htw.inverse().matrix();
                Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);
                const double angle   = std::acos(Eigen::Vector3d::UnitZ().dot(uZTw));
                const bool fallen    = angle > cfg.fallen_angle;

                const char* state_str = getup.run_state == RunState::RUNNING  ? "RUNNING"
                                        : getup.run_state == RunState::QUEUED ? "QUEUED"
                                                                              : "NO_TASK";

                // Edge-triggered status: log only when the (orientation, GetUp state) situation
                // changes, not on every sensor tick.
                std::string status =
                    std::string(fallen ? "fallen" : "upright") + " getup=" + state_str + (getup.done ? "(done)" : "");
                if (status != last_status) {
                    log<DEBUG>("GetUpPlanner: ", status, " angle=", angle, " rad");
                    last_status = status;
                }

                if (getup.run_state == RunState::RUNNING && !getup.done) {
                    emit<Task>(std::make_unique<Continue>());
                    return;
                }

                // Only emit if we're fallen and not already requesting a getup
                if (fallen && getup.run_state == RunState::NO_TASK) {
                    emit<Task>(std::make_unique<GetUp>());
                    log<INFO>("GetUpPlanner: dispatching GetUp (angle=", angle, " rad)");
                }
                // Otherwise upright (no task needed) or fallen-but-not-idle — covered by the status
                // log above, so emit nothing here.
            });
    }

}  // namespace module::planning
