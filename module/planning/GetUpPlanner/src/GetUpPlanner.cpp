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
                if (getup.run_state == RunState::RUNNING && !getup.done) {
                    emit<Task>(std::make_unique<Continue>());
                    log<DEBUG>("Idle");
                    return;
                }
                // Transform to torso{t} from world{w} space
                Eigen::Matrix4d Hwt = sensors.Htw.inverse().matrix();
                // Basis Z vector of torso {t} in world {w} space
                Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);

                // Get the angle of the robot with the world z axis
                double angle = std::acos(Eigen::Vector3d::UnitZ().dot(uZTw));
                log<DEBUG>("Angle: ", angle);

                // // Check if angle between torso z axis and world z axis is greater than config value
                // Only emit if we're not already requesting a getup
                if (angle > cfg.fallen_angle && getup.run_state == RunState::NO_TASK) {
                    emit<Task>(std::make_unique<GetUp>());
                    log<DEBUG>("Execute getup");
                }
                // Otherwise do not need to get up so emit no tasks
            });
    }

}  // namespace module::planning
