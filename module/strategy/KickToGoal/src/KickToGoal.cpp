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
#include "KickToGoal.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/KickTo.hpp"
#include "message/strategy/KickToGoal.hpp"
#include "message/support/FieldDescription.hpp"

namespace module::strategy {

    using extension::Configuration;
    using KickToGoalTask = message::strategy::KickToGoal;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::planning::KickTo;
    using message::support::FieldDescription;

    KickToGoal::KickToGoal(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("KickToGoal.yaml").then([this](const Configuration& config) {
            // Use configuration here from file KickToGoal.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<KickToGoalTask>,
           With<Field>,
           With<Sensors>,
           With<FieldDescription>,
           Every<30, Per<std::chrono::seconds>>>()
            .then([this](const Field& field, const Sensors& sensors, const FieldDescription& field_description) {
                // Get the robot's position (pose) on the field
                Eigen::Isometry3d Hrf =
                    Eigen::Isometry3d(sensors.Hrw) * Eigen::Isometry3d(field.Hfw.inverse().cast<double>());

                // Get the goal position relative to the robot to kick to
                Eigen::Vector3d rGFf = Eigen::Vector3d(-field_description.dimensions.field_length / 2.0, 0.0, 0.0);
                Eigen::Vector3d rGRr = Hrf * rGFf;

                emit<Task>(std::make_unique<KickTo>(rGRr));  // kick the ball if possible
            });
    }

}  // namespace module::strategy
