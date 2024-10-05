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
#include "FootController.hpp"

#include "extension/Configuration.hpp"

namespace module::actuation {

    using extension::Configuration;


    FootController::FootController(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("FootController.yaml").then([this](const Configuration& config) {
            // Set log level from the configuration
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.mode        = config["mode"].as<std::string>();

            cfg.desired_gains = config["servo_gains"].as<std::map<std::string, double>>();
            // Set gains of servo to startup phase values
            cfg.servos.clear();
            cfg.startup_gain = config["startup"]["servo_gain"].as<double>();
            for (const auto& [key, gain] : cfg.desired_gains) {
                auto new_servo                 = Servo();
                new_servo.goal.position_p_gain = cfg.startup_gain;
                new_servo.goal.torque_enabled  = true;
                cfg.servos[ServoID(key)]       = new_servo;
            }
            // Emit request to set desired gains after a delay
            emit<Scope::DELAY>(std::make_unique<SetGains>(),
                               std::chrono::seconds(config["startup"]["duration"].as<int>()));
        });

        on<Trigger<SetGains>>().then([this] {
            for (const auto& [key, gain] : cfg.desired_gains) {
                auto new_servo                 = Servo();
                new_servo.goal.position_p_gain = gain;
                new_servo.goal.torque_enabled  = true;
                cfg.servos[ServoID(key)]       = new_servo;
            }
        });


        on<Provide<ControlLeftFoot>, With<Sensors>, Needs<LeftLegIK>, Priority::HIGH>().then(
            [this](const ControlLeftFoot& left_foot, const Sensors& sensors) {
                auto left_leg = std::make_unique<LeftLegIK>();

                control_foot(left_foot, left_leg, sensors, LimbID::LimbID::LEFT_LEG);

                // Emit IK tasks to achieve the desired pose
                emit<Task>(left_leg, 0, false, "Control left foot");
            });

        on<Provide<ControlRightFoot>, With<Sensors>, Needs<RightLegIK>, Priority::HIGH>().then(
            [this](const ControlRightFoot& right_foot, const Sensors& sensors) {
                auto right_leg = std::make_unique<RightLegIK>();

                control_foot(right_foot, right_leg, sensors, LimbID::RIGHT_LEG);

                // Emit IK tasks to achieve the desired pose
                emit<Task>(right_leg, 0, false, "Control right foot");
            });
    }

}  // namespace module::actuation
