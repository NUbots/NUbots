/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#include "OnboardWalkOptimisation.hpp"

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/support/optimisation/OptimisationCommand.hpp"
#include "message/support/optimisation/OptimisationResetDone.hpp"
#include "message/support/optimisation/OptimisationRobotPosition.hpp"

namespace module::support::optimisation {

    using extension::Configuration;
    using message::input::Sensors;
    using message::support::optimisation::OptimisationCommand;
    using message::support::optimisation::OptimisationResetDone;
    using message::support::optimisation::OptimisationRobotPosition;

    OnboardWalkOptimisation::OnboardWalkOptimisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("OnboardWalkOptimisation.yaml").then([this](const Configuration& config) {
            this->log_level    = config["log_level"].as<NUClear::LogLevel>();
            cfg.standing_angle = config["standing_angle"].as<float>();
            cfg.wait_time      = config["wait_time"].as<float>();
        });

        on<Trigger<OptimisationCommand>>().then([this](const OptimisationCommand& msg) {
            if (msg.command == OptimisationCommand::CommandType::RESET_ROBOT) {
                NUClear::log<NUClear::LogLevel::DEBUG>("Onboard Resetting!");
                is_upright = false;
                resetting  = true;
            }
        });

        on<Trigger<Sensors>>().then("Stability Reset Check", [this](const Sensors& sensors) {
            if (resetting) {
                // Length of time from first standing upright
                const auto time_waited =
                    std::chrono::duration_cast<std::chrono::duration<float>>(NUClear::clock::now() - start_time)
                        .count();
                // Trigger the next individual after standing for the config value cfg.wait_time
                if (is_upright && (time_waited > cfg.wait_time)) {
                    NUClear::log<NUClear::LogLevel::DEBUG>(
                        fmt::format("Stood upright for {:.1f} seconds. Reset done.", time_waited));
                    resetting  = false;
                    is_upright = false;
                    emit(std::make_unique<OptimisationResetDone>());
                }

                // Transform to torso {t} from world {w} space
                Eigen::Matrix4d Hwt = sensors.Htw.inverse().matrix();
                // Basis Z vector of torso {t} in world {w} space
                Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);

                // Check if angle between torso z axis and world z axis is greater than config value cfg.fallen_angle
                if (!is_upright && std::acos(Eigen::Vector3d::UnitZ().dot(uZTw)) < cfg.standing_angle) {
                    is_upright = true;
                    // Start timer
                    start_time = NUClear::clock::now();
                    NUClear::log<NUClear::LogLevel::DEBUG>(fmt::format("Standing for {} seconds.", cfg.wait_time));
                }
                else if (is_upright && !(std::acos(Eigen::Vector3d::UnitZ().dot(uZTw)) < cfg.standing_angle)) {
                    is_upright = false;
                    NUClear::log<NUClear::LogLevel::DEBUG>("No longer standing.");
                }
            }
            else {
                // Calculate and emit an odometry position vector
                Eigen::Isometry3d Hwt = sensors.Htw.inverse();
                Eigen::Vector3d rTWw  = Hwt.translation();

                auto robot_position   = std::make_unique<OptimisationRobotPosition>();
                robot_position->value = rTWw;
                emit(robot_position);
            }
        });
    }
}  // namespace module::support::optimisation
