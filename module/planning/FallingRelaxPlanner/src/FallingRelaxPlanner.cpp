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
#include "FallingRelaxPlanner.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/input/Sensors.hpp"
#include "message/planning/RelaxWhenFalling.hpp"

#include "utility/skill/Script.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;
    using message::actuation::BodySequence;
    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using message::planning::RelaxWhenFalling;
    using utility::skill::load_script;
    using utility::support::Expression;

    double smooth(double value, double new_value, double alpha) {
        return alpha * value + (1.0 - alpha) * new_value;
    }

    /// @brief A state to categorise each of the properties we are monitoring
    enum class State {
        /// @brief This sensor believes the robot is stable
        STABLE,
        /// @brief This sensor believes the robot is unstable but is unsure if it is falling
        UNSTABLE,
        /// @brief This sensor believes the robot is falling
        FALLING
    };

    FallingRelaxPlanner::FallingRelaxPlanner(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("FallingRelaxPlanner.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.gyro_mag.mean       = config["gyroscope_magnitude"]["mean"].as<Expression>();
            cfg.gyro_mag.unstable   = config["gyroscope_magnitude"]["unstable"].as<Expression>();
            cfg.gyro_mag.falling    = config["gyroscope_magnitude"]["falling"].as<Expression>();
            cfg.gyro_mag.smoothing  = config["gyroscope_magnitude"]["smoothing"].as<Expression>();
            cfg.acc_mag.mean        = config["accelerometer_magnitude"]["mean"].as<Expression>();
            cfg.acc_mag.unstable    = config["accelerometer_magnitude"]["unstable"].as<Expression>();
            cfg.acc_mag.falling     = config["accelerometer_magnitude"]["falling"].as<Expression>();
            cfg.acc_mag.smoothing   = config["accelerometer_magnitude"]["smoothing"].as<Expression>();
            cfg.acc_angle.mean      = config["accelerometer_angle"]["mean"].as<Expression>();
            cfg.acc_angle.unstable  = config["accelerometer_angle"]["unstable"].as<Expression>();
            cfg.acc_angle.falling   = config["accelerometer_angle"]["falling"].as<Expression>();
            cfg.acc_angle.smoothing = config["accelerometer_angle"]["smoothing"].as<Expression>();
            cfg.fall_script         = config["fall_script"].as<std::string>();
        });

        on<Provide<RelaxWhenFalling>, Uses<BodySequence>, Trigger<Sensors>>().then(
            [this](const RunInfo& info, const Uses<BodySequence>& body, const Sensors& sensors) {
                // OTHER_TRIGGER means we ran because of a sensors update
                if (info.run_reason == OTHER_TRIGGER) {
                    auto& a = sensors.accelerometer;
                    auto& g = sensors.gyroscope;

                    // Smooth the values we use to determine if we are falling
                    gyro_mag  = smooth(gyro_mag,
                                      std::abs(std::abs(g.x()) + std::abs(g.y()) + std::abs(g.z()) - cfg.gyro_mag.mean),
                                      cfg.gyro_mag.smoothing);
                    acc_mag   = smooth(acc_mag,  //
                                     std::abs(a.norm() - cfg.acc_mag.mean),
                                     cfg.acc_mag.smoothing);
                    acc_angle = smooth(acc_angle,
                                       std::acos(std::min(1.0, std::abs(a.normalized().z())) - cfg.acc_angle.mean),
                                       cfg.acc_angle.smoothing);

                    // Check if we are stable according to each sensor
                    State gyro_mag_state  = gyro_mag < cfg.gyro_mag.unstable  ? State::STABLE
                                            : gyro_mag < cfg.gyro_mag.falling ? State::UNSTABLE
                                                                              : State::FALLING;
                    State acc_mag_state   = acc_mag < cfg.acc_mag.unstable  ? State::STABLE
                                            : acc_mag < cfg.acc_mag.falling ? State::UNSTABLE
                                                                            : State::FALLING;
                    State acc_angle_state = acc_angle < cfg.acc_angle.unstable  ? State::STABLE
                                            : acc_angle < cfg.acc_angle.falling ? State::UNSTABLE
                                                                                : State::FALLING;

                    // Falling if at least two of the three checks are unstable or if any one of them is falling
                    bool falling = (gyro_mag_state == State::FALLING || acc_mag_state == State::FALLING
                                    || acc_angle_state == State::FALLING)
                                   || (gyro_mag_state == State::UNSTABLE && acc_mag_state == State::UNSTABLE)
                                   || (gyro_mag_state == State::UNSTABLE && acc_angle_state == State::UNSTABLE)
                                   || (acc_mag_state == State::UNSTABLE && acc_angle_state == State::UNSTABLE);

                    // We are falling
                    if (falling) {
                        // We are falling! Relax the limbs!
                        log<NUClear::DEBUG>("Falling:",
                                            "Gyroscope Magnitude:",
                                            gyro_mag,
                                            gyro_mag_state == State::FALLING    ? "FALLING"
                                            : gyro_mag_state == State::UNSTABLE ? "UNSTABLE"
                                                                                : "STABLE",
                                            "Accelerometer Magnitude:",
                                            acc_mag,
                                            acc_mag_state == State::FALLING    ? "FALLING"
                                            : acc_mag_state == State::UNSTABLE ? "UNSTABLE"
                                                                               : "STABLE",
                                            "Accelerometer Angle:",
                                            acc_angle,
                                            acc_angle_state == State::FALLING    ? "FALLING"
                                            : acc_angle_state == State::UNSTABLE ? "UNSTABLE"
                                                                                 : "STABLE");

                        emit(std::make_unique<Stability>(Stability::FALLING));
                        if (body.run_state == GroupInfo::RunState::NO_TASK) {
                            emit<Task>(load_script<BodySequence>(cfg.fall_script));
                        }
                        else {
                            emit<Task>(std::make_unique<Idle>());
                        }
                    }
                }
                else {
                    // Emit an idle task for every other reason
                    emit(std::make_unique<Idle>());
                }
            });
    }

}  // namespace module::planning
