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
#ifndef MODULE_PLANNING_FALLINGRELAXPLANNER_HPP
#define MODULE_PLANNING_FALLINGRELAXPLANNER_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/input/Sensors.hpp"
#include "message/planning/RelaxWhenFalling.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/skill/Script.hpp"

namespace module::planning {

    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using message::planning::RelaxWhenFalling;
    using utility::nusight::graph;
    using utility::skill::load_script;

    class FallingRelaxPlanner : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {

            struct Levels {
                /// @brief The mean value of this sensor to subtract
                double mean = 0.0;
                /// @brief The threshold for this sensor to be considered unstable
                double unstable = 0.0;
                /// @brief The threshold for this sensor to be considered falling
                double falling = 0.0;
                /// @brief The smoothing factor for this sensor
                double smoothing = 0.0;
            };

            /// @brief The configuration for the gyroscope magnitude check
            Levels gyro_mag{};
            /// @brief The configuration for the accelerometer magnitude check
            Levels acc_mag{};
            /// @brief The configuration for the accelerometer angle check
            Levels acc_angle{};

            /// @brief The script to run when falling, set based on the section to relax
            std::string fall_script = "";
        } cfg;


        /// @brief A state to categorise each of the properties we are monitoring
        enum class State {
            /// @brief This sensor believes the robot is stable
            STABLE,
            /// @brief This sensor believes the robot is unstable but is unsure if it is falling
            UNSTABLE,
            /// @brief This sensor believes the robot is falling
            FALLING
        };

        /// @brief the current smoothed value of the gyroscope magnitude
        double gyro_mag = 0.0;
        /// @brief the current smoothed value of the accelerometer magnitude
        double acc_mag = 0.0;
        /// @brief the current smoothed value of the accelerometer angle from upright
        double acc_angle = 0.0;

        template <typename Scalar>
        Scalar smooth(Scalar value, Scalar new_value, Scalar alpha) {
            return alpha * value + (1.0 - alpha) * new_value;
        }

        /// @brief The reactor for falling relax, templated with the sections of the robot to relax
        template <typename BodySection>
        void make_relax() {
            on<Provide<RelaxWhenFalling>, Uses<BodySection>, Trigger<Sensors>>().then(
                [this](const RunReason& run_reason, const Uses<BodySection>& upper_body, const Sensors& sensors) {
                    // If this isn't a sensors update, keep doing what we were doing
                    if (run_reason != RunReason::OTHER_TRIGGER) {
                        emit<Task>(std::make_unique<Continue>());
                        return;
                    }

                    auto& a = sensors.accelerometer;
                    auto& g = sensors.gyroscope;

                    // Smooth the values we use to determine if we are falling
                    gyro_mag  = smooth(gyro_mag,
                                      std::abs(std::abs(g.x()) + std::abs(g.y()) + std::abs(g.z()) - cfg.gyro_mag.mean),
                                      cfg.gyro_mag.smoothing);
                    acc_mag   = smooth(acc_mag,  //
                                     std::abs(a.norm()),
                                     cfg.acc_mag.smoothing);
                    acc_angle = smooth(acc_angle,
                                       std::acos(std::min(1.0, std::abs(a.normalized().z())) - cfg.acc_angle.mean),
                                       cfg.acc_angle.smoothing);


                    // Check if we are stable according to each sensor
                    State gyro_mag_state  = gyro_mag < cfg.gyro_mag.unstable  ? State::STABLE
                                            : gyro_mag < cfg.gyro_mag.falling ? State::UNSTABLE
                                                                              : State::FALLING;
                    State acc_mag_state   = acc_mag > cfg.acc_mag.unstable  ? State::STABLE
                                            : acc_mag > cfg.acc_mag.falling ? State::UNSTABLE
                                                                            : State::FALLING;
                    State acc_angle_state = acc_angle < cfg.acc_angle.unstable  ? State::STABLE
                                            : acc_angle < cfg.acc_angle.falling ? State::UNSTABLE
                                                                                : State::FALLING;

                    // Falling if any 2 of 3 sensors report falling or the angle reports falling
                    bool falling = (gyro_mag_state == State::FALLING && acc_mag_state == State::FALLING)
                                   || (gyro_mag_state == State::FALLING && acc_angle_state == State::FALLING)
                                   || (acc_mag_state == State::FALLING && acc_angle_state == State::FALLING)
                                   || (acc_angle_state == State::FALLING);

                    emit(graph("Falling sensor: x:gyro, y:acc, z:angle",
                               gyro_mag,
                               acc_mag,
                               acc_angle * 180 / 3.14159265358979));
                    emit(graph("Falling", falling));

                    if (falling) {
                        log<DEBUG>("Falling:",
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

                        // We are falling but not running relax yet, run relax
                        if (upper_body.run_state == RunState::NO_TASK) {
                            log<INFO>("Falling!");
                            emit<Task>(load_script<BodySection>(cfg.fall_script));
                        }
                        // Falling and already relaxing, continue relaxing
                        else {
                            emit<Task>(std::make_unique<Continue>());
                        }
                    }
                    // Not falling, but still running, set stability to standing
                    else if (!falling && upper_body.run_state == RunState::RUNNING) {
                        log<INFO>("Unfalling!");
                        emit(std::make_unique<Stability>(Stability::STANDING));
                    }
                    else {
                        log<INFO>("No task");
                    }
                });
        }

    public:
        /// @brief Called by the powerplant to build and setup the FallingRelaxPlanner reactor.
        explicit FallingRelaxPlanner(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_FALLINGRELAXPLANNER_HPP
