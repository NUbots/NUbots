/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
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

#ifndef MODULES_INPUT_SENSORFILTER_HPP
#define MODULES_INPUT_SENSORFILTER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>
#include <tinyrobotics/kinematics.hpp>
#include <tinyrobotics/parser.hpp>

#include "extension/Configuration.hpp"

#include "message/actuation/BodySide.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/actuation/tinyrobotics.hpp"
#include "utility/input/FrameID.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/math/filter/MahonyFilter.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using utility::actuation::tinyrobotics::forward_kinematics_to_servo_map;
    using utility::actuation::tinyrobotics::sensors_to_configuration;
    using utility::input::FrameID;
    using utility::input::ServoID;
    using utility::math::euler::eul_intrinsic_to_mat;
    using utility::math::euler::mat_to_eul_intrinsic;
    using utility::math::filter::mahony_update;
    using utility::nusight::graph;
    using utility::platform::getRawServo;
    using utility::platform::make_packet_error_string;
    using utility::platform::make_servo_hardware_error_string;
    using utility::support::Expression;

    using message::actuation::BodySide;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using message::localisation::ResetFieldLocalisation;
    using message::platform::ButtonLeftDown;
    using message::platform::ButtonLeftUp;
    using message::platform::ButtonMiddleDown;
    using message::platform::ButtonMiddleUp;
    using message::platform::RawSensors;

    using tinyrobotics::forward_kinematics;

    class SensorFilter : public NUClear::Reactor {
    public:
        explicit SensorFilter(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct FootDownMethod {
            enum Value { UNKNOWN = 0, Z_HEIGHT = 1, FSR = 2 };
            Value value = Value::UNKNOWN;

            // Constructors
            FootDownMethod() = default;
            FootDownMethod(int const& v) : value(static_cast<Value>(v)) {}
            FootDownMethod(Value const& v) : value(v) {}
            FootDownMethod(std::string const& str) {
                // clang-format off
                        if      (str == "Z_HEIGHT") { value = Value::Z_HEIGHT; }
                        else if (str == "FSR")  { value = Value::FSR; }
                        else {
                            value = Value::UNKNOWN;
                            throw std::runtime_error("String " + str + " did not match any enum for FootDownMethod");
                        }
                // clang-format on
            }

            // Conversions
            [[nodiscard]] operator Value() const {
                return value;
            }
            [[nodiscard]] operator std::string() const {
                switch (value) {
                    case Value::Z_HEIGHT: return "Z_HEIGHT";
                    case Value::FSR: return "FSR";
                    default: throw std::runtime_error("enum Method's value is corrupt, unknown value stored");
                }
            }
        };

        struct Config {

            /// @brief Config for the foot down detector
            struct FootDown {
                FootDown() = default;
                FootDown(const FootDownMethod& method, const std::map<FootDownMethod, float>& thresholds) {
                    set_method(method, thresholds);
                }
                void set_method(const FootDownMethod& method, const std::map<FootDownMethod, float>& thresholds) {
                    if (thresholds.count(method) == 0) {
                        throw std::runtime_error(fmt::format("Invalid foot down method '{}'", std::string(method)));
                    }
                    current_method       = method;
                    certainty_thresholds = thresholds;
                }
                [[nodiscard]] float threshold() const {
                    return certainty_thresholds.at(current_method);
                }
                [[nodiscard]] FootDownMethod method() const {
                    return current_method;
                }
                FootDownMethod current_method                        = FootDownMethod::Z_HEIGHT;
                std::map<FootDownMethod, float> certainty_thresholds = {
                    {FootDownMethod::Z_HEIGHT, 0.01f},
                    {FootDownMethod::FSR, 60.0f},
                };
            } foot_down;

            /// @brief The number of times a button must be pressed before it is considered pressed
            int button_debounce_threshold = 0;

            /// @brief Initial transform from torso {t} to world {w} space
            Eigen::Isometry3d initial_Hwt = Eigen::Isometry3d::Identity();

            /// @brief Mahony filter bias
            Eigen::Vector3d initial_bias = Eigen::Vector3d::Zero();

            /// @brief Mahony filter proportional gain
            double Ki = 0.0;

            /// @brief Mahony filter integral gain
            double Kp = 0.0;

            /// @brief Bool to determine whether to use ground truth from the simulator
            bool use_ground_truth = false;
        } cfg;

        /// @brief Number of actuatable joints in the NUgus robot
        static const int n_servos = 20;

        /// @brief tinyrobotics model of NUgus used for kinematics
        tinyrobotics::Model<double, n_servos> nugus_model;

        /// @brief Current support phase of the robot
        WalkState::SupportPhase current_support_phase = WalkState::SupportPhase::LEFT;

        /// @brief Transform from anchor {a} to world {w} space
        Eigen::Isometry3d Hwa = Eigen::Isometry3d::Identity();

        /// @brief Transform from torso {t} to anchor {a} space
        Eigen::Isometry3d Hat = Eigen::Isometry3d::Identity();

        /// @brief Transform from torso {t} to world {w} space using the mahony filter
        Eigen::Isometry3d Hwt_mahony = Eigen::Isometry3d::Identity();

        /// @brief Transform from torso {t} to world {w} space using mahony + anchor method
        Eigen::Isometry3d Hwt_anchor = Eigen::Isometry3d::Identity();

        /// @brief Bias used in the mahony filter, updates with each mahony update
        Eigen::Vector3d bias_mahony = Eigen::Vector3d::Zero();

        /// @brief Current walk command
        Eigen::Vector3d walk_command = Eigen::Vector3d::Zero();

        /// @brief Current state of the left button
        bool left_down = false;

        /// @brief Current state of the middle button
        bool middle_down = false;

        /// @brief Updates the sensors message with raw sensor data, including the timestamp, battery
        /// voltage, servo sensors, accelerometer, gyroscope, buttons, and LED.
        /// @param sensors The sensors message to update
        /// @param previous_sensors The previous sensors message
        /// @param raw_sensors The raw sensor data
        void update_raw_sensors(std::unique_ptr<Sensors>& sensors,
                                const std::shared_ptr<const Sensors>& previous_sensors,
                                const RawSensors& raw_sensors);

        /// @brief Detect when a button has been pressed
        /// @param sensors A vector of previous sensor messages
        void detect_button_press(const std::list<std::shared_ptr<const RawSensors>>& sensors);

        /// @brief Update the sensors message with kinematics data
        /// @param sensors The sensors message to update
        /// @param raw_sensors The raw sensor data
        void update_kinematics(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors);

        /// @brief Display debug information
        /// @param sensors The sensors message to update
        /// @param raw_sensors The raw sensor data
        void debug_sensor_filter(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors);

        /// @brief Updates the sensors message with odometry data filtered using MahonyFilter. This includes the
        // position, orientation, velocity and rotational velocity of the torso in world space.
        /// @param sensors The sensors message to update
        /// @param previous_sensors The previous sensors message
        /// @param raw_sensors The raw sensor data
        void update_odometry(std::unique_ptr<Sensors>& sensors,
                             const std::shared_ptr<const Sensors>& previous_sensors,
                             const RawSensors& raw_sensors,
                             const std::shared_ptr<const WalkState>& walk_state);
    };
}  // namespace module::input
#endif  // MODULES_INPUT_SENSORFILTER_HPP
