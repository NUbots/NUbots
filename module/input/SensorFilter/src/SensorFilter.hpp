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

#include "MotionModel.hpp"

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
#include "utility/math/filter/UKF.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {


    using utility::math::filter::MahonyFilter;
    using utility::math::filter::UKF;

    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using message::platform::RawSensors;

    class SensorFilter : public NUClear::Reactor {
    public:
        explicit SensorFilter(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Config {
            /// @brief Config for the foot down detector
            struct FootDown {
                /// @brief The type of foot down detector to use (either "FSR" or "Z_HEIGHT")
                std::string method = "UNKNOWN";
                double threshold   = 0.0;
            } foot_down;

            /// @brief The number of times a button must be pressed before it is considered pressed
            int button_debounce_threshold = 0;

            /// @brief Initial rotation from torso {t} to world {w} space
            Eigen::Matrix3d initial_Rwt = Eigen::Matrix3d::Identity();

            /// @brief Mahony filter bias
            Eigen::Vector3d initial_bias = Eigen::Vector3d::Zero();

            /// @brief Mahony filter proportional gain
            double Ki = 0.0;

            /// @brief Mahony filter integral gain
            double Kp = 0.0;

            /// @brief Config for the UKF
            struct UKF {
                struct Noise {
                    Noise() = default;
                    struct Measurement {
                        Eigen::Matrix3d flat_foot_translation = Eigen::Matrix3d::Zero();
                    } measurement{};
                    struct Process {
                        Eigen::Vector3d position = Eigen::Vector3d::Zero();
                        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
                    } process{};
                } noise{};
                struct Initial {
                    Initial() = default;
                    struct Mean {
                        Eigen::Vector3d position = Eigen::Vector3d::Zero();
                        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
                    } mean{};
                    struct Covariance {
                        Eigen::Vector3d position = Eigen::Vector3d::Zero();
                        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
                    } covariance{};
                } initial{};
            } ukf{};

            /// @brief Initial state of the for the UKF filter
            MotionModel<double>::StateVec initial_mean{};

            /// @brief Initial covariance of the for the UKF filter
            MotionModel<double>::StateVec initial_covariance{};

            /// @brief Bool to determine whether to use ground truth from the simulator
            bool use_ground_truth = false;
        } cfg;

        /// @brief Number of actuatable joints in the NUgus robot
        static const int n_servos = 20;

        /// @brief tinyrobotics model of NUgus used for kinematics
        tinyrobotics::Model<double, n_servos> nugus_model;

        /// @brief UKF filter for velocity estimation
        utility::math::filter::UKF<double, MotionModel> ukf{};

        /// @brief Current support phase of the robot
        WalkState::SupportPhase current_support_phase = WalkState::SupportPhase::LEFT;

        /// @brief Transform from anchor {a} to world {w} space
        Eigen::Isometry3d Hwa = Eigen::Isometry3d::Identity();

        /// @brief Mahony filter for orientation (roll and pitch) estimation
        MahonyFilter<double> mahony_filter{};

        /// @brief Bias used in the mahony filter, updates with each mahony update
        Eigen::Vector3d bias_mahony = Eigen::Vector3d::Zero();

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

        /// @brief Updates the sensors message with odometry data filtered using MahonyFilter. This includes the
        // position, orientation, velocity and rotational velocity of the torso in world space.
        /// @param sensors The sensors message to update
        /// @param previous_sensors The previous sensors message
        /// @param raw_sensors The raw sensor data
        void update_odometry(std::unique_ptr<Sensors>& sensors,
                             const std::shared_ptr<const Sensors>& previous_sensors,
                             const RawSensors& raw_sensors,
                             const std::shared_ptr<const WalkState>& walk_state);

        /// @brief Display debug information
        /// @param sensors The sensors message to update
        /// @param raw_sensors The raw sensor data
        void debug_sensor_filter(std::unique_ptr<Sensors>& sensors,
                                 const RawSensors& raw_sensors,
                                 const std::shared_ptr<const WalkState>& walk_state);
    };
}  // namespace module::input
#endif  // MODULES_INPUT_SENSORFILTER_HPP
