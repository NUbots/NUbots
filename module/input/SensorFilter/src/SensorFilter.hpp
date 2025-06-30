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
#include <chrono>
#include <nuclear>
#include <tinyrobotics/kinematics.hpp>
#include <tinyrobotics/parser.hpp>

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/math/filter/MadgwickFilter.hpp"
#include "utility/math/filter/MahonyFilter.hpp"

namespace module::input {

    using utility::math::filter::MadgwickFilter;
    using utility::math::filter::MahonyFilter;

    using message::behaviour::state::Stability;
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
            /// @brief Cutoff frequency for the low pass filter of torso x velocity
            double x_cut_off_frequency = 0.0;
            /// @brief Cutoff frequency for the low pass filter of torso y velocity
            double y_cut_off_frequency = 0.0;
            /// @brief Bool to determine whether to use ground truth from the simulator
            bool use_ground_truth = false;
        } cfg;

        /// @brief Number of actuatable joints in the NUgus robot
        static const int n_servos = 20;

        /// @brief tinyrobotics model of NUgus used for kinematics
        tinyrobotics::Model<double, n_servos> nugus_model;

        /// @brief Planted foot associated with the current anchor point (Hwp)
        WalkState::Phase planted_anchor_foot = WalkState::Phase::LEFT;

        /// @brief Transform from planted foot {p} to world {w} space
        Eigen::Isometry3d Hwp = Eigen::Isometry3d::Identity();

        /// @brief Mahony filter for orientation (roll and pitch) estimation
        MadgwickFilter<double> madgwick_filter{};

        /// @brief Bias used in the mahony filter, updates with each mahony update
        Eigen::Vector3d bias = Eigen::Vector3d::Zero();

        /// @brief Previous smoothed accelerometer reading for spike detection
        Eigen::Vector3d prev_smoothed_accel = Eigen::Vector3d::Zero();
        /// @brief Previous smoothed gyroscope reading for spike detection
        Eigen::Vector3d prev_smoothed_gyro = Eigen::Vector3d::Zero();

        /// @brief Learned gyro bias that persists across filter operations
        Eigen::Vector3d learned_gyro_bias = Eigen::Vector3d::Zero();
        /// @brief Learning rate for adaptive bias estimation
        double bias_learning_rate = 0.0001;  // Very slow learning
        /// @brief Time when filter was last initialized
        std::chrono::steady_clock::time_point filter_start_time = std::chrono::steady_clock::now();
        /// @brief Minimum time before bias learning starts (seconds)
        double bias_learning_delay = 10.0;
        /// @brief Maximum acceleration change threshold for spike detection
        double accel_spike_threshold = 5.0;  // m/s²
        /// @brief Maximum gyroscope change threshold for spike detection
        double gyro_spike_threshold = 2.0;  // rad/s
        /// @brief Smoothing factor for sensor low-pass filtering (0-1, higher = more smoothing)
        double sensor_smoothing_factor = 0.85;
        /// @brief Previous orientation for stability checking
        Eigen::Quaterniond prev_orientation = Eigen::Quaterniond::Identity();
        /// @brief Maximum orientation change per second (rad/s) before considering it unstable
        double max_orientation_change_rate = 1.0;

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
                             const Stability& stability);

        /// @brief Display debug information
        /// @param sensors The sensors message to update
        /// @param raw_sensors The raw sensor data
        void debug_sensor_filter(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors);

        /// @brief Save learned bias to a file for persistence across restarts
        void save_learned_bias();
    };
}  // namespace module::input
#endif  // MODULES_INPUT_SENSORFILTER_HPP
