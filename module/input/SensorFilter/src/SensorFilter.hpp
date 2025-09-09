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
#include <optional>
#include <deque>  // Add this for std::deque
#include <chrono>  // Add this for timestamps
#include <tinyrobotics/kinematics.hpp>
#include <tinyrobotics/parser.hpp>

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Sensors.hpp"
#include "message/input/Stella.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/RawSensors.hpp"


#include "utility/math/filter/KalmanFilter.hpp"
#include "utility/math/filter/MahonyFilter.hpp"
#include "utility/math/filter/YawFilter.hpp"

namespace module::input {

    using utility::math::filter::MahonyFilter;
    using utility::math::filter::YawFilter;
    using utility::math::filter::KalmanFilter;

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using StellaMsg = message::input::Stella;
    using message::localisation::RobotPoseGroundTruth;
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

            /// @brief Adaptive Mahony filter gains based on stability state
            struct AdaptiveGains {
                /// @brief Kp gain for standing state
                double standing_Kp = 0.0;
                /// @brief Kp gain for dynamic states
                double dynamic_Kp = 0.0;
            } adaptive_gains;

            /// @brief The number of times a button must be pressed before it is considered pressed
            int button_debounce_threshold = 0;
            /// @brief Cutoff frequency for the low pass filter of torso x velocity
            double x_cut_off_frequency = 0.0;
            /// @brief Cutoff frequency for the low pass filter of torso y velocity
            double y_cut_off_frequency = 0.0;
            /// @brief Bool to determine whether to use ground truth from the simulator
            bool use_ground_truth = false;

            /// @brief Z-bias filter configuration
            struct StellaConfig {
                /// @brief Process noise covariance for z-bias estimation
                double process_noise = 0.001;
                /// @brief Measurement noise covariance for z-bias estimation
                double measurement_noise = 0.05;
                /// @brief Initial covariance for z-bias state
                double initial_covariance = 1.0;
                /// @brief Delay before Stella initialization (seconds)
                double initialization_delay = 10.0;
                /// @brief Alpha parameter for scale factor exponential filtering
                double scale_factor_alpha = 0.9;
            } stella_config;

            /// @brief Sliding window configuration
            struct SlidingWindowConfig {
                bool enabled = false;
                int window_size = 10;
                double smoothness_weight = 1.0;
                double stella_base_weight = 1.0;
                double kinematic_base_weight = 1.0;
                double xtol_rel = 1e-6;
                double ftol_rel = 1e-6;
                int maxeval = 100;
            } sliding_window;
        } cfg;

        /// @brief Number of actuatable joints in the NUgus robot
        static const int n_servos = 20;

        /// @brief tinyrobotics model of NUgus used for kinematics
        tinyrobotics::Model<double, n_servos> nugus_model;

        /// @brief Planted foot associated with the current anchor point (Hwp)
        WalkState::Phase planted_anchor_foot = WalkState::Phase::LEFT;

        /// @brief Transform from planted foot {p} to world {w} space
        Eigen::Isometry3d Hwp = Eigen::Isometry3d::Identity();

        double scale_factor = 1.0;

        /// @brief Stella initialization states
        enum class StellaState {
            WAITING_FOR_POINTS,    // Waiting for first map points
            WAITING_FOR_DELAY,     // Got points, waiting for 10s delay
            INITIALIZED            // Ready to use
        };

        StellaState stella_state = StellaState::WAITING_FOR_POINTS;
        std::optional<std::chrono::steady_clock::time_point> stella_start_time;

        /// @brief Mahony filter for orientation (roll and pitch) estimation
        MahonyFilter<double> mahony_filter{};

        /// @brief Yaw filter for fusing gyroscope and kinematic estimates
        YawFilter<double> yaw_filter{};

        /// @brief Bias used in the mahony filter, updates with each mahony update
        Eigen::Vector3d bias_mahony = Eigen::Vector3d::Zero();

        /// @brief Current state of the left button
        bool left_down = false;
        /// @brief Current state of the middle button
        bool middle_down = false;

        /// @brief Bool indicating if the ground truth is initialised
        bool ground_truth_initialised = false;

        /// @brief Ground truth Hfw
        Eigen::Isometry3d ground_truth_Hfw = Eigen::Isometry3d::Identity();

        /// @brief Fixed transform from nubots world to stella world frame
        Eigen::Isometry3d Hwn = Eigen::Isometry3d::Identity();

        /// @brief 1D Kalman filter for estimating z-bias between Stella and anchor point methods
        /// State: [z_bias], Input: [], Measurement: [z_diff]
        KalmanFilter<double, 1, 0, 1> z_bias_filter{};

        /// @brief Flag to track if z-bias filter has been initialized
        bool z_bias_filter_initialized = false;

        /// @brief Measurement factor for sliding window optimization
        struct MeasurementFactor {
            enum Type { STELLA, KINEMATIC };
            Type type;
            int time_index;
            Eigen::Vector2d measurement;
            double weight;
            std::chrono::steady_clock::time_point timestamp;
        };

        /// @brief Sliding window pose history
        std::deque<Eigen::Vector2d> pose_window_;

        /// @brief Current factors for optimization (used by objective function)
        std::vector<MeasurementFactor> current_factors_;

        /// @brief Fault detection status
        struct FaultDetectionStatus {
            bool stella_healthy = true;
            bool kinematics_healthy = true;
            double stella_confidence = 1.0;
            double kinematics_confidence = 1.0;
        };

        /// @brief Optimize sliding window states
        std::pair<Eigen::VectorXd, double> optimize_sliding_window(
            const Eigen::VectorXd& initial_guess,
            const std::vector<MeasurementFactor>& factors);

        /// @brief Compute cost for sliding window optimization
        double compute_sliding_window_cost(const Eigen::VectorXd& states);

        /// @brief Update odometry with sliding window optimization
        void update_odometry_sliding_window(std::unique_ptr<Sensors>& sensors,
                                           const std::shared_ptr<const Sensors>& previous_sensors,
                                           const RawSensors& raw_sensors,
                                           const message::behaviour::state::Stability& stability,
                                           const std::shared_ptr<const RobotPoseGroundTruth>& robot_pose_ground_truth,
                                           const std::shared_ptr<const StellaMsg>& stella,
                                           const FaultDetectionStatus& fault_status);

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
        /// @param robot_pose_ground_truth The ground truth robot pose
        void update_odometry(std::unique_ptr<Sensors>& sensors,
                             const std::shared_ptr<const Sensors>& previous_sensors,
                             const RawSensors& raw_sensors,
                             const message::behaviour::state::Stability& stability,
                             const std::shared_ptr<const RobotPoseGroundTruth>& robot_pose_ground_truth,
                             const std::shared_ptr<const StellaMsg>& stella);

        /// @brief Display debug information
        /// @param sensors The sensors message to update
        /// @param robot_pose_ground_truth The ground truth robot pose
        void debug_sensor_filter(std::unique_ptr<Sensors>& sensors,
                                 const std::shared_ptr<const RobotPoseGroundTruth>& robot_pose_ground_truth);



    };
}  // namespace module::input
#endif  // MODULES_INPUT_SENSORFILTER_HPP
