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
#include "VirtualLoadSensor.hpp"

#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/math/filter/KalmanFilter.hpp"
#include "utility/math/filter/UKF.hpp"

using extension::Configuration;

namespace module::input {

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using message::platform::RawSensors;

    /**
     * @author Jade Fountain
     * @author Trent Houliston
     */
    class SensorFilter : public NUClear::Reactor {
    public:
        explicit SensorFilter(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Number of actuatable joints in the NUgus robot
        static const int n_joints = 20;

        /// @brief tinyrobotics NUgus model used for kinematics
        tinyrobotics::Model<double, n_joints> nugus_model;

        /// @brief Unscented kalman filter for pose estimation
        utility::math::filter::UKF<double, MotionModel> ukf{};

        /// @brief Number of states in the Kalman filter. x = [roll, pitch, roll rate, pitch rate]'
        static const size_t n_states = 4;

        /// @brief Number of inputs in the Kalman filter. Model has no inputs
        static const size_t n_inputs = 0;

        /// @brief Number of measurements in the Kalman filter. y = [roll, pitch, roll rate, pitch rate]'
        static const size_t n_measurements = 4;

        /// @brief Kalman filter for pose estimation
        utility::math::filter::KalmanFilter<double, n_states, n_inputs, n_measurements> kf{};

        struct FootDownMethod {
            enum Value { UNKNOWN = 0, Z_HEIGHT = 1, LOAD = 2, FSR = 3 };
            Value value = Value::UNKNOWN;

            // Constructors
            FootDownMethod() = default;
            FootDownMethod(int const& v) : value(static_cast<Value>(v)) {}
            FootDownMethod(Value const& v) : value(v) {}
            FootDownMethod(std::string const& str) {
                // clang-format off
                        if      (str == "Z_HEIGHT") { value = Value::Z_HEIGHT; }
                        else if (str == "LOAD") { value = Value::LOAD; }
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
                    case Value::LOAD: return "VIRTUAL";
                    case Value::FSR: return "FSR";
                    default: throw std::runtime_error("enum Method's value is corrupt, unknown value stored");
                }
            }
        };

        struct FilteringMethod {
            enum Value { UNKNOWN = 0, UKF = 1, KF = 2, MAHONY = 3, GROUND_TRUTH = 4 };
            Value value = Value::UNKNOWN;

            // Constructors
            FilteringMethod() = default;
            FilteringMethod(int const& v) : value(static_cast<Value>(v)) {}
            FilteringMethod(Value const& v) : value(v) {}
            FilteringMethod(std::string const& str) {
                // clang-format off
                        if      (str == "UKF") { value = Value::UKF; }
                        else if (str == "KF") { value = Value::KF; }
                        else if (str == "MAHONY")  { value = Value::MAHONY; }
                        else if (str == "GROUND_TRUTH")  { value = Value::GROUND_TRUTH; }
                        else {
                            value = Value::UNKNOWN;
                            throw std::runtime_error("String " + str + " did not match any enum for FilteringMethod");
                        }
                // clang-format on
            }

            // Conversions
            [[nodiscard]] operator Value() const {
                return value;
            }
            [[nodiscard]] operator std::string() const {
                switch (value) {
                    case Value::UKF: return "UKF";
                    case Value::KF: return "KF";
                    case Value::MAHONY: return "MAHONY";
                    case Value::GROUND_TRUTH: return "GROUND_TRUTH";
                    default: throw std::runtime_error("enum Method's value is corrupt, unknown value stored");
                }
            }
        };


        struct Config {
            /// @brief Path to NUgus URDF file
            std::string urdf_path = "";

            /// @brief Config for the button debouncer
            struct Button {
                Button() = default;
                /// @brief The number of times a button must be pressed before it is considered pressed
                int debounce_threshold = 0;
            } buttons{};

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
                    {FootDownMethod::LOAD, 0.05f},
                    {FootDownMethod::FSR, 60.0f},
                };
            } footDown;

            /// @brief Specifies the filtering method to use (UKF, KF, MAHONY)
            FilteringMethod filtering_method;

            //  **************************************** UKF Config ****************************************
            /// @brief Config for the UKF
            struct UKF {
                Eigen::Vector3d velocity_decay = Eigen::Vector3d::Zero();

                struct Noise {
                    Noise() = default;
                    struct Measurement {
                        Eigen::Matrix3d accelerometer           = Eigen::Matrix3d::Zero();
                        Eigen::Matrix3d accelerometer_magnitude = Eigen::Matrix3d::Zero();
                        Eigen::Matrix3d gyroscope               = Eigen::Matrix3d::Zero();
                        Eigen::Matrix3d flat_foot_odometry      = Eigen::Matrix3d::Zero();
                        Eigen::Matrix4d flat_foot_orientation   = Eigen::Matrix4d::Zero();
                    } measurement{};
                    struct Process {
                        Eigen::Vector3d position            = Eigen::Vector3d::Zero();
                        Eigen::Vector3d velocity            = Eigen::Vector3d::Zero();
                        Eigen::Vector4d rotation            = Eigen::Vector4d::Zero();
                        Eigen::Vector3d rotational_velocity = Eigen::Vector3d::Zero();
                        Eigen::Vector3d gyroscope_bias      = Eigen::Vector3d::Zero();
                    } process{};
                } noise{};
                struct Initial {
                    Initial() = default;
                    struct Mean {
                        Eigen::Vector3d position            = Eigen::Vector3d::Zero();
                        Eigen::Vector3d velocity            = Eigen::Vector3d::Zero();
                        Eigen::Vector4d rotation            = Eigen::Vector4d::Zero();
                        Eigen::Vector3d rotational_velocity = Eigen::Vector3d::Zero();
                        Eigen::Vector3d gyroscope_bias      = Eigen::Vector3d::Zero();
                    } mean{};
                    struct Covariance {
                        Eigen::Vector3d position            = Eigen::Vector3d::Zero();
                        Eigen::Vector3d velocity            = Eigen::Vector3d::Zero();
                        Eigen::Vector4d rotation            = Eigen::Vector4d::Zero();
                        Eigen::Vector3d rotational_velocity = Eigen::Vector3d::Zero();
                        Eigen::Vector3d gyroscope_bias      = Eigen::Vector3d::Zero();
                    } covariance{};
                } initial{};
            } ukf{};

            /// @brief Initial state of the for the UKF filter
            MotionModel<double>::StateVec initial_mean;

            /// @brief Initial covariance of the for the UKF filter
            MotionModel<double>::StateVec initial_covariance;

            /// @brief Parameter for scaling the walk command to better match actual achieved velocity
            Eigen::Vector3d deadreckoning_scale = Eigen::Vector3d::Zero();

            //  **************************************** Kalman Filter Config ****************************************
            /// @brief Kalman Continuous time process model
            Eigen::Matrix<double, n_states, n_states> Ac;

            /// @brief Kalman Continuous time input model
            Eigen::Matrix<double, n_states, n_inputs> Bc;

            /// @brief Kalman Continuous time measurement model
            Eigen::Matrix<double, n_measurements, n_states> C;

            /// @brief Kalman Process noise
            Eigen::Matrix<double, n_states, n_states> Q;

            /// @brief Kalman Measurement noise
            Eigen::Matrix<double, n_measurements, n_measurements> R;

            //  **************************************** Mahony Filter Config ****************************************
            /// @brief Mahony filter bias
            Eigen::Vector3d bias = Eigen::Vector3d::Zero();

            /// @brief Mahony filter proportional gain
            double Ki = 0.0;

            /// @brief Mahony filter integral gain
            double Kp = 0.0;
        } cfg;

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

        /// @brief Runs a deadreckoning update on the odometry for x, y and yaw using the walk command
        /// @param dt The time since the last update
        /// @param walk_state Current state of walk engine
        void integrate_walkcommand(const double dt, const Stability& stability, const WalkState& walk_state);

        /// @brief Updates translational and yaw components of odometry using the anchor method
        /// @param walk_state Current state of walk engine
        void anchor_update(std::unique_ptr<Sensors>& sensors, const WalkState& walk_state);

        /// @brief Configure UKF filter
        void configure_ukf(const Configuration& config);

        /// @brief Configure Kalman filter
        void configure_kf(const Configuration& config);

        /// @brief Configure Mahony filter
        void configure_mahony(const Configuration& config);

        /// @brief Updates the sensors message with odometry data filtered using UKF. This includes the
        // position, orientation, velocity and rotational velocity of the torso in world space.
        /// @param sensors The sensors message to update
        /// @param previous_sensors The previous sensors message
        /// @param raw_sensors The raw sensor data
        void update_odometry_ukf(std::unique_ptr<Sensors>& sensors,
                                 const std::shared_ptr<const Sensors>& previous_sensors,
                                 const RawSensors& raw_sensors);

        /// @brief Updates the sensors message with odometry data filtered using Kalman Filter. This includes the
        // position, orientation, velocity and rotational velocity of the torso in world space.
        /// @param sensors The sensors message to update
        /// @param previous_sensors The previous sensors message
        /// @param raw_sensors The raw sensor data
        void update_odometry_kf(std::unique_ptr<Sensors>& sensors,
                                const std::shared_ptr<const Sensors>& previous_sensors,
                                const RawSensors& raw_sensors,
                                const std::shared_ptr<const Stability>& stability,
                                const std::shared_ptr<const WalkState>& walk_state);


        /// @brief Updates the sensors message with odometry data filtered using MahonyFilter. This includes the
        // position, orientation, velocity and rotational velocity of the torso in world space.
        /// @param sensors The sensors message to update
        /// @param previous_sensors The previous sensors message
        /// @param raw_sensors The raw sensor data
        void update_odometry_mahony(std::unique_ptr<Sensors>& sensors,
                                    const std::shared_ptr<const Sensors>& previous_sensors,
                                    const RawSensors& raw_sensors,
                                    const std::shared_ptr<const WalkState>& walk_state);

        /// @brief Updates the sensors message with odometry data filtered using ground truth from WeBots. This includes
        /// the position, orientation, velocity and rotational velocity of the torso in world space.
        /// @param sensors The sensors message to update
        /// @param previous_sensors The previous sensors message
        /// @param raw_sensors The raw sensor data
        void update_odometry_ground_truth(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors);

        /// @brief Display debug information
        /// @param sensors The sensors message to update
        /// @param raw_sensors The raw sensor data
        void debug_sensor_filter(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors);

    private:
        /// @brief Transform from planted foot {p} to world {w} space
        Eigen::Isometry3d Hwp = Eigen::Isometry3d::Identity();

        /// @brief Current support phase of the robot
        WalkState::SupportPhase current_support_phase = WalkState::SupportPhase::LEFT;

        /// @brief Dead reckoning yaw orientation of the robot in world space
        double yaw = 0;

        /// @brief Transform from torso {t} to world {w} space
        Eigen::Isometry3d Hwt = Eigen::Isometry3d::Identity();

        // @brief Transform from torso {t} to world {w} space using mahony filter (only roll and pitch estimation)
        Eigen::Isometry3d Hwt_mahony = Eigen::Isometry3d::Identity();

        /// @brief Current walk command
        Eigen::Vector3d walk_command = Eigen::Vector3d::Zero();

        /// @brief Current state of the left button
        bool left_down = false;

        /// @brief Current state of the middle button
        bool middle_down = false;

        /// @brief Our sensor for foot down
        VirtualLoadSensor<float> load_sensor{};

        // This keeps track of whether each sides foot was down in the previous time step
        // e.g. if right foot down at time t, then at time t+1, previous_foot_down[RightSide] = true
        std::array<bool, 2> previous_foot_down = {false, false};

        // Foot to world in foot-flat (both feet down) rotation at the timestep with the most recent foot landing
        std::array<Eigen::Isometry3d, 2> footlanding_Hwf{};

        // Handle for the sensor filter update loop, allows disabling new sensor updates when a reset event occurs
        ReactionHandle update_loop{};
        std::atomic_bool reset_filter{false};
    };
}  // namespace module::input
#endif  // MODULES_INPUT_SENSORFILTER_HPP
