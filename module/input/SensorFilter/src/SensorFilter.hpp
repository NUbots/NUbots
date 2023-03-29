/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_INPUT_SENSORFILTER_HPP
#define MODULES_INPUT_SENSORFILTER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "MotionModel.hpp"
#include "VirtualLoadSensor.hpp"

#include "message/actuation/BodySide.hpp"
#include "message/actuation/KinematicsModel.hpp"
#include "message/input/Sensors.hpp"
#include "message/motion/GetupCommand.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/actuation/ForwardKinematics.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/math/filter/KalmanFilter.hpp"
#include "utility/math/filter/MahonyFilter.hpp"
#include "utility/math/filter/UKF.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using message::actuation::BodySide;
    using message::actuation::KinematicsModel;
    using message::input::Sensors;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::ExecuteGetup;
    using message::motion::KillGetup;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;
    using message::platform::ButtonLeftDown;
    using message::platform::ButtonLeftUp;
    using message::platform::ButtonMiddleDown;
    using message::platform::ButtonMiddleUp;
    using message::platform::RawSensors;

    using utility::actuation::kinematics::calculateAllPositions;
    using utility::actuation::kinematics::calculateCentreOfMass;
    using utility::actuation::kinematics::calculateInertialTensor;
    using utility::input::ServoID;
    using utility::math::euler::EulerIntrinsicToMatrix;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::nusight::graph;
    using utility::platform::getRawServo;
    using utility::platform::make_error_string;
    using utility::platform::make_servo_error_string;
    using utility::support::Expression;

    /**
     * @author Jade Fountain
     * @author Trent Houliston
     */
    class SensorFilter : public NUClear::Reactor {
    public:
        explicit SensorFilter(std::unique_ptr<NUClear::Environment> environment);

        utility::math::filter::UKF<double, MotionModel> motionFilter{};

        // Define the kalman filter model dimensions
        static const size_t n_states       = 4;
        static const size_t n_inputs       = 0;
        static const size_t n_measurements = 4;

        /// @brief Kalman filter for pose estimation
        utility::math::filter::KalmanFilter<double, n_states, n_inputs, n_measurements> pose_filter;

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
                            throw std::runtime_error("String " + str + " did not match any enum for ServoID");
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
        struct Config {
            Config() = default;

            struct MotionFilter {
                MotionFilter() = default;

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
            } motionFilter{};

            /// @brief Initial state of the for the UKF filter
            MotionModel<double>::StateVec initial_mean;

            /// @brief Initial covariance of the for the UKF filter
            MotionModel<double>::StateVec initial_covariance;

            struct Button {
                Button()               = default;
                int debounce_threshold = 0;
            } buttons{};

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

            double deadreckoning_scale_dx     = 1.0;
            double deadreckoning_scale_dy     = 1.0;
            double deadreckoning_scale_dtheta = 1.0;

            Eigen::Vector3d bias;
            Eigen::Vector4d initial_quat;
            double Ki;
            double Kp;
            double ts;
        } cfg;

        /// @brief Updates the sensors message with raw sensor data, including the timestamp, battery
        /// voltage, servo sensors, accelerometer, gyroscope, buttons, and LED.
        /// @param sensors The sensors message to update
        /// @param previous_sensors The previous sensors message
        /// @param raw_sensors The raw sensor data
        void update_raw_sensors(std::unique_ptr<Sensors>& sensors,
                                const std::shared_ptr<const Sensors>& previous_sensors,
                                const RawSensors& raw_sensors);

        /// @brief Update the sensors message with kinematics data
        /// @param sensors The sensors message to update
        /// @param kinematics_model The kinematics model to use for calculations
        void update_kinematics(std::unique_ptr<Sensors>& sensors,
                               const KinematicsModel& kinematics_model,
                               const RawSensors& raw_sensors);

        /// @brief Updates the sensors message with odometry data filtered using Kalman Filter. This includes the
        // position, orientation, velocity and rotational velocity of the torso in world space.
        /// @param sensors The sensors message to update
        /// @param previous_sensors The previous sensors message
        /// @param raw_sensors The raw sensor data
        void update_odometry_kf(std::unique_ptr<Sensors>& sensors,
                                const std::shared_ptr<const Sensors>& previous_sensors,
                                const RawSensors& raw_sensors);

        /// @brief Updates the sensors message with odometry data filtered using UKF. This includes the
        // position, orientation, velocity and rotational velocity of the torso in world space.
        /// @param sensors The sensors message to update
        /// @param previous_sensors The previous sensors message
        /// @param raw_sensors The raw sensor data
        void update_odometry_ukf(std::unique_ptr<Sensors>& sensors,
                                 const std::shared_ptr<const Sensors>& previous_sensors,
                                 const RawSensors& raw_sensors);

        /// @brief Updates the sensors message with odometry data filtered using MahonyFilter. This includes the
        // position, orientation, velocity and rotational velocity of the torso in world space.
        /// @param sensors The sensors message to update
        /// @param previous_sensors The previous sensors message
        /// @param raw_sensors The raw sensor data
        void update_odometry_mahony(std::unique_ptr<Sensors>& sensors,
                                    const std::shared_ptr<const Sensors>& previous_sensors,
                                    const RawSensors& raw_sensors);

        /// @brief Display debug information
        /// @param sensors The sensors message to update
        /// @param raw_sensors The raw sensor data
        void debug_sensor_filter(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors);

    private:
        /// @brief Dead reckoning position of the robot {r} [x,y,z] in world {w} space
        Eigen::Vector3d rRWw = Eigen::Vector3d::Zero();

        /// @brief Dead reckoning yaw orientation of the robot in world space
        double yaw = 0;

        /// @brief Kinematics estimate of the robot's height above the ground
        double z_height = 0.5;

        /// @brief Mahony filter quaternion for orientation
        Eigen::Quaterniond quat_Rwt = Eigen::Quaterniond::Identity();

        /// @brief Mahony filter bias
        Eigen::Vector3d bias = Eigen::Vector3d::Zero();

        /// @brief Current walk command
        Eigen::Vector3d walk_command = Eigen::Vector3d::Zero();

        /// @brief Bool to indicate if the robot is falling
        bool falling = false;

        /// @brief Bool to indicate if the robot is walking
        bool walk_engine_enabled = false;

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

        // Foot to CoM in torso space
        std::array<Eigen::Vector3d, 2> rMFt{};

        // Handle for the sensor filter update loop, allows disabling new sensor updates when a reset event occurs
        ReactionHandle update_loop{};
        std::atomic_bool reset_filter{true};
    };
}  // namespace module::input
#endif  // MODULES_INPUT_SENSORFILTER_HPP
