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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_INPUT_SENSORFILTER_HPP
#define MODULES_INPUT_SENSORFILTER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "MotionModel.hpp"
#include "VirtualLoadSensor.hpp"

#include "message/motion/KinematicsModel.hpp"

#include "utility/math/filter/UKF.hpp"


namespace module::input {

    /**
     * @author Jade Fountain
     * @author Trent Houliston
     */
    class SensorFilter : public NUClear::Reactor {
    public:
        explicit SensorFilter(std::unique_ptr<NUClear::Environment> environment);

        utility::math::filter::UKF<double, MotionModel> motionFilter{};

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

                Eigen::Vector3d velocityDecay = Eigen::Vector3d::Zero();

                struct Noise {
                    Noise() = default;
                    struct Measurement {
                        Eigen::Matrix3d accelerometer          = Eigen::Matrix3d::Zero();
                        Eigen::Matrix3d accelerometerMagnitude = Eigen::Matrix3d::Zero();
                        Eigen::Matrix3d gyroscope              = Eigen::Matrix3d::Zero();
                        Eigen::Matrix3d flatFootOdometry       = Eigen::Matrix3d::Zero();
                        Eigen::Matrix4d flatFootOrientation    = Eigen::Matrix4d::Zero();
                    } measurement{};
                    struct Process {
                        Eigen::Vector3d position           = Eigen::Vector3d::Zero();
                        Eigen::Vector3d velocity           = Eigen::Vector3d::Zero();
                        Eigen::Vector4d rotation           = Eigen::Vector4d::Zero();
                        Eigen::Vector3d rotationalVelocity = Eigen::Vector3d::Zero();
                        Eigen::Vector3d gyroscopeBias      = Eigen::Vector3d::Zero();
                    } process{};
                } noise{};
                struct Initial {
                    Initial() = default;
                    struct Mean {
                        Eigen::Vector3d position           = Eigen::Vector3d::Zero();
                        Eigen::Vector3d velocity           = Eigen::Vector3d::Zero();
                        Eigen::Vector4d rotation           = Eigen::Vector4d::Zero();
                        Eigen::Vector3d rotationalVelocity = Eigen::Vector3d::Zero();
                        Eigen::Vector3d gyroscopeBias      = Eigen::Vector3d::Zero();
                    } mean{};
                    struct Covariance {
                        Eigen::Vector3d position           = Eigen::Vector3d::Zero();
                        Eigen::Vector3d velocity           = Eigen::Vector3d::Zero();
                        Eigen::Vector4d rotation           = Eigen::Vector4d::Zero();
                        Eigen::Vector3d rotationalVelocity = Eigen::Vector3d::Zero();
                        Eigen::Vector3d gyroscopeBias      = Eigen::Vector3d::Zero();
                    } covariance{};
                } initial{};
            } motionFilter{};

            struct Button {
                Button()              = default;
                int debounceThreshold = 0;
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
        } config;

    private:
        // Current state of the button pushes
        // used to debounce button presses
        bool leftDown   = false;
        bool middleDown = false;

        // Our sensor for foot down
        VirtualLoadSensor<float> load_sensor{};

        // This keeps track of whether each sides foot was down in the previous time step
        // e.g. if right foot down at time t, then at time t+1, previous_foot_down[RightSide] = true
        std::array<bool, 2> previous_foot_down = {false, false};
        // Foot to world in foot-flat (both feet down) rotation at the timestep with the most recent foot landing
        std::array<Eigen::Affine3d, 2> footlanding_Hwf{};

        // Foot to CoM in torso space
        std::array<Eigen::Vector3d, 2> rMFt{};
        Eigen::Vector3d rTWw{};

        // Storage for previous gyroscope values
        Eigen::Vector3d theta = Eigen::Vector3d::Zero();

        // Handle for the sensor filter update loop, allows disabling new sensor updates when a reset event occurs
        ReactionHandle update_loop{};
        std::atomic_bool reset_filter{true};
    };
}  // namespace module::input
#endif  // MODULES_INPUT_SENSORFILTER_HPP
