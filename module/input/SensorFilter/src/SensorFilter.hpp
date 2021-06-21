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
     * @author Jake Fountain
     * @author Trent Houliston
     */
    class SensorFilter : public NUClear::Reactor {
    public:
        explicit SensorFilter(std::unique_ptr<NUClear::Environment> environment);

        utility::math::filter::UKF<double, MotionModel> motionFilter;

        struct Config {
            Config() : motionFilter(), buttons(), footDown() {}

            bool debug;

            struct MotionFilter {
                MotionFilter() : velocityDecay(Eigen::Vector3d::Zero()), noise(), initial() {}

                Eigen::Vector3d velocityDecay;

                struct Noise {
                    Noise() : measurement(), process() {}
                    struct Measurement {
                        Eigen::Matrix3d accelerometer;
                        Eigen::Matrix3d accelerometerMagnitude;
                        Eigen::Matrix3d gyroscope;
                        Eigen::Matrix3d flatFootOdometry;
                        Eigen::Matrix4d flatFootOrientation;
                    } measurement;
                    struct Process {
                        Eigen::Vector3d position;
                        Eigen::Vector3d velocity;
                        Eigen::Vector4d rotation;
                        Eigen::Vector3d rotationalVelocity;
                        Eigen::Vector3d gyroscopeBias;
                    } process;
                } noise;
                struct Initial {
                    Initial() : mean(), covariance() {}
                    struct Mean {
                        Eigen::Vector3d position;
                        Eigen::Vector3d velocity;
                        Eigen::Vector4d rotation;
                        Eigen::Vector3d rotationalVelocity;
                        Eigen::Vector3d gyroscopeBias;
                    } mean;
                    struct Covariance {
                        Eigen::Vector3d position;
                        Eigen::Vector3d velocity;
                        Eigen::Vector4d rotation;
                        Eigen::Vector3d rotationalVelocity;
                        Eigen::Vector3d gyroscopeBias;
                    } covariance;
                } initial;
            } motionFilter;

            struct Button {
                Button() : debounceThreshold(0) {}
                int debounceThreshold;
            } buttons;

            struct FootDown {
                FootDown() = default;
                FootDown(const std::string& method, const std::map<std::string, float>& thresholds) {
                    set_method(method, thresholds);
                }
                void set_method(const std::string& method, const std::map<std::string, float>& thresholds) {
                    if (thresholds.count(method) == 0) {
                        throw std::runtime_error(fmt::format("Invalid foot down method '{}'", method));
                    }
                    current_method       = method;
                    certainty_thresholds = thresholds;
                }
                [[nodiscard]] float threshold() const {
                    return certainty_thresholds.at(current_method);
                }
                [[nodiscard]] std::string method() const {
                    return current_method;
                }
                std::string current_method                        = "Z_HEIGHT";
                std::map<std::string, float> certainty_thresholds = {
                    {"Z_HEIGHT", 0.01f},
                    {"VIRTUAL", 0.05f},
                    {"FSR", 60.0f},
                };
            } footDown;
        } config;

    private:
        // Current state of the button pushes
        // used to debounce button presses
        bool leftDown   = false;
        bool middleDown = false;

        // Our sensor for foot down
        VirtualLoadSensor<float> load_sensor;

        // Foot to world in foot-flat rotation when the foot landed
        std::array<bool, 2> previous_foot_down = {false, false};
        std::array<Eigen::Affine3d, 2> footlanding_Hwf;

        // Foot to CoM in torso space
        std::array<Eigen::Vector3d, 2> rMFt{};
        Eigen::Vector3d rTWw{};

        // Storage for previous gyroscope values
        Eigen::Vector3d theta;

        // Handle for the sensor filter update loop, allows disabling new sensor updates when a reset event occurs
        ReactionHandle update_loop;
        std::atomic_bool reset_filter{true};
    };
}  // namespace module::input
#endif  // MODULES_INPUT_SENSORFILTER_HPP
