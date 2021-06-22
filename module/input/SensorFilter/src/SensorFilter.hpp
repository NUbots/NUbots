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

        struct Config {
            Config() = default;

            bool debug = false;

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
                FootDown()               = default;
                bool fromLoad            = true;
                float certaintyThreshold = 0.05;
            } footDown{};
        } config{};

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

        // Storage for previous gyroscope values
        Eigen::Vector3d theta = Eigen::Vector3d::Zero();

        // Handle for the sensor filter update loop, allows disabling new sensor updates when a reset event occurs
        ReactionHandle update_loop{};
        std::atomic_bool reset_filter{true};
    };
}  // namespace module::input
#endif  // MODULES_INPUT_SENSORFILTER_HPP
