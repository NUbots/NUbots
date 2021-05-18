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

#ifndef MODULES_PLATFORM_DARWIN_SENSORFILTER_HPP
#define MODULES_PLATFORM_DARWIN_SENSORFILTER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "MotionModel.hpp"
#include "VirtualLoadSensor.hpp"

#include "message/motion/KinematicsModel.hpp"

#include "utility/math/filter/UKF.hpp"


namespace module::platform::darwin {

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
                FootDown() : fromLoad(true), certaintyThreshold(0.05) {}
                bool fromLoad;
                float certaintyThreshold;
            } footDown;
        } config;

    private:
        // Current state of the button pushes
        // used to debounce button presses
        bool leftDown   = false;
        bool middleDown = false;

        // Our sensor for foot down
        VirtualLoadSensor<float> load_sensor;

        // This keeps track of whether each sides foot was down in the previous time step
        // e.g. if right foot down at time t, then at time t+1, previous_foot_down[RightSide] = true
        std::array<bool, 2> previous_foot_down = {false, false};

        // Foot to world in foot-flat (both feet down) rotation at the timestep with the most recent foot landing
        std::array<Eigen::Affine3d, 2> footlanding_Hwf;

        // Storage for previous gyroscope values
        Eigen::Vector3d theta;

        /// @brief The number of Sensor updates since last reset was required
        /// Resets are required on startup, on configuration change, or on Cholesky decomposition failure
        uint32_t updates_since_reset_event;

        /// @brief The number of updates required to use the UKF
        /// We use the first MIN_UPDATES after a reset to set our initial position properly,
        /// storing each of those updates in `first_updates`
        static constexpr uint32_t MIN_UPDATES = 10;

        /// @brief The first updates after a reset. <b> Must be cleared on reset </b>
        std::vector<MotionModel<double>::StateVec> first_updates;
    };
}  // namespace module::platform::darwin
#endif  // MODULES_PLATFORM_DARWIN_SENSORFILTER_HPP
