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

#ifndef MODULES_PLATFORM_DARWIN_SENSORFILTER_H
#define MODULES_PLATFORM_DARWIN_SENSORFILTER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "utility/math/filter/UKF.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"

#include "MotionModel.h"
#include "VirtualLoadSensor.h"
#include "message/motion/KinematicsModel.h"
#include "utility/math/matrix/Rotation3D.h"


namespace module {
namespace platform {
    namespace darwin {

        /**
         * TODO document
         *
         * @author Jake Fountain
         * @author Trent Houliston
         */
        class SensorFilter : public NUClear::Reactor {
        public:
            explicit SensorFilter(std::unique_ptr<NUClear::Environment> environment);

            utility::math::filter::UKF<MotionModel> motionFilter;

            struct Config {
                Config() : motionFilter(), buttons(), footDown() {}

                bool debug;

                struct MotionFilter {
                    MotionFilter() : velocityDecay(arma::fill::zeros), noise(), initial() {}

                    arma::vec3 velocityDecay;

                    struct Noise {
                        Noise() : measurement(), process() {}
                        struct Measurement {
                            Measurement()
                                : accelerometer(arma::fill::eye)
                                , accelerometerMagnitude(arma::fill::eye)
                                , gyroscope(arma::fill::eye)
                                , flatFootOdometry(arma::fill::eye)
                                , flatFootOrientation(arma::fill::eye) {}
                            arma::mat33 accelerometer;
                            arma::mat33 accelerometerMagnitude;
                            arma::mat33 gyroscope;
                            arma::mat33 flatFootOdometry;
                            arma::mat44 flatFootOrientation;
                        } measurement;
                        struct Process {
                            Process()
                                : position(arma::fill::ones)
                                , velocity(arma::fill::ones)
                                , rotation(arma::fill::ones)
                                , rotationalVelocity(arma::fill::ones) {}
                            arma::vec3 position;
                            arma::vec3 velocity;
                            arma::vec4 rotation;
                            arma::vec3 rotationalVelocity;
                            arma::vec3 gyroscopeBias;
                        } process;
                    } noise;
                    struct Initial {
                        Initial() : mean(), covariance() {}
                        struct Mean {
                            Mean()
                                : position(arma::fill::ones)
                                , velocity(arma::fill::ones)
                                , rotation(arma::fill::ones)
                                , rotationalVelocity(arma::fill::ones) {}
                            arma::vec3 position;
                            arma::vec3 velocity;
                            arma::vec4 rotation;
                            arma::vec3 rotationalVelocity;
                            arma::vec3 gyroscopeBias;
                        } mean;
                        struct Covariance {
                            Covariance()
                                : position(arma::fill::ones)
                                , velocity(arma::fill::ones)
                                , rotation(arma::fill::ones)
                                , rotationalVelocity(arma::fill::ones) {}
                            arma::vec3 position;
                            arma::vec3 velocity;
                            arma::vec4 rotation;
                            arma::vec3 rotationalVelocity;
                            arma::vec3 gyroscopeBias;
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

            // Foot to world in foot-flat rotation when the foot landed
            std::array<bool, 2> previous_foot_down = {false, false};
            std::array<Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign>, 2> footlanding_Hwf;

            // Storage for previous gyroscope values
            arma::vec3 theta;
        };
    }  // namespace darwin
}  // namespace platform
}  // namespace module
#endif  // MODULES_PLATFORM_DARWIN_SENSORFILTER_H
