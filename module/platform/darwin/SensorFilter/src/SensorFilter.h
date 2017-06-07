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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_PLATFORM_DARWIN_SENSORFILTER_H
#define MODULES_PLATFORM_DARWIN_SENSORFILTER_H

#include <nuclear>

#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/filter/UKF.h"

#include "MotionModel.h"
#include "DarwinVirtualLoadSensor.h"
#include "utility/math/matrix/Rotation3D.h"
#include "message/motion/KinematicsModels.h"


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
                    Config() : battery(), motionFilter(), buttons() {}

                    struct Battery {
                        Battery() : chargedVoltage(0.0f), flatVoltage(0.0f) {}
                        float chargedVoltage;
                        float flatVoltage;
                    } battery;

                    struct MotionFilter {
                        MotionFilter() : velocityDecay(arma::fill::zeros), noise(), initial() {}

                        Eigen::Vector3d velocityDecay;

                        struct Noise {
                            Noise() : measurement(), process() {}
                            struct Measurement {
                                Measurement() : accelerometer(arma::fill::eye), accelerometerMagnitude(arma::fill::eye), gyroscope(arma::fill::eye),
                                                footUpWithZ(arma::fill::eye), flatFootOdometry(arma::fill::eye),
                                                flatFootOrientation(arma::fill::eye) {}
                                arma::mat33 accelerometer;
                                arma::mat33 accelerometerMagnitude;
                                arma::mat33 gyroscope;
                                arma::mat44 footUpWithZ;
                                arma::mat33 flatFootOdometry;
                                arma::mat44 flatFootOrientation;
                            } measurement;
                            struct Process {
                                Process() : position(arma::fill::ones), velocity(arma::fill::ones),
                                            rotation(arma::fill::ones), rotationalVelocity(arma::fill::ones) {}
                                Eigen::Vector3d position;
                                Eigen::Vector3d velocity;
                                Eigen::Vector4d rotation;
                                Eigen::Vector3d rotationalVelocity;
                            } process;
                        } noise;
                        struct Initial {
                            Initial() : mean(), covariance() {}
                            struct Mean {
                                Mean() : position(arma::fill::ones), velocity(arma::fill::ones),
                                         rotation(arma::fill::ones), rotationalVelocity(arma::fill::ones) {}
                                Eigen::Vector3d position;
                                Eigen::Vector3d velocity;
                                Eigen::Vector4d rotation;
                                Eigen::Vector3d rotationalVelocity;
                            } mean;
                            struct Covariance {
                                Covariance() : position(arma::fill::ones), velocity(arma::fill::ones),
                                               rotation(arma::fill::ones), rotationalVelocity(arma::fill::ones) {}
                                Eigen::Vector3d position;
                                Eigen::Vector3d velocity;
                                Eigen::Vector4d rotation;
                                Eigen::Vector3d rotationalVelocity;
                            } covariance;
                        } initial;
                    } motionFilter;

                    struct Button {
                        Button() : debounceThreshold(0) {}
                        int debounceThreshold;
                    } buttons;
                } config;

            private:
                // Current state of the button pushes
                // used to debounce button presses
                bool leftDown = false;
                bool middleDown = false;

                // Our sensor for foot down
                DarwinVirtualLoadSensor leftFootDown;
                DarwinVirtualLoadSensor rightFootDown;

                //World to foot in world rotation when the foot landed
                std::array<Eigen::Vector3d, 2> footlanding_rFWw;

                //Foot to world in foot-flat rotation when the foot landed
                std::array<utility::math::matrix::Rotation3D, 2> footlanding_Rfw;

                //World to foot in foot-flat rotation when the foot landed
                std::array<utility::math::matrix::Rotation3D, 2> footlanding_Rwf;

            };
        }
    }
}
#endif  // MODULES_PLATFORM_DARWIN_SENSORFILTER_H

