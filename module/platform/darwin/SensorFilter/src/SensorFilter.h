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

#include "message/input/Sensors.h"

#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/filter/UKF.h"
#include "MotionModel.h"
#include "DarwinVirtualLoadSensor.h"
#include "utility/motion/RobotModels.h"

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

                struct {
                    struct {
                        float chargedVoltage;
                        float flatVoltage;
                    } battery;

                    struct {
                        arma::vec3 velocityDecay;
                        struct  {
                            struct {
                                arma::mat33 accelerometer;
                                arma::mat33 gyroscope;
                                arma::mat44 footUpWithZ;
                                arma::mat22 flatFootOdometry;
                            } measurement;
                            struct {
                                arma::vec3 position;
                                arma::vec3 velocity;
                                arma::vec4 rotation;
                                arma::vec3 rotationalVelocity;
                            } process;
                        } noise;
                        struct {
                            struct {
                                arma::vec3 position;
                                arma::vec3 velocity;
                                arma::vec4 rotation;
                                arma::vec3 rotationalVelocity;
                            } mean;
                            struct {
                                arma::vec3 position;
                                arma::vec3 velocity;
                                arma::vec4 rotation;
                                arma::vec3 rotationalVelocity;
                            } covariance;
                        } initial;
                    } motionFilter;

                    struct {
                        int debounceThreshold;
                    } buttons;
                } config;

            private:
                // Current state of the button pushes
                bool leftDown = false;
                bool middleDown = false;

                // Our sensor for foot down
                DarwinVirtualLoadSensor leftFootDown,rightFootDown;
                // Our torso rotation from foot when it landed
                std::array<utility::math::matrix::Rotation3D, 2> footlanding_Rtf;
                // Our torso position from foot in foot coordinates when it landed
                std::array<arma::vec3, 2> footlanding_rTFf;

                // The worlds rotation from our torso
                std::array<utility::math::matrix::Rotation3D, 2> footlanding_Rwt;
                // The torsos position from the origin in world
                std::array<arma::vec3, 2> footlanding_rTWw;
            };
        }
    }
}
#endif  // MODULES_PLATFORM_DARWIN_SENSORFILTER_H

