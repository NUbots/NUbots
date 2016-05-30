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
#include "utility/math/filter/UKF.h"
#include "MotionModel.h"
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
                        struct {
                            double footDownWeight;
                        } fsr;
                    } foot;

                    struct {
                        int debounceThreshold;
                    } buttons;
                } config;

            private:
                // Current state of the button pushes
                bool leftDown = false;
                bool middleDown = false;

                // Our torso position from the left foot when it landed
                arma::vec3 leftFootLanding;
                // Our torso position from the global origin when the left foot landed
                arma::vec2 leftFootLandingWorld;
                //The rotation from foot to world coordinates at landing
                arma::mat33 leftFootLandingWorldRot;

                // Our torso position from the right foot when it landed
                arma::vec2 rightFootLanding;
                // Our torso position from the global origin when the right foot landed
                arma::vec2 rightFootLandingWorld;
                //The rotation from foot to world coordinates at landing
                arma::mat33 rightFootLandingWorldRot;
            };
        }
    }
}
#endif  // MODULES_PLATFORM_DARWIN_SENSORFILTER_H

