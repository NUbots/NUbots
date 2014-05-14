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

#include <math.h> //needed for normalisation function
#include <assert.h>
#include "InverseDepthPointModel.h" //includes armadillo
#include "utility/motion/ForwardKinematics.h"
#include "messages/input/ServoID.h"

#include <iostream>

namespace utility {
    namespace math {
        namespace kalman {

            using messages::localisation::Self;
            using messages::input::Sensors;
            using messages::input::ServoID;

            arma::vec::fixed<InverseDepthPointModel::size> InverseDepthPointModel::limitState(const arma::vec::fixed<size>& state) {
                
                return state;
            }

            // @brief The process equation is used to update the systems state using the process euquations of the system.
            // @param sigma_point The sigma point representing a system state.
            // @param deltaT The amount of time that has passed since the previous update, in seconds.
            // @param measurement The reading from the rate gyroscope in rad/s used to update the orientation.
            // @return The new estimated system state.
            arma::vec::fixed<InverseDepthPointModel::size> InverseDepthPointModel::timeUpdate(const arma::vec::fixed<size>& state, double deltaT) {
                return state;
            }

            arma::vec InverseDepthPointModel::predictedObservation(const arma::vec::fixed<size>& state, const std::pair<const Sensors&, const Self&> stateData) {
                arma::vec2 selfHeading = arma::normalise(stateData.second.heading);
                arma::mat44 robotToWorld_world = arma::mat44({             selfHeading[0],              -selfHeading[1], 0,     stateData.second.position[0],
                                                                           selfHeading[1],               selfHeading[0], 0,     stateData.second.position[1],
                                                                                        0,                            0, 1, stateData.first.bodyCentreHeight,
                                                                                        0,                            0, 0,                                1});

                arma::mat44 cameraToBody_body = stateData.first.forwardKinematics.at(ServoID::HEAD_PITCH);

                arma::mat44 robotToBody_body = arma::eye(4,4);
                //TODO: copy localisation in develop
                robotToBody_body.submat(0,0,2,2) = stateData.first.orientation;

                arma::mat44 worldToCamera_camera = utility::math::matrix::orthonormal44Inverse(cameraToBody_body) * robotToBody_body * utility::math::matrix::orthonormal44Inverse(robotToWorld_world);

                arma::vec4 initialObservedDirection = utility::math::matrix::yRotationMatrix(-state[kPHI],4) * utility::math::matrix::zRotationMatrix(state[kTHETA],4) * arma::vec4{1,0,0,0};
                
                arma::vec4 cameraToFeatureVector_cam =  worldToCamera_camera * ((arma::vec4({state[kX],state[kY],state[kZ],1}) - worldToCamera_camera.submat(0,3,3,3)) * state[kRHO] + initialObservedDirection);

                arma::vec screenAngular = { std::atan2(cameraToFeatureVector_cam[1], cameraToFeatureVector_cam[0]) , std::atan2(cameraToFeatureVector_cam[2], cameraToFeatureVector_cam[0])};

                return screenAngular;     //Camera y,z = hor, vert
            }


            arma::vec InverseDepthPointModel::observationDifference(const arma::vec& a, const arma::vec& b) {
                return a - b;
            }

            arma::mat::fixed<InverseDepthPointModel::size, InverseDepthPointModel::size> InverseDepthPointModel::processNoise() {
                return arma::eye(size, size) *1e-6; //std::numeric_limits<double>::epsilon();
            }

        }
    }
}