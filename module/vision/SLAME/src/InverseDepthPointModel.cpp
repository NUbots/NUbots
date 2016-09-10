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

#include <cmath> //needed for normalisation function
#include <cassert>
#include "InverseDepthPointModel.h" //includes armadillo
#include "utility/motion/ForwardKinematics.h"
#include "message/input/ServoID.h"
#include "utility/math/vision.h"
#include "utility/math/matrix/Transform3D.h"

namespace utility {
    namespace math {
        namespace kalman {

            using message::localisation::Self;
            using message::input::Sensors;
            using message::input::ServoID;

            arma::vec::fixed<InverseDepthPointModel::size> InverseDepthPointModel::limitState(const arma::vec::fixed<size>& state) {

                return state;
            }

            // @brief The process equation is used to update the systems state using the process euquations of the system.
            // @param sigma_point The sigma point representing a system state.
            // @param deltaT The amount of time that has passed since the previous update, in seconds.
            // @param measurement The reading from the rate gyroscope in rad/s used to update the orientation.
            // @return The new estimated system state.
            arma::vec::fixed<InverseDepthPointModel::size> InverseDepthPointModel::timeUpdate(const arma::vec::fixed<size>& state, const double&) {
                return state;
            }

            arma::vec InverseDepthPointModel::predictedObservation(const arma::vec::fixed<size>& state, const arma::mat& worldToCamera_camera) {
                // NUClear::log("State = ",state);
                // NUClear::log("worldToCamera_camera", worldToCamera_camera);
                arma::vec initialObservedDirection = utility::math::vision::directionVectorFromScreenAngular({state[kTHETA], state[kPHI]});
                arma::mat cameraToWorld_world = matrix::Transform3D(worldToCamera_camera).i();
                // NUClear::log("World Coordinate of feature direction\n", (arma::vec4({state[kX],state[kY],state[kZ],0}) - cameraToWorld_world.submat(0,3,3,3) + arma::vec{0,0,0,1}) * state[kRHO] + initialObservedDirection);
                arma::vec cameraToFeatureVector_cam =  worldToCamera_camera * ((arma::vec4({state[kX],state[kY],state[kZ],0}) - cameraToWorld_world.submat(0,3,3,3) + arma::vec{0,0,0,1}) * state[kRHO] + initialObservedDirection);
                //OLD ANGLE METHOD
                arma::vec screenAngular = utility::math::vision::screenAngularFromDirectionVector(cameraToFeatureVector_cam.rows(0,3));
                // NUClear::log("Predicted screen angular of feature =", screenAngular);
                return screenAngular;     //Camera y,z = hor, vert

                //NEW
                // arma::vec screenBearing = utility::math::vision::screenPositionFromDirectionVector(cameraToFeatureVector_cam.rows(0,3));
                // return screenBearing;     //Camera y,z = hor, vert
            }


            arma::vec InverseDepthPointModel::observationDifference(const arma::vec& a, const arma::vec& b) {
                return a - b;
            }

            arma::mat::fixed<InverseDepthPointModel::size, InverseDepthPointModel::size> InverseDepthPointModel::processNoise() {
                return arma::eye(size, size) * 1e-6; //std::numeric_limits<double>::epsilon();
            }

            arma::vec InverseDepthPointModel::getFieldPosFromState(const arma::vec::fixed<size>& state){
                if(state[kRHO] <= 0){
                    NUClear::log("State at infinity:", state);
                    return arma::vec3();
                }
                arma::vec original_cam_pos = {state[kX], state[kY], state[kZ]};
                arma::vec m = utility::math::vision::directionVectorFromScreenAngular({state[kTHETA], state[kPHI]}).rows(0,2);
                return original_cam_pos + m / state[kRHO];
            }

        }
    }
}