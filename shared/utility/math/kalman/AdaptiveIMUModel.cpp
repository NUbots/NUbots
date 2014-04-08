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
#include "AdaptiveIMUModel.h" //includes armadillo

#include <iostream>
#include <nuclear>

namespace utility {
    namespace math {
        namespace kalman {
            arma::vec::fixed<AdaptiveIMUModel::size> AdaptiveIMUModel::limitState(const arma::vec::fixed<size>& state) {

                arma::mat stateMatrix = arma::reshape(state,3,2);
                double normDown = arma::norm(stateMatrix.col(0),2);
                if(normDown == 0){
                    //TODO: RESTART FILTER
                    NUClear::log<NUClear::WARN>("AdaptiveIMUModel::limitState - Down vector has zero length!!!");
                    return state;
                }

                stateMatrix.col(0) = stateMatrix.col(0) / normDown;     //Normalise down

                double dotProd = arma::dot(stateMatrix.col(0), stateMatrix.col(1));
                //Angle for checking:
                double angle = acos(dotProd) * 180 / M_PI;
                if(angle < 45 || angle > 135){
                    NUClear::log<NUClear::WARN>("AdaptiveIMUModel::limitState - New forward and down vectors are close: angle =", angle, "(degrees) ", angle < 45 ? "( < 45 degrees)" : "( > 135 degrees)");
                }
                stateMatrix.col(1) = stateMatrix.col(1) - stateMatrix.col(0)*dotProd;       //Orthogonalise forward and down

                double normForward = arma::norm(stateMatrix.col(1),2);     //Normalise forward
                if(normForward == 0){
                    //TODO: RESTART FILTER
                    NUClear::log<NUClear::WARN>("AdaptiveIMUModel::limitState - Projected forward vector has zero length!!!");
                    return state;
                }
                stateMatrix.col(1) = stateMatrix.col(1) / normForward;


                return static_cast<arma::vec::fixed<size>>(arma::vectorise(stateMatrix));
            }

            // @brief The process equation is used to update the systems state using the process euquations of the system.
            // @param sigma_point The sigma point representing a system state.
            // @param deltaT The amount of time that has passed since the previous update, in seconds.
            // @param measurement The reading from the rate gyroscope in rad/s used to update the orientation.
            // @return The new estimated system state.
            arma::vec::fixed<AdaptiveIMUModel::size> AdaptiveIMUModel::timeUpdate(const arma::vec::fixed<size>& state, double deltaT, const arma::vec3& measurement) {
                //new universal rotation code for gyro (SORA)
                //See: http://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Simultaneous_orthogonal_rotation_angle
                arma::mat stateMatrix = arma::reshape(state,3,2);
                arma::vec3 omega = measurement * deltaT;    //Offset applied
                double phi = arma::norm(omega, 2);
                if (phi == 0) {
                    return state;
                }
                arma::vec3 unitOmega = omega / phi;

                const auto omegaCrossStateDown = arma::cross(unitOmega,stateMatrix.col(0));
                stateMatrix.col(0) = stateMatrix.col(0) * cos(phi) + omegaCrossStateDown * sin(phi) + unitOmega * arma::dot(unitOmega, stateMatrix.col(0)) * (1.0 - cos(phi));

                const auto omegaCrossStateForward = arma::cross(unitOmega,stateMatrix.col(1));
                stateMatrix.col(1) = stateMatrix.col(1) * cos(phi) + omegaCrossStateForward * sin(phi) + unitOmega * arma::dot(unitOmega, stateMatrix.col(1)) * (1.0 - cos(phi));

                return static_cast<arma::vec::fixed<size>>(arma::vectorise(stateMatrix));
            }


            arma::vec AdaptiveIMUModel::predictedObservation(const arma::vec::fixed<size>& state, std::nullptr_t) {
                return state.rows(0,2) * 9.807;
            }


            arma::vec AdaptiveIMUModel::observationDifference(const arma::vec& a, const arma::vec& b) {
                return a - b;
            }

            arma::mat::fixed<AdaptiveIMUModel::size, AdaptiveIMUModel::size> AdaptiveIMUModel::processNoise() {
                return arma::eye(size, size) * processNoiseFactor; //std::numeric_limits<double>::epsilon();
            }

        }
    }
}