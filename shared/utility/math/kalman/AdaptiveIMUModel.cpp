/*
 * This file is part of the Autocalibration Codebase.
 *
 * The Autocalibration Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Autocalibration Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Autocalibration Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include <math.h> //needed for normalisation function
#include <assert.h>
#include "AdaptiveIMUModel.h" //includes armadillo

#include <iostream>
// #include <nuclear>

namespace utility {
    namespace math {
        namespace kalman {
            Eigen::Matrix<double, AdaptiveIMUModel::size, 1> AdaptiveIMUModel::limitState(const Eigen::Matrix<double, size, 1>& state) {

                arma::mat stateMatrix = arma::reshape(state,3,2);
                double normDown = stateMatrix.col(0).norm();
                if(normDown == 0){
                    //TODO: RESTART FILTER
                    // NUClear::log<NUClear::WARN>("AdaptiveIMUModel::limitState - Down vector has zero length!!!");
                    return state;
                }

                stateMatrix.col(0) = stateMatrix.col(0) / normDown;     //Normalise down

                double dotProd = arma::dot(stateMatrix.col(0), stateMatrix.col(1));
                //Angle for checking:
                double angle = acos(dotProd) * 180 / M_PI;
                if(angle < 45 || angle > 135){
                    // NUClear::log<NUClear::WARN>("AdaptiveIMUModel::limitState - New forward and down vectors are close: angle =", angle, "(degrees) ", angle < 45 ? "( < 45 degrees)" : "( > 135 degrees)");
                }
                stateMatrix.col(1) = stateMatrix.col(1) - stateMatrix.col(0)*dotProd;       //Orthogonalise forward and down

                double normForward = stateMatrix.col(1).norm();     //Normalise forward
                if(normForward == 0){
                    //TODO: RESTART FILTER
                    // NUClear::log<NUClear::WARN>("AdaptiveIMUModel::limitState - Projected forward vector has zero length!!!");
                    return state;
                }
                stateMatrix.col(1) = stateMatrix.col(1) / normForward;


                return static_cast<Eigen::Matrix<double, size, 1>>(arma::vectorise(stateMatrix));
            }

            // @brief The process equation is used to update the systems state using the process euquations of the system.
            // @param sigma_point The sigma point representing a system state.
            // @param deltaT The amount of time that has passed since the previous update, in seconds.
            // @param measurement The reading from the rate gyroscope in rad/s used to update the orientation.
            // @return The new estimated system state.
            Eigen::Matrix<double, AdaptiveIMUModel::size, 1> AdaptiveIMUModel::timeUpdate(const Eigen::Matrix<double, size, 1>& state, double deltaT, const Eigen::Vector3d& measurement) {
                //new universal rotation code for gyro (SORA)
                //See: http://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Simultaneous_orthogonal_rotation_angle
                arma::mat stateMatrix = arma::reshape(state,3,2);
                Eigen::Vector3d omega = measurement * deltaT;    //Offset applied
                double phi = omega.norm();
                if (phi == 0) {
                    return state;
                }
                Eigen::Vector3d unitOmega = omega / phi;

                const auto omegaCrossStateDown = arma::cross(unitOmega,stateMatrix.col(0));
                stateMatrix.col(0) = stateMatrix.col(0) * cos(phi) + omegaCrossStateDown * sin(phi) + unitOmega * arma::dot(unitOmega, stateMatrix.col(0)) * (1.0 - cos(phi));

                const auto omegaCrossStateForward = arma::cross(unitOmega,stateMatrix.col(1));
                stateMatrix.col(1) = stateMatrix.col(1) * cos(phi) + omegaCrossStateForward * sin(phi) + unitOmega * arma::dot(unitOmega, stateMatrix.col(1)) * (1.0 - cos(phi));

                return static_cast<Eigen::Matrix<double, size, 1>>(arma::vectorise(stateMatrix));
            }


            Eigen::VectorXd AdaptiveIMUModel::predictedObservation(const Eigen::Matrix<double, size, 1>& state) {
                return state.rows(0,2) * 9.807;
            }


            Eigen::VectorXd AdaptiveIMUModel::observationDifference(const arma::vec& a, const arma::vec& b) {
                return a - b;
            }

            arma::mat::fixed<AdaptiveIMUModel::size, AdaptiveIMUModel::size> AdaptiveIMUModel::processNoise() {
                return Eigen::Matrix<double, size, size>::Identity() * processNoiseFactor; //std::numeric_limits<double>::epsilon();
            }

        }
    }
}
