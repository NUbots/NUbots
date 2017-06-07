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
#include "LinearVec3Model.h" //includes armadillo

#include <iostream>

namespace utility {
    namespace math {
        namespace kalman {
            Eigen::Matrix<double, LinearVec3Model::size, 1> LinearVec3Model::limitState(const Eigen::Matrix<double, size, 1>& state) {
                return state;
            }


            Eigen::Matrix<double, LinearVec3Model::size, 1> LinearVec3Model::timeUpdate(const Eigen::Matrix<double, size, 1>& state, double deltaT, const Eigen::Vector3d& dState) {
                return state + deltaT * dState;
            }


            arma::vec LinearVec3Model::predictedObservation(const Eigen::Matrix<double, size, 1>& state, std::nullptr_t) {
                return state;
            }


            arma::vec LinearVec3Model::observationDifference(const arma::vec& a, const arma::vec& b) {
                return a - b;
            }

            arma::mat::fixed<LinearVec3Model::size, LinearVec3Model::size> LinearVec3Model::processNoise() {
                return arma::eye(size, size) * processNoiseFactor; //std::numeric_limits<double>::epsilon();
            }

        }
    }
}
