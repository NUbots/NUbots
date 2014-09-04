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
#include "LinearVec3Model.h" //includes armadillo

#include <iostream>
#include <nuclear>

namespace utility {
    namespace math {
        namespace kalman {
            arma::vec::fixed<LinearVec3Model::size> LinearVec3Model::limitState(const arma::vec::fixed<size>& state) {
                return state;
            }


            arma::vec::fixed<LinearVec3Model::size> LinearVec3Model::timeUpdate(const arma::vec::fixed<size>& state, double deltaT, const arma::vec3& dState) {
                return state + deltaT * dState;
            }


            arma::vec LinearVec3Model::predictedObservation(const arma::vec::fixed<size>& state, std::nullptr_t) {
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