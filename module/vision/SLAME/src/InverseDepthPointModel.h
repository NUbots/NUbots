/* Inverse Depth Parameterisation for SLAME
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

#ifndef UTILITY_MATH_KALMAN_INVERSE_DEPTH_MODEL_H
#define UTILITY_MATH_KALMAN_INVERSE_DEPTH_MODEL_H

#include <armadillo>
#include "message/localisation/FieldObject.h"
#include "message/input/Sensors.h"

namespace utility {
    namespace math {
        namespace kalman {
            enum IDPStateComponents{
                kX = 0,
                kY = 1,
                kZ = 2,
                kRHO = 3,
                kTHETA = 4,
                kPHI = 5
            };

            class InverseDepthPointModel {
            public:
                //(x,y,z,rho,theta,phi)
                static constexpr size_t size = 6;

                InverseDepthPointModel() {} // empty constructor

                arma::vec::fixed<size> timeUpdate(const arma::vec::fixed<size>& state, const double& deltaT);

                static arma::vec predictedObservation(const arma::vec::fixed<size>& state, const arma::mat& worldToCamera_camera);

                arma::vec observationDifference(const arma::vec& a, const arma::vec& b);

                arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);

                arma::mat::fixed<size, size> processNoise();

                static arma::vec getFieldPosFromState(const arma::vec::fixed<size>& state);

            };


        }
    }
}
#endif
