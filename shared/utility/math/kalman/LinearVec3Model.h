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

#ifndef UTILITY_MATH_KALMAN_LINEARVEC3MODEL_H
#define UTILITY_MATH_KALMAN_LINEARVEC3MODEL_H

/* Inertial Motion Unit*/
#include <armadillo>
namespace utility {
namespace math {
    namespace kalman {

        class LinearVec3Model {
            // Number of dimensions
            // State is velocity of torso relative to the robot local ground matrix
            //
        public:
            static constexpr size_t size = 3;

            LinearVec3Model() {}  // empty constructor

            arma::vec::fixed<size> timeUpdate(const arma::vec::fixed<size>& state,
                                              double deltaT,
                                              const arma::vec3& measurement);

            arma::vec predictedObservation(const arma::vec::fixed<size>& state, std::nullptr_t);

            arma::vec observationDifference(const arma::vec& a, const arma::vec& b);

            arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);

            arma::mat::fixed<size, size> processNoise();

            const double processNoiseFactor = 1e-6;
        };
    }
}
}
#endif