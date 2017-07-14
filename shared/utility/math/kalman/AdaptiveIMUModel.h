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

#ifndef UTILITY_MATH_KALMAN_ADAPTIVEIMUMODEL_H
#define UTILITY_MATH_KALMAN_ADAPTIVEIMUMODEL_H

/* Inertial Motion Unit*/
#include <armadillo>
namespace utility {
namespace math {
    namespace kalman {

        class AdaptiveIMUModel {
            // Number of dimensions
            // State is 3 vectors:
            //    1.Store unit vector pointing globally down (gravity)
            //      Coordinate system (robot) (same as CM730 coords):
            //                  x = forward, out of chest
            //                  y = leftwards
            //                  z = robot upward, towards head
            //     2.Global x approximated by orthogonalisation, in same robot coordinate system
            //     3.Bias of gyroscope in x,y,z angle offsets
        public:
            static constexpr size_t size = 6;

            AdaptiveIMUModel() {}  // empty constructor

            arma::vec::fixed<size> timeUpdate(const arma::vec::fixed<size>& state,
                                              double deltaT,
                                              const arma::vec3& measurement);

            arma::vec predictedObservation(const arma::vec::fixed<size>& state);

            arma::vec observationDifference(const arma::vec& a, const arma::vec& b);

            arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);

            arma::mat::fixed<size, size> processNoise();

            static constexpr double processNoiseFactor = 1e-6;
        };
    }  // namespace kalman
}  // namespace math
}  // namespace utility
#endif
