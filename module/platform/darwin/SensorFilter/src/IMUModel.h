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

#ifndef UTILITY_MATH_KALMAN_IMUMODEL_H
#define UTILITY_MATH_KALMAN_IMUMODEL_H

/* Inertial Motion Unit*/
#include <armadillo>

namespace module {
    namespace platform {
        namespace darwin {

            class IMUModel {
            public:

                static constexpr double G = -9.80665;

                // The indicies for our vector
                static constexpr uint VX = 0;
                static constexpr uint VY = 1;
                static constexpr uint VZ = 2;
                static constexpr uint QW = 3;
                static constexpr uint QX = 4;
                static constexpr uint QY = 5;
                static constexpr uint QZ = 6;

                struct MeasurementType {
                    struct GYROSCOPE {};
                    struct ACCELEROMETER {};
                    struct FORWARD {};
                    struct UP {};
                };

                arma::vec processNoiseDiagonal;

                static constexpr size_t size = 7;

                IMUModel() {} // empty constructor

                arma::vec::fixed<size> timeUpdate(const arma::vec::fixed<size>& state, double deltaT);

                arma::vec3 predictedObservation(const arma::vec::fixed<size>& state);
                arma::vec3 predictedObservation(const arma::vec::fixed<size>& state, const MeasurementType::UP&);
                arma::vec3 predictedObservation(const arma::vec::fixed<size>& state, const MeasurementType::ACCELEROMETER&);
                arma::vec3 predictedObservation(const arma::vec::fixed<size>& state, const MeasurementType::GYROSCOPE&);

                arma::vec observationDifference(const arma::vec& a, const arma::vec& b);

                arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);

                arma::mat::fixed<size, size> processNoise();
            };

        }
    }
}
#endif
