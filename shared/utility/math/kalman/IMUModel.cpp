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

#include "IMUModel.h"

namespace utility {
    namespace math {
        namespace kalman {
            arma::vec::fixed<IMUModel::size> IMUModel::limitState(const arma::vec::fixed<size>& state) {
                arma::vec::fixed<size> newState = state;
                newState.rows(QW, QZ) = arma::normalise(newState.rows(QW, QZ));
                return newState;
            }

            // @brief The process equation is used to update the systems state using the process euquations of the system.
            // @param sigma_point The sigma point representing a system state.
            // @param deltaT The amount of time that has passed since the previous update, in seconds.
            // @param measurement The reading from the rate gyroscope in rad/s used to update the orientation.
            // @return The new estimated system state.
            arma::vec::fixed<IMUModel::size> IMUModel::timeUpdate(const arma::vec::fixed<size>& state, double deltaT) {

                arma::mat v(3, 4);

                v << -state[QX] <<   state[QW] <<  state[QZ]  << -state[QY]  << arma::endr
                  << -state[QY] <<  -state[QZ] <<  state[QW]  <<  state[QX]  << arma::endr
                  << -state[QZ] <<   state[QY] << -state[QX]  <<  state[QW];

                arma::vec::fixed<IMUModel::size> newState;

                newState = state;
                newState.rows(QW, QZ) += 0.5 * deltaT * v.t() * state.rows(VX, VZ);
                return newState;

            }


            // Accelerometer
            arma::vec3 IMUModel::predictedObservation(const arma::vec::fixed<size>& state, float x) {

                arma::vec3 down = { 2 * state[QX] * state[QZ] + 2 * state[QY] * state[QW]
                                  , 2 * state[QY] * state[QZ] - 2 * state[QX] * state[QW]
                                  , 1 - 2 * state[QX] * state[QX] - 2 * state[QY] * state[QY] };

                down *= G;

                return down;
            }

            // Gyroscope
            arma::vec3 IMUModel::predictedObservation(const arma::vec::fixed<size>& state, int y) {
                return state.rows(VX, VZ);
            }


            // Forward Vector
            arma::vec3 IMUModel::predictedObservation(const arma::vec::fixed<size>& state) {

                return { 1 - 2 * state[QY] * state[QY] - 2 * state[QZ] * state[QZ]
                       , 2 * state[QX] * state[QY] + 2 * state[QZ] * state[QW]
                       , 2 * state[QX] * state[QZ] - 2 * state[QY] * state[QW] };
            }

            arma::vec IMUModel::observationDifference(const arma::vec& a, const arma::vec& b) {
                return a - b;

            }

            arma::mat::fixed<IMUModel::size, IMUModel::size> IMUModel::processNoise() {
                return arma::eye(size, size) * processNoiseFactor;
            }

        }
    }
}