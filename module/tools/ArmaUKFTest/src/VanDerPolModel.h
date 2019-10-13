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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_TOOLS_VANDERPOLMODEL_H
#define MODULE_TOOLS_VANDERPOLMODEL_H

#include <armadillo>

namespace module {
namespace tools {

    template <typename Scalar>
    class VanDerPolModel {
    public:
        enum Values {
            X1 = 0,
            X2 = 1,
        };

        // The size of our state
        static constexpr size_t size = 2;

        using StateVec = arma::vec::fixed<size>;
        using StateMat = arma::mat::fixed<size, size>;

        // Our static process noise matrix
        StateVec process_noise;

        StateVec timeUpdate(const StateVec& state, Scalar deltaT) {
            StateVec new_state({state[X2], (Scalar(1) - state[X1] * state[X1]) * state[X2] - state[X1]});
            return state + new_state * deltaT;
        }

        typename arma::Col<Scalar>::template fixed<1> predictedObservation(const StateVec& state) {
            return {state[X1]};
        }

        template <typename T, typename U>
        auto observationDifference(const T& a, const U& b) {
            return a - b;
        }

        StateVec limitState(const StateVec& state) {
            StateVec newState = state;
            return newState;
        }

        StateMat processNoise() {
            // Return our process noise matrix
            return arma::diagmat(process_noise);
        }
    };
}  // namespace tools
}  // namespace module

#endif  // MODULE_TOOLS_VANDERPOLMODEL_H
