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

#ifndef MODULE_TOOLS_VANDERPOLMODEL_H
#define MODULE_TOOLS_VANDERPOLMODEL_H

#include <Eigen/Core>

namespace module {
namespace tools {

    template <typename Scalar>
    class VanDerPolModel {
    public:
        enum Components {
            X1 = 0,
            X2 = 1,
        };

        static constexpr size_t size = 2;

        using StateVec = arma::vec::fixed<size>;
        using StateMat = arma::mat::fixed<size, size>;

        StateVec process_noise;

        // number and range of reset particles
        int n_rogues         = 0;
        StateVec reset_range = arma::zeros(size);

        StateVec timeUpdate(const StateVec& state, const Scalar& deltaT) {
            StateVec new_state({state[X2], (Scalar(1) - state[X1] * state[X1]) * state[X2] - state[X1]});

            return state + new_state * deltaT;
        }

        StateMat processNoise() {
            return arma::diagmat(process_noise);
        }

        template <typename T, typename U>
        auto observationDifference(const T& a, const U& b) {
            return a - b;
        }

        typename arma::Col<Scalar>::template fixed<1> predictedObservation(const StateVec& state) {
            // Our prediction is the first state
            return {state[X1]};
        }

        StateVec limitState(const StateVec& state) {
            return state;
        }

        // Getters
        int getRogueCount() const {
            return n_rogues;
        }
        StateVec getRogueRange() const {
            return reset_range;
        }
    };
}  // namespace tools
}  // namespace module
#endif  // MODULE_TOOLS_VANDERPOLMODEL_H
