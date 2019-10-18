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

#ifndef SHARED_TESTS_PFVANDERPOLMODEL_H
#define SHARED_TESTS_PFVANDERPOLMODEL_H

#include <Eigen/Core>

namespace shared {
namespace tests {

    // This class implements a van der Pol oscillator with mu = 1.
    // This is a two state system and is described by the following set of non-linear ODEs
    // \dot{x_{1}} = x_{2}
    // \dot{x_{2}} = (1 - x_{1}^{2}) * x_{2} - x_{1}
    //
    // This class is based on the implementation described at
    // https://au.mathworks.com/help/control/ug/nonlinear-state-estimation-using-unscented-kalman-filter.html
    template <typename Scalar>
    class PFVanDerPolModel {
    public:
        enum Components {
            X1 = 0,
            X2 = 1,
        };

        static constexpr size_t size = 2;

        using StateVec = Eigen::Matrix<Scalar, size, 1>;
        using StateMat = Eigen::Matrix<Scalar, size, size>;

        StateVec process_noise;

        // number and range of reset particles
        int n_rogues         = 0;
        StateVec reset_range = StateVec::Zero();

        StateVec time(const StateVec& state, const Scalar& deltaT) {
            // Evaluate the van der Pol ODEs for mu = 1
            StateVec new_state(state[X2], (Scalar(1) - state[X1] * state[X1]) * state[X2] - state[X1]);

            // Euler integration of continuous-time dynamics x' = f(x) with sample time deltaT
            return state + new_state * deltaT;
        }

        StateMat noise(const Scalar& deltaT) {
            // Return our process noise matrix
            return process_noise.asDiagonal() * deltaT;
        }

        template <typename T, typename U>
        auto difference(const T& a, const U& b) {
            return a - b;
        }

        Eigen::Matrix<Scalar, 1, 1> predict(const StateVec& state) {
            // Our prediction is the first state
            return Eigen::Matrix<Scalar, 1, 1>(state[X1]);
        }

        StateVec limit(const StateVec& state) {
            // Don't apply any limitting
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
}  // namespace tests
}  // namespace shared
#endif  // SHARED_TESTS_PFVANDERPOLMODEL_H
