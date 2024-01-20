/*
 * MIT License
 *
 * Copyright (c) 2019 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef SHARED_TESTS_VANDERPOLMODEL_HPP
#define SHARED_TESTS_VANDERPOLMODEL_HPP

#include <Eigen/Core>

namespace shared::tests {

    template <typename Scalar>
    class VanDerPolModel {
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

        int n_particles = 100;

        StateVec time(const StateVec& state, const Scalar& deltaT) {
            StateVec new_state(state[X2], (Scalar(1) - state[X1] * state[X1]) * state[X2] - state[X1]);
            return state + new_state * deltaT;
        }

        StateMat noise(const Scalar& deltaT) {
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
            return state;
        }

        // Getters
        int getRogueCount() const {
            return n_rogues;
        }
        [[nodiscard]] StateVec get_rogue() const {
            return reset_range;
        }

        [[nodiscard]] int getParticleCount() const {
            return n_particles;
        }
    };
}  // namespace shared::tests
#endif  // SHARED_TESTS_VANDERPOLMODEL_HPP
