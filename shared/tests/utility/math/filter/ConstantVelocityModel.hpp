/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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

#ifndef SHARED_TESTS_CONSTANTVELOCITYMODEL_HPP
#define SHARED_TESTS_CONSTANTVELOCITYMODEL_HPP

#include <Eigen/Core>

namespace shared::tests {

    /// A linear 1D constant-velocity model (position, velocity) with white-noise-acceleration process noise and a
    /// position measurement. On a linear model a UKF must give exactly the Kalman filter's answer.
    template <typename Scalar>
    class ConstantVelocityModel {
    public:
        static constexpr size_t size = 2;

        using StateVec = Eigen::Matrix<Scalar, size, 1>;
        using StateMat = Eigen::Matrix<Scalar, size, size>;

        /// White-noise-acceleration spectral density
        Scalar acceleration_noise = Scalar(1);

        [[nodiscard]] static StateMat transition(const Scalar& deltaT) {
            StateMat F = StateMat::Identity();
            F(0, 1)    = deltaT;
            return F;
        }

        StateVec time(const StateVec& state, const Scalar& deltaT) {
            return transition(deltaT) * state;
        }

        StateMat noise(const Scalar& deltaT) {
            StateMat Q;
            Q << deltaT * deltaT * deltaT / Scalar(3), deltaT * deltaT / Scalar(2), deltaT * deltaT / Scalar(2), deltaT;
            return acceleration_noise * Q;
        }

        template <typename T, typename U>
        auto difference(const T& a, const U& b) {
            return a - b;
        }

        Eigen::Matrix<Scalar, 1, 1> predict(const StateVec& state) {
            return Eigen::Matrix<Scalar, 1, 1>(state[0]);
        }

        StateVec limit(const StateVec& state) {
            return state;
        }
    };
}  // namespace shared::tests
#endif  // SHARED_TESTS_CONSTANTVELOCITYMODEL_HPP
