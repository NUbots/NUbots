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
 * Copyright 2019 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_FILTER_KALMANFILTER_HPP
#define UTILITY_MATH_FILTER_KALMANFILTER_HPP

#include <Eigen/Cholesky>
#include <Eigen/Core>

namespace utility::math::filter {

    template <typename Scalar, size_t n_states, size_t n_inputs, size_t n_outputs>
    class KalmanFilter {
    private:
        // @brief The continuous time state transition model
        Eigen::Matrix<Scalar, n_states, n_states> A = Eigen::Matrix<Scalar, n_states, n_states>::Zero();

        // @brief The continuous time input model
        Eigen::Matrix<Scalar, n_states, n_inputs> B = Eigen::Matrix<Scalar, n_states, n_inputs>::Zero();

        // @brief The process noise covariance matrix
        Eigen::Matrix<Scalar, n_states, n_states> Q = Eigen::Matrix<Scalar, n_states, n_states>::Identity();

        // @brief The measurement model
        Eigen::Matrix<Scalar, n_outputs, n_states> C = Eigen::Matrix<Scalar, n_outputs, n_states>::Zero();

        // @brief The measurement noise covariance matrix
        Eigen::Matrix<Scalar, n_outputs, n_outputs> R = Eigen::Matrix<Scalar, n_outputs, n_outputs>::Identity();

        // @brief The filters current state estimate
        Eigen::Matrix<Scalar, n_states, 1> x = Eigen::Matrix<Scalar, n_states, 1>::Zero();

        // @brief The filters current state covariance estimate
        Eigen::Matrix<Scalar, n_states, n_states> P = Eigen::Matrix<Scalar, n_states, n_states>::Identity();

        /**
         * @brief Performs zero-order hold discretization on the state transition matrix A and input matrix B
         *
         * @param A The continuous time state transition matrix
         * @param B The continuous time input matrix
         */
        void zero_order_hold(Eigen::Matrix<Scalar, n_states, n_states>& Ad,
                             Eigen::Matrix<Scalar, n_states, n_inputs>& Bd,
                             const Scalar& dt) {
            Ad = Eigen::Matrix<Scalar, n_states, n_states>::Identity() + dt * A;
            Bd = dt * B;
        }


        /**
         * @brief Performs a correction step on the filter using the given measurement
         *
         * @param y The measurement vector
         */
        void correct(const Eigen::Matrix<Scalar, n_outputs, 1>& y) {
            // Calculate the optimal Kalman gain
            const Eigen::Matrix<Scalar, n_states, n_outputs> K =
                P * C.transpose() * (C * P * C.transpose() + R).inverse();

            // Update the state estimate
            x += K * (y - C * x);

            // Update the state covariance estimate
            P = (Eigen::Matrix<Scalar, n_states, n_states>::Identity() - K * C) * P;
        }

        /**
         * @brief Performs a prediction step on the filter using the given control input
         *
         * @param u The control input vector
         * @param dt The time since the last prediction
         */
        void predict(const Eigen::Matrix<Scalar, n_inputs, 1>& u, const Scalar& dt) {
            // Discretize the A and B matrices
            Eigen::Matrix<Scalar, n_states, n_states> Ad;
            Eigen::Matrix<Scalar, n_states, n_inputs> Bd;
            zero_order_hold(Ad, Bd, dt);

            // Update the state estimate
            x = Ad * x + Bd * u;

            // Update the state covariance estimate
            P = Ad * P * Ad.transpose() + Q;
        }


    public:
        /**
         * @brief Construct a new Kalman Filter
         */
        KalmanFilter(
            Eigen::Matrix<Scalar, n_states, n_states> A   = Eigen::Matrix<Scalar, n_states, n_states>::Identity(),
            Eigen::Matrix<Scalar, n_states, n_inputs> B   = Eigen::Matrix<Scalar, n_states, n_inputs>::Zero(),
            Eigen::Matrix<Scalar, n_outputs, n_states> C  = Eigen::Matrix<Scalar, n_outputs, n_states>::Identity(),
            Eigen::Matrix<Scalar, n_states, n_states> Q   = Eigen::Matrix<Scalar, n_states, n_states>::Identity(),
            Eigen::Matrix<Scalar, n_outputs, n_outputs> R = Eigen::Matrix<Scalar, n_outputs, n_outputs>::Identity())
            : A(A), B(B), Q(Q), C(C), R(R) {}


        /**
         * @brief Initialise the filter with the given state, covariance, process noise and measurement noise
         * @param initial_state The initial state
         * @param initial_covariance The initial state covariance
         */
        void init(const Eigen::Matrix<Scalar, n_states, 1>& initial_state,
                  const Eigen::Matrix<Scalar, n_states, n_states>& initial_covariance) {
            x = initial_state;
            P = initial_covariance;
        }

        /**
         * @brief Run the filter on the given measurement and control input
         * @param y The measurement vector
         * @param u The control input vector
         * @param dt The time since the update
         */
        void run(const Eigen::Matrix<Scalar, n_inputs, 1>& u,
                 const Eigen::Matrix<Scalar, n_outputs, 1>& y,
                 const Scalar& dt) {
            // Perform prediction step
            predict(u, dt);
            // Perform correction step
            correct(y);
        }

        /**
         * @brief Get the current state estimate
         * @return The current state estimate
         */
        Eigen::Matrix<Scalar, n_states, 1> get_state() const {
            return x;
        };

        /**
         * @brief Get the current state covariance estimate
         * @return The current state covariance estimate
         */
        Eigen::Matrix<Scalar, n_states, n_states> get_covariance() const {
            return P;
        };
    };


}  // namespace utility::math::filter
#endif
