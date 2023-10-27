/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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

#ifndef UTILITY_MATH_FILTER_KALMAN_HPP
#define UTILITY_MATH_FILTER_KALMAN_HPP

#include <Eigen/Cholesky>
#include <Eigen/Core>

namespace utility::math::filter {

    template <typename Scalar, int NumStates, int NumInputs, int NumMeasurements>
    class KalmanFilter {
    public:
        // Dimension types for vectors and matrices
        using StateVec      = Eigen::Matrix<Scalar, NumStates, 1>;
        using StateMat      = Eigen::Matrix<Scalar, NumStates, NumStates>;
        using InputVec      = Eigen::Matrix<Scalar, NumInputs, 1>;
        using InputMat      = Eigen::Matrix<Scalar, NumInputs, NumInputs>;
        using MeasVec       = Eigen::Matrix<Scalar, NumMeasurements, 1>;
        using MeasMat       = Eigen::Matrix<Scalar, NumMeasurements, NumMeasurements>;
        using GainMat       = Eigen::Matrix<Scalar, NumStates, NumMeasurements>;
        using MeasModelMat  = Eigen::Matrix<Scalar, NumMeasurements, NumStates>;
        using InputModelMat = Eigen::Matrix<Scalar, NumStates, NumInputs>;

        /// @brief Default constructor for KalmanFilter
        KalmanFilter() = default;

        /// @brief Construct a KalmanFilter with the given parameters
        KalmanFilter(StateVec initial_state,
                     StateMat initial_covariance,
                     StateMat state_transition_matrix,
                     InputModelMat input_model_matrix,
                     MeasModelMat measurement_model_matrix,
                     StateMat process_noise_covariance,
                     MeasMat measurement_noise_covariance)
            : state(initial_state)
            , covariance(initial_covariance)
            , A(state_transition_matrix)
            , B(input_model_matrix)
            , C(measurement_model_matrix)
            , Q(process_noise_covariance)
            , R(measurement_noise_covariance) {}


        /**
         * @brief Update the filter parameters.
         *
         * @param[in] state_transition_matrix
         * @param[in] input_model_matrix
         * @param[in] measurement_model_matrix
         * @param[in] process_noise_covariance
         * @param[in] measurement_noise_covariance
         */
        void update(StateMat state_transition_matrix,
                    InputModelMat input_model_matrix,
                    MeasModelMat measurement_model_matrix,
                    StateMat process_noise_covariance,
                    MeasMat measurement_noise_covariance) {
            A = state_transition_matrix;
            B = input_model_matrix;
            C = measurement_model_matrix;
            Q = process_noise_covariance;
            R = measurement_noise_covariance;
        }

        /**
         * @brief Resets the filter to a new initial state and covariance.
         *
         * @param[in] initial_mean
         * @param[in] initial_covariance
         * @return Eigen::ComputationInfo indicating success or failure of the reset.
         */
        Eigen::ComputationInfo reset(StateVec initial_mean, StateMat initial_covariance) {
            state      = initial_mean;
            covariance = initial_covariance;
            return Eigen::Success;
        }

        /**
         * @brief Performs a time update on the state and covariance using the system model and control input.
         *
         * @param[in] control_input
         * @param[in] dt
         */
        void time(const InputVec& control_input, const Scalar& dt) {
            Eigen::Matrix<Scalar, NumStates, NumStates> Ad;
            Eigen::Matrix<Scalar, NumStates, NumInputs> Bd;
            zero_order_hold(Ad, Bd, dt);
            state      = Ad * state + Bd * control_input;
            covariance = Ad * covariance * Ad.transpose() + Q;
        }

        /**
         * @brief Performs a measurement update on the state and covariance using the measurement model and measurement.
         *
         * @param[in] measurement
         */
        void measure(const MeasVec& measurement) {
            // Calculate the optimal Kalman gain
            GainMat K = covariance * C.transpose() * (C * covariance * C.transpose() + R).inverse();
            // Update the state estimate
            state += K * (measurement - C * state);
            // Update the covariance estimate
            covariance = (StateMat::Identity() - K * C) * covariance * (StateMat::Identity() - K * C).transpose()
                         + K * R * K.transpose();
        }

        /// @brief Get the current state estimate
        const StateVec& get_state() const {
            return state;
        }

        /// @brief Get the current covariance
        const StateMat& get_covariance() const {
            return covariance;
        }

    private:
        /**
         * @brief Discretises the system model using a zero-order hold method.
         *
         * @param[out] Ad Discretised state transition matrix.
         * @param[out] Bd Discretised input model matrix.
         * @param[in] dt Timestep to discretise with.
         */
        void zero_order_hold(Eigen::Matrix<Scalar, NumStates, NumStates>& Ad,
                             Eigen::Matrix<Scalar, NumStates, NumInputs>& Bd,
                             const Scalar& dt) {
            Ad = Eigen::Matrix<Scalar, NumStates, NumStates>::Identity() + dt * A;
            Bd = dt * B;
        }

        /// @brief Current estimate of the system state
        StateVec state = StateVec::Zero();

        /// @brief Current covariance of the system state
        StateMat covariance = StateMat::Identity();

        /// @brief State transition matrix
        StateMat A = StateMat::Identity();

        /// @brief Input model matrix
        InputModelMat B = InputModelMat::Zero();

        /// @brief Measurement model matrix
        MeasModelMat C = MeasModelMat::Zero();

        /// @brief Process noise covariance matrix
        StateMat Q = StateMat::Identity();

        /// @brief Measurement noise covariance matrix
        MeasMat R = MeasMat::Identity();
    };

}  // namespace utility::math::filter

#endif
