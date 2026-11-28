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

#ifndef UTILITY_MATH_FILTER_SRIF_HPP
#define UTILITY_MATH_FILTER_SRIF_HPP

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/QR>

namespace utility::math::filter {

    /**
     * @brief A square-root information filter (SRIF) for a linear(ised) Gaussian system.
     *
     * The Gaussian is stored as its square-root information matrix Xi and vector nu, where Xi^T Xi = P^-1 and
     * Xi mu = nu. Both the time and the measurement update are a single QR factorisation, so the covariance is positive
     * semi-definite by construction and the filter stays well conditioned when the information is very uneven (e.g. a
     * target seen tightly across the line of sight but loosely along it).
     *
     * For a nonlinear measurement y = h(x) + v, linearise at the current mean and pass z = y - h(mu) + H mu: one
     * measure() call is then the extended Kalman filter update in square-root information form.
     *
     * Ported from the MATLAB cooperative tracking evaluation (soccer-sim) by Sebastian Brooker.
     *
     * @tparam Scalar The scalar type of the filter
     * @tparam N      The number of states
     */
    template <typename Scalar, int N>
    class SRIF {
    public:
        using StateVec = Eigen::Matrix<Scalar, N, 1>;
        using StateMat = Eigen::Matrix<Scalar, N, N>;

        SRIF() = default;

        /// @brief Construct the filter from a mean and covariance
        SRIF(const StateVec& mean, const StateMat& covariance) {
            set_state(mean, covariance);
        }

        /// @brief Set the filter state from a mean and covariance
        /// @param mean       The mean of the state
        /// @param covariance The covariance of the state, must be positive definite
        void set_state(const StateVec& mean, const StateMat& covariance) {
            // With P = L L^T, Xi = L^-1 gives Xi^T Xi = P^-1. QR then makes it upper triangular.
            const StateMat L  = covariance.llt().matrixL();
            const StateMat Xi = L.template triangularView<Eigen::Lower>().solve(StateMat::Identity());
            Eigen::Matrix<Scalar, N, N + 1> A;
            A << Xi, Xi * mean;
            set_from_qr(A);
        }

        /// @brief The mean of the state
        [[nodiscard]] StateVec mean() const {
            return Xi.template triangularView<Eigen::Upper>().solve(nu);
        }

        /// @brief The covariance of the state
        [[nodiscard]] StateMat covariance() const {
            const StateMat Xi_inv = Xi.template triangularView<Eigen::Upper>().solve(StateMat::Identity());
            return Xi_inv * Xi_inv.transpose();
        }

        /// @brief The square-root information matrix Xi (upper triangular, Xi^T Xi = P^-1)
        [[nodiscard]] const StateMat& sqrt_info_matrix() const {
            return Xi;
        }

        /// @brief The square-root information vector nu (Xi mu = nu)
        [[nodiscard]] const StateVec& sqrt_info_vector() const {
            return nu;
        }

        /**
         * @brief Time update for x+ = F x + G w, w ~ N(0, Q)
         *
         * Uses the square-root information time update: with Rw the square-root information matrix of w, factorise
         *   [ Rw            0          0  ]         [ *   *    *   ]
         *   [ -Xi F^-1 G    Xi F^-1    nu ]  --QR-> [ 0   Xi+  nu+ ]
         * which marginalises the process noise out of the joint information of (w, x+).
         *
         * @tparam M The dimension of the process noise
         * @param F  The state transition matrix, must be invertible
         * @param G  The process noise input matrix
         * @param Q  The covariance of the process noise, must be positive definite
         */
        template <int M>
        void time(const StateMat& F, const Eigen::Matrix<Scalar, N, M>& G, const Eigen::Matrix<Scalar, M, M>& Q) {
            using NoiseMat        = Eigen::Matrix<Scalar, M, M>;
            const NoiseMat Lq     = Q.llt().matrixL();
            const NoiseMat Rw     = Lq.template triangularView<Eigen::Lower>().solve(NoiseMat::Identity());
            const StateMat XiFinv = F.transpose().partialPivLu().solve(Xi.transpose()).transpose();
            Eigen::Matrix<Scalar, M + N, M + N + 1> A = Eigen::Matrix<Scalar, M + N, M + N + 1>::Zero();
            A.template block<M, M>(0, 0)              = Rw;
            A.template block<N, M>(M, 0)              = -XiFinv * G;
            A.template block<N, N>(M, M)              = XiFinv;
            A.template block<N, 1>(M, M + N)          = nu;

            Eigen::HouseholderQR<Eigen::Matrix<Scalar, M + N, M + N + 1>> qr(A);
            const Eigen::Matrix<Scalar, M + N, M + N + 1> T = qr.matrixQR().template triangularView<Eigen::Upper>();
            Eigen::Matrix<Scalar, N, N + 1> B;
            B << T.template block<N, N>(M, M), T.template block<N, 1>(M, M + N);
            normalise_signs(B);
        }

        /**
         * @brief Measurement update for z = H x + v, v ~ N(0, R)
         *
         * With W the whitening matrix of R (W R W^T = I), factorise
         *   [ Xi    nu  ]         [ Xi+  nu+ ]
         *   [ W H   W z ]  --QR-> [ 0    *   ]
         *
         * @tparam M The dimension of the measurement
         * @param z  The (linearised) measurement
         * @param H  The measurement matrix
         * @param R  The covariance of the measurement noise, must be positive definite
         * @return false if the update produced non-finite values, in which case the state is left unchanged
         */
        template <int M>
        bool measure(const Eigen::Matrix<Scalar, M, 1>& z,
                     const Eigen::Matrix<Scalar, M, N>& H,
                     const Eigen::Matrix<Scalar, M, M>& R) {
            using MeasMat    = Eigen::Matrix<Scalar, M, M>;
            const MeasMat Lr = R.llt().matrixL();
            const MeasMat W  = Lr.template triangularView<Eigen::Lower>().solve(MeasMat::Identity());
            Eigen::Matrix<Scalar, N + M, N + 1> A;
            A << Xi, nu, W * H, W * z;

            Eigen::HouseholderQR<Eigen::Matrix<Scalar, N + M, N + 1>> qr(A);
            const Eigen::Matrix<Scalar, N + M, N + 1> T = qr.matrixQR().template triangularView<Eigen::Upper>();
            const Eigen::Matrix<Scalar, N, N + 1> B     = T.template topRows<N>();
            if (!B.allFinite()) {
                return false;
            }
            normalise_signs(B);
            return true;
        }

    private:
        /// @brief Square-root information matrix, upper triangular with a positive diagonal
        StateMat Xi = StateMat::Identity();
        /// @brief Square-root information vector
        StateVec nu = StateVec::Zero();

        /// @brief Set [Xi nu] from the upper triangular factor of a QR of A = [Xi' nu']
        void set_from_qr(const Eigen::Matrix<Scalar, N, N + 1>& A) {
            Eigen::HouseholderQR<Eigen::Matrix<Scalar, N, N + 1>> qr(A);
            const Eigen::Matrix<Scalar, N, N + 1> T = qr.matrixQR().template triangularView<Eigen::Upper>();
            normalise_signs(T);
        }

        /// @brief Store [Xi nu] = B, flipping row signs so the diagonal of Xi is positive (a QR is unique only up to
        /// the sign of each row)
        void normalise_signs(Eigen::Matrix<Scalar, N, N + 1> B) {
            for (int i = 0; i < N; ++i) {
                if (B(i, i) < Scalar(0)) {
                    B.row(i) *= Scalar(-1);
                }
            }
            Xi = B.template leftCols<N>().template triangularView<Eigen::Upper>();
            nu = B.col(N);
        }
    };

}  // namespace utility::math::filter

#endif  // UTILITY_MATH_FILTER_SRIF_HPP
