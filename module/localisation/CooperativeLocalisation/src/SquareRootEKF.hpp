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
#ifndef MODULE_LOCALISATION_COOPERATIVELOCALISATION_SQUAREROOTEKF_HPP
#define MODULE_LOCALISATION_COOPERATIVELOCALISATION_SQUAREROOTEKF_HPP

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/QR>

namespace module::localisation {

    /// @brief Square-root (covariance form) extended Kalman filter: stores the mean and an upper triangular S with
    /// P = S^T S, and does both updates with a QR, so P stays positive definite. Linearise a nonlinear measurement
    /// y = h(x) at the mean and pass z = y - h(mean) + H mean.
    template <int N>
    class SquareRootEKF {
    public:
        using Vec = Eigen::Matrix<double, N, 1>;
        using Mat = Eigen::Matrix<double, N, N>;

        SquareRootEKF() = default;
        SquareRootEKF(const Vec& mean, const Mat& covariance) : x(mean), S(covariance.llt().matrixU()) {}

        [[nodiscard]] const Vec& mean() const {
            return x;
        }
        [[nodiscard]] Mat covariance() const {
            return S.transpose() * S;
        }

        /// @brief Time update x+ = F x + G w, w ~ N(0, Q): S+ = R factor of QR([S F^T; Sq G^T]), Q = Sq^T Sq
        template <int M>
        void time(const Mat& F, const Eigen::Matrix<double, N, M>& G, const Eigen::Matrix<double, M, M>& Q) {
            Eigen::Matrix<double, N + M, N> A;
            A << S * F.transpose(), Eigen::Matrix<double, M, M>(Q.llt().matrixU()) * G.transpose();
            x = F * x;
            S = upper(A).template topRows<N>();
        }

        /// @brief Measurement update for z = H x + v, v ~ N(0, R). QR of [Sr 0; S H^T S] = [Sy T12; 0 S+], where
        /// Sy^T Sy is the innovation covariance and the gain is T12^T Sy^-T
        /// @return false if the update produced non-finite values, in which case the state is left unchanged
        template <int M>
        bool measure(const Eigen::Matrix<double, M, 1>& z,
                     const Eigen::Matrix<double, M, N>& H,
                     const Eigen::Matrix<double, M, M>& R) {
            Eigen::Matrix<double, M + N, M + N> A;
            A << Eigen::Matrix<double, M, M>(R.llt().matrixU()), Eigen::Matrix<double, M, N>::Zero(), S * H.transpose(),
                S;
            const Eigen::Matrix<double, M + N, M + N> T = upper(A);
            const Eigen::Matrix<double, M, M> Sy        = T.template topLeftCorner<M, M>();
            const Vec x_new                             = x
                              + T.template topRightCorner<M, N>().transpose()
                                    * Sy.transpose().template triangularView<Eigen::Lower>().solve(z - H * x);
            const Mat S_new = T.template bottomRightCorner<N, N>();
            if (!x_new.allFinite() || !S_new.allFinite()) {
                return false;
            }
            x = x_new;
            S = S_new;
            return true;
        }

    private:
        Vec x = Vec::Zero();
        Mat S = Mat::Identity();

        /// @brief The upper triangular factor of a QR decomposition
        template <typename Derived>
        static Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> upper(
            const Eigen::MatrixBase<Derived>& A) {
            return Eigen::HouseholderQR<Derived>(A).matrixQR().template triangularView<Eigen::Upper>();
        }
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_COOPERATIVELOCALISATION_SQUAREROOTEKF_HPP
