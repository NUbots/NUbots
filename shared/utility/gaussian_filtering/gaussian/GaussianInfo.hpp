/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

#ifndef UTILITY_GAUSSIAN_FILTERING_GAUSSIAN_GAUSSIAN_INFO_HPP
#define UTILITY_GAUSSIAN_FILTERING_GAUSSIAN_GAUSSIAN_INFO_HPP

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <cmath>
#include <cstddef>
#include <numbers>

#include "GaussianBase.hpp"

namespace utility::gaussian_filtering::gaussian {

    /// @brief A Gaussian distribution in square-root information form, N^-0.5(x; nu, Xi)
    template <typename Scalar = double>
    class GaussianInfo : public GaussianBase<Scalar> {
    public:
        virtual ~GaussianInfo() override = default;

    protected:
        GaussianInfo() : GaussianBase<Scalar>() {}

        /// @param n The dimension of the distribution
        explicit GaussianInfo(std::size_t n) : GaussianBase<Scalar>(), nu_(n), Xi_(n, n) {}

        /// @param Xi The square root information matrix
        explicit GaussianInfo(const Eigen::MatrixX<Scalar>& Xi)
            : GaussianBase<Scalar>(), nu_(Eigen::VectorX<Scalar>::Zero(Xi.cols())), Xi_(Xi) {
            assert(nu_.size() == Xi_.cols());
            assert(Xi_.isUpperTriangular());
        }

        /// @param nu The square root information vector
        /// @param Xi The square root information matrix
        GaussianInfo(const Eigen::VectorX<Scalar>& nu, const Eigen::MatrixX<Scalar>& Xi)
            : GaussianBase<Scalar>(), nu_(nu), Xi_(Xi) {
            assert(nu_.size() == Xi_.cols());
            assert(Xi_.isUpperTriangular());
        }

        template <typename OtherScalar>
        friend class GaussianInfo;

        /// @brief Copy from a different scalar type
        template <typename OtherScalar>
        explicit GaussianInfo(const GaussianInfo<OtherScalar>& p)
            : GaussianBase<Scalar>(), nu_(p.nu_.template cast<Scalar>()), Xi_(p.Xi_.template cast<Scalar>()) {
            assert(nu_.size() == Xi_.cols());
            assert(Xi_.isUpperTriangular());
        }

    public:
        /// @brief Cast to a different scalar type
        template <typename OtherScalar>
        GaussianInfo<OtherScalar> cast() const {
            return GaussianInfo<OtherScalar>(*this);
        }

        /// @brief From a mean and the upper triangular square root of the covariance
        static GaussianInfo from_sqrt_moment(const Eigen::VectorX<Scalar>& mu, const Eigen::MatrixX<Scalar>& S) {
            assert(mu.size() == S.cols());
            assert(S.isUpperTriangular());

            GaussianInfo out(S.cols());

            // qr(S^{-T})
            out.Xi_ = S.template triangularView<Eigen::Upper>().transpose().solve(
                Eigen::MatrixX<Scalar>::Identity(S.cols(), S.cols()));
            Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(out.Xi_);
            out.Xi_ = out.Xi_.template triangularView<Eigen::Upper>();

            // Xi*mu = nu
            out.nu_ = out.Xi_ * mu;

            return out;
        }

        /// @brief From a mean and covariance
        static GaussianInfo from_moment(const Eigen::VectorX<Scalar>& mu, const Eigen::MatrixX<Scalar>& P) {
            assert(mu.size() == P.cols());
            assert(P.rows() == P.cols());

            // Let S be an upper-triangular matrix such that S^T*S = P
            Eigen::LLT<Eigen::MatrixX<Scalar>, Eigen::Upper> llt(P);
            Eigen::MatrixX<Scalar> S = llt.matrixU();

            return from_sqrt_moment(mu, S);
        }

        /// @brief From a square root information vector and upper triangular matrix
        static GaussianInfo from_sqrt_info(const Eigen::VectorX<Scalar>& nu, const Eigen::MatrixX<Scalar>& Xi) {
            assert(nu.size() == Xi.cols());
            assert(Xi.isUpperTriangular());

            GaussianInfo out(Xi.cols());
            out.nu_ = nu;
            out.Xi_ = Xi;
            return out;
        }

        /// @brief Zero mean, from an upper triangular square root information matrix
        static GaussianInfo from_sqrt_info(const Eigen::MatrixX<Scalar>& Xi) {
            return from_sqrt_info(Eigen::VectorX<Scalar>::Zero(Xi.cols()), Xi);
        }

        /// @brief The dimension of the distribution
        virtual Eigen::Index dim() const override {
            return Xi_.cols();
        }

        /// @brief The mean vector
        virtual Eigen::VectorX<Scalar> mean() const override {
            // Solve Xi*mu = nu for mu
            return Xi_.template triangularView<Eigen::Upper>().solve(nu_);
        }

        /// @brief The upper triangular square root of the covariance matrix
        virtual Eigen::MatrixX<Scalar> sqrt_cov() const override {
            // S = qr(Xi^{-T})
            Eigen::MatrixX<Scalar> S = Xi_.template triangularView<Eigen::Upper>().transpose().solve(
                Eigen::MatrixX<Scalar>::Identity(Xi_.cols(), Xi_.cols()));
            Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(S);
            S = S.template triangularView<Eigen::Upper>();
            return S;
        }

        /// @brief The covariance matrix
        virtual Eigen::MatrixX<Scalar> cov() const override {
            const Eigen::MatrixX<Scalar>& S = sqrt_cov();
            return S.transpose() * S;
        }

        /// @brief The square root of the information matrix
        virtual Eigen::MatrixX<Scalar> sqrt_info_mat() const override {
            return Xi_;
        }

        /**
         * @brief Given the joint density p(x), return the marginal density p(x(idx)).
         * @param idx The indices of the variables to keep
         * @param idx_not The indices of the variables to marginalise out
         */
        template <typename IndexType, typename NotIndexType>
        GaussianInfo marginal(const IndexType& idx, const NotIndexType& idx_not) const {
            const std::size_t& n_i     = idx.size();
            const std::size_t& n_not_i = idx_not.size();
            const std::size_t n        = n_i + n_not_i;
            assert(n == static_cast<std::size_t>(dim()));

            // Form [Xi(:, idx_not), Xi(:, idx), nu]
            Eigen::MatrixX<Scalar> RR(n, n + 1);
            RR << Xi_(Eigen::all, idx_not), Xi_(Eigen::all, idx), nu_;

            // Q-less QR yields
            // [R1, R2, nu1;
            //   0, R3, nu2]
            Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(RR);

            // p(x(idx)) = N^-0.5(x(idx); nu2, R3)
            GaussianInfo out(n_i);
            out.nu_ = RR.block(n_not_i, n, n_i, 1);
            out.Xi_ = RR.block(n_not_i, n_not_i, n_i, n_i).template triangularView<Eigen::Upper>();
            return out;
        }

        /// @brief The marginal density p(x(idx)), marginalising out every other variable
        template <typename IndexType>
        GaussianInfo marginal(const IndexType& idx) const {
            const std::size_t& n = dim();
            std::vector<bool> is_not_in_idx(n, true);
            for (Eigen::Index ii = 0; ii < idx.size(); ++ii) {
                std::size_t i    = idx[ii];
                is_not_in_idx[i] = false;
            }

            // Complementary indices
            std::vector<int> idx_not;
            idx_not.reserve(n);
            for (std::size_t i = 0; i < n; ++i) {
                if (is_not_in_idx[i]) {
                    idx_not.push_back(i);
                }
            }

            return marginal(idx, idx_not);
        }

        /**
         * @brief Propagate p(x) through y = h(x), linearised at the mean, returning p(y).
         * @param h Function taking x and a Jacobian to fill, returning y
         */
        template <typename Func>
        GaussianInfo affine_transform(Func h) const {
            Eigen::MatrixX<Scalar> J;
            Eigen::VectorX<Scalar> mux = mean();
            Eigen::VectorX<Scalar> muy = h(mux, J);
            const std::size_t m        = J.rows();
            const std::size_t n        = J.cols();
            assert(m == static_cast<std::size_t>(muy.size()));
            assert(n == static_cast<std::size_t>(dim()));

            // Linearise y = h(x) about x = mux: y ~= J*x + h(mux) - J*mux
            Eigen::VectorX<Scalar> b = muy - J * mux;
            Eigen::JacobiSVD<Eigen::MatrixX<Scalar>> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::VectorX<Scalar> s = svd.singularValues();
            Eigen::MatrixX<Scalar> U = svd.matrixU();
            Eigen::MatrixX<Scalar> V = svd.matrixV();

            Scalar tol = std::max(m, n) * std::numeric_limits<Scalar>::epsilon() * s(0);
            // Find the number of singular values greater than tol
            std::size_t r = (s.array() > tol).count();

            Eigen::VectorX<Scalar> s1 = s.head(r);
            Eigen::MatrixX<Scalar> U1 = U.leftCols(r);
            Eigen::MatrixX<Scalar> U2 = U.rightCols(m - r);
            Eigen::MatrixX<Scalar> V1 = V.leftCols(r);
            Eigen::MatrixX<Scalar> V2 = V.rightCols(n - r);
            Eigen::MatrixX<Scalar> Jp = V1 * s1.cwiseInverse().asDiagonal() * U1.transpose();
            Eigen::MatrixX<Scalar> X  = Xi_ * V2;
            Eigen::MatrixX<Scalar> Y  = Xi_ * Jp;

            Scalar sigma_max_ub = std::sqrt(X.array().square().sum() + Y.array().square().sum());
            Scalar kappa        = Scalar(1e7) * sigma_max_ub;

            std::size_t total_rows    = n + (m - r);
            std::size_t total_cols    = (n - r) + m + 1;
            Eigen::MatrixX<Scalar> RR = Eigen::MatrixX<Scalar>::Zero(total_rows, total_cols);

            // Top block: [X, Y, nu + Y*b]
            RR.block(0, 0, n, n - r)     = X;
            RR.block(0, n - r, n, m)     = Y;
            RR.block(0, n - r + m, n, 1) = nu_ + Y * b;

            // Bottom block: [zeros(m - r, n - r), kappa*U2.', kappa*U2.'*b]
            if (m > r) {
                RR.block(n, n - r, m - r, m)     = kappa * U2.transpose();
                RR.block(n, n - r + m, m - r, 1) = kappa * U2.transpose() * b;
            }

            // Q-less QR decomposition
            Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(RR);

            GaussianInfo out(m);
            out.Xi_ = RR.block(n - r, n - r, m, m).template triangularView<Eigen::Upper>();
            out.nu_ = RR.block(n - r, n - r + m, m, 1);

            return out;
        }

        /// @brief The log density at x
        virtual Scalar log(const Eigen::VectorX<Scalar>& x) const override {
            assert(x.cols() == 1);
            assert(x.size() == dim());
            using std::abs;
            using std::log;
            const Eigen::Index n              = x.size();
            const Eigen::VectorX<Scalar> diff = Xi_ * x - nu_;
            const Scalar logdet               = (Xi_.diagonal().array().abs().log()).sum();
            const Scalar halflog2pi           = log(2 * std::numbers::pi) / 2.0;
            return -0.5 * diff.squaredNorm() - n * halflog2pi + logdet;
        }

        /// @brief The log density at x and its gradient g
        Scalar log(const Eigen::VectorX<Scalar>& x, Eigen::VectorX<Scalar>& g) const {
            assert(x.size() == dim());
            g.resize(x.size());
            const Eigen::VectorX<Scalar> diff = Xi_ * x - nu_;
            g                                 = -Xi_.transpose() * diff;
            return log(x);
        }

        /// @brief The log density at x, its gradient g and its Hessian H
        Scalar log(const Eigen::VectorX<Scalar>& x, Eigen::VectorX<Scalar>& g, Eigen::MatrixX<Scalar>& H) const {
            assert(x.size() == dim());
            g.resize(x.size());
            H.resize(x.size(), x.size());
            H = -Xi_.transpose() * Xi_;
            return log(x, g);
        }

        /// @brief The joint density of this and another, independent, distribution
        GaussianInfo join(const GaussianInfo& other) const {
            const Eigen::Index& n1 = dim();
            const Eigen::Index& n2 = other.dim();
            GaussianInfo out(n1 + n2);
            out.nu_ << nu_, other.nu_;
            out.Xi_ << Xi_, Eigen::MatrixX<Scalar>::Zero(n1, n2), Eigen::MatrixX<Scalar>::Zero(n2, n1), other.Xi_;
            return out;
        }

        /// @brief The joint density of this and another, independent, distribution
        GaussianInfo operator*(const GaussianInfo& other) const {
            return join(other);
        }

        /// @brief Replace this with its joint density with another, independent, distribution
        GaussianInfo& operator*=(const GaussianInfo& other) {
            const Eigen::Index& n1 = dim();
            const Eigen::Index& n2 = other.dim();
            nu_.conservativeResize(n1 + n2);
            nu_.tail(n2) = other.nu_;
            Xi_.conservativeResizeLike(Eigen::MatrixX<Scalar>::Zero(n1 + n2, n1 + n2));
            Xi_.bottomRightCorner(n2, n2) = other.Xi_;
            return *this;
        }

    protected:
        Eigen::VectorX<Scalar> nu_;  ///< The square root information vector
        Eigen::MatrixX<Scalar> Xi_;  ///< The square root information matrix
    };

}  // namespace utility::gaussian_filtering::gaussian

#endif  // UTILITY_GAUSSIAN_FILTERING_GAUSSIAN_GAUSSIAN_INFO_HPP
