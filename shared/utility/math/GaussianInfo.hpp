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

/**
 * @file GaussianInfo.hpp
 * @brief Multivariate Gaussian in square-root information form.
 *
 * Stores the distribution as (nu, Xi) where Xi is upper-triangular and:
 *   Xi^T * Xi  = P^{-1}   (information matrix)
 *   Xi^T * nu  = P^{-1} * mu  (information vector)
 *   mean       = Xi^{-1} * nu
 *
 * Ported from MCHA4400 course material. Boost-free, Eigen-only.
 */

#ifndef UTILITY_MATH_GAUSSIANINFO_HPP
#define UTILITY_MATH_GAUSSIANINFO_HPP

#include <cassert>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>

namespace utility::math {

    /**
     * @brief Gaussian distribution in square-root information form.
     * @tparam Scalar Numeric type (default: double).
     */
    template <typename Scalar = double>
    class GaussianInfo {
    public:
        // Default constructor: produces an uninitialised (zero-dim) density.
        // Use static factories (fromMoment, fromSqrtInfo, …) for meaningful instances.
        GaussianInfo() = default;

    protected:
        // -------------------------------------------------------------------------
        // Constructors (protected — use static factories)
        // -------------------------------------------------------------------------

        explicit GaussianInfo(std::size_t n) : nu_(n), Xi_(n, n) {
            nu_.setZero();
            Xi_.setZero();
        }

        GaussianInfo(const Eigen::VectorX<Scalar>& nu, const Eigen::MatrixX<Scalar>& Xi) : nu_(nu), Xi_(Xi) {
            assert(nu_.size() == Xi_.cols());
            assert(Xi_.isUpperTriangular());
        }

        template <typename OtherScalar>
        friend class GaussianInfo;

        template <typename OtherScalar>
        explicit GaussianInfo(const GaussianInfo<OtherScalar>& p)
            : nu_(p.nu_.template cast<Scalar>()), Xi_(p.Xi_.template cast<Scalar>()) {
            assert(nu_.size() == Xi_.cols());
            assert(Xi_.isUpperTriangular());
        }

    public:
        // -------------------------------------------------------------------------
        // Type cast
        // -------------------------------------------------------------------------

        template <typename OtherScalar>
        GaussianInfo<OtherScalar> cast() const {
            return GaussianInfo<OtherScalar>(*this);
        }

        // -------------------------------------------------------------------------
        // Two-argument factories
        // -------------------------------------------------------------------------

        /**
         * @brief Construct from mean mu and upper-triangular sqrt-covariance S (S^T*S = P).
         */
        static GaussianInfo fromSqrtMoment(const Eigen::VectorX<Scalar>& mu, const Eigen::MatrixX<Scalar>& S) {
            assert(mu.size() == S.cols());
            assert(S.isUpperTriangular());

            GaussianInfo out(S.cols());
            // Xi = qr(S^{-T})
            out.Xi_ = S.template triangularView<Eigen::Upper>().transpose().solve(
                Eigen::MatrixX<Scalar>::Identity(S.cols(), S.cols()));
            Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(out.Xi_);
            out.Xi_ = out.Xi_.template triangularView<Eigen::Upper>();
            // nu = Xi * mu
            out.nu_ = out.Xi_ * mu;
            return out;
        }

        /**
         * @brief Construct from mean mu and covariance P.
         */
        static GaussianInfo fromMoment(const Eigen::VectorX<Scalar>& mu, const Eigen::MatrixX<Scalar>& P) {
            assert(mu.size() == P.cols());
            assert(P.rows() == P.cols());
            Eigen::LLT<Eigen::MatrixX<Scalar>, Eigen::Upper> llt(P);
            return fromSqrtMoment(mu, llt.matrixU());
        }

        /**
         * @brief Construct from sqrt-info vector nu and upper-triangular sqrt-info matrix Xi.
         */
        static GaussianInfo fromSqrtInfo(const Eigen::VectorX<Scalar>& nu, const Eigen::MatrixX<Scalar>& Xi) {
            assert(nu.size() == Xi.cols());
            assert(Xi.isUpperTriangular());
            GaussianInfo out(Xi.cols());
            out.nu_ = nu;
            out.Xi_ = Xi;
            return out;
        }

        /**
         * @brief Construct from information vector eta and information matrix Lambda.
         */
        static GaussianInfo fromInfo(const Eigen::VectorX<Scalar>& eta, const Eigen::MatrixX<Scalar>& Lambda) {
            assert(eta.size() == Lambda.cols());
            assert(Lambda.rows() == Lambda.cols());
            Eigen::LLT<Eigen::MatrixX<Scalar>, Eigen::Upper> llt(Lambda);
            Eigen::MatrixX<Scalar> Xi = llt.matrixU();
            Eigen::VectorX<Scalar> nu = Xi.template triangularView<Eigen::Upper>().transpose().solve(eta);
            return fromSqrtInfo(nu, Xi);
        }

        // -------------------------------------------------------------------------
        // One-argument factories (zero mean)
        // -------------------------------------------------------------------------

        static GaussianInfo fromSqrtMoment(const Eigen::MatrixX<Scalar>& S) {
            return fromSqrtMoment(Eigen::VectorX<Scalar>::Zero(S.cols()), S);
        }

        static GaussianInfo fromMoment(const Eigen::MatrixX<Scalar>& P) {
            return fromMoment(Eigen::VectorX<Scalar>::Zero(P.cols()), P);
        }

        static GaussianInfo fromSqrtInfo(const Eigen::MatrixX<Scalar>& Xi) {
            return fromSqrtInfo(Eigen::VectorX<Scalar>::Zero(Xi.cols()), Xi);
        }

        static GaussianInfo fromInfo(const Eigen::MatrixX<Scalar>& Lambda) {
            return fromInfo(Eigen::VectorX<Scalar>::Zero(Lambda.cols()), Lambda);
        }

        // -------------------------------------------------------------------------
        // Accessors
        // -------------------------------------------------------------------------

        Eigen::Index dim() const {
            return Xi_.cols();
        }

        /** @brief Mean: solves Xi * mu = nu for mu. */
        Eigen::VectorX<Scalar> mean() const {
            return Xi_.template triangularView<Eigen::Upper>().solve(nu_);
        }

        /** @brief Upper-triangular sqrt of covariance S such that S^T*S = P. */
        Eigen::MatrixX<Scalar> sqrtCov() const {
            Eigen::MatrixX<Scalar> S = Xi_.template triangularView<Eigen::Upper>().transpose().solve(
                Eigen::MatrixX<Scalar>::Identity(Xi_.cols(), Xi_.cols()));
            Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(S);
            S = S.template triangularView<Eigen::Upper>();
            return S;
        }

        /** @brief Covariance matrix P = S^T * S. */
        Eigen::MatrixX<Scalar> cov() const {
            const Eigen::MatrixX<Scalar> S = sqrtCov();
            return S.transpose() * S;
        }

        /** @brief Information matrix Lambda = Xi^T * Xi = P^{-1}. */
        Eigen::MatrixX<Scalar> infoMat() const {
            return Xi_.transpose() * Xi_;
        }

        /** @brief Information vector eta = Xi^T * nu = P^{-1} * mu. */
        Eigen::VectorX<Scalar> infoVec() const {
            return Xi_.transpose() * nu_;
        }

        /** @brief Sqrt-information matrix Xi (upper triangular, stored directly). */
        Eigen::MatrixX<Scalar> sqrtInfoMat() const {
            return Xi_;
        }

        /** @brief Sqrt-information vector nu (stored directly). */
        Eigen::VectorX<Scalar> sqrtInfoVec() const {
            return nu_;
        }

        // -------------------------------------------------------------------------
        // Marginal  p(x(idx))
        // -------------------------------------------------------------------------

        /**
         * @brief Return marginal density p(x(idx)) by QR-eliminating variables in idxNot.
         *
         * Forms [Xi(:,idxNot), Xi(:,idx), nu], Q-less QR gives
         * [R1 R2 nu1; 0 R3 nu2]. Returns N^{-0.5}(x(idx); nu2, R3).
         */
        template <typename IndexType, typename NotIndexType>
        GaussianInfo marginal(const IndexType& idx, const NotIndexType& idxNot) const {
            const std::size_t nI    = idx.size();
            const std::size_t nNotI = idxNot.size();
            const std::size_t n     = nI + nNotI;
            assert(static_cast<Eigen::Index>(n) == dim());

            Eigen::MatrixX<Scalar> A(dim(), static_cast<Eigen::Index>(n + 1));
            for (std::size_t j = 0; j < nNotI; ++j)
                A.col(static_cast<Eigen::Index>(j)) = Xi_.col(static_cast<Eigen::Index>(idxNot[j]));
            for (std::size_t j = 0; j < nI; ++j)
                A.col(static_cast<Eigen::Index>(nNotI + j)) = Xi_.col(static_cast<Eigen::Index>(idx[j]));
            A.col(static_cast<Eigen::Index>(n)) = nu_;

            Eigen::HouseholderQR<Eigen::MatrixX<Scalar>> qr(A);
            Eigen::MatrixX<Scalar> R = qr.matrixQR().template triangularView<Eigen::Upper>();

            GaussianInfo out(nI);
            out.nu_ = R.block(static_cast<Eigen::Index>(nNotI),
                              static_cast<Eigen::Index>(n),
                              static_cast<Eigen::Index>(nI),
                              1);
            out.Xi_ = R.block(static_cast<Eigen::Index>(nNotI),
                              static_cast<Eigen::Index>(nNotI),
                              static_cast<Eigen::Index>(nI),
                              static_cast<Eigen::Index>(nI));
            return out;
        }

        template <typename IndexType>
        GaussianInfo marginal(const IndexType& idx) const {
            const std::size_t n = static_cast<std::size_t>(dim());
            std::vector<bool> isNotInIdx(n, true);
            for (Eigen::Index ii = 0; ii < static_cast<Eigen::Index>(idx.size()); ++ii)
                isNotInIdx[idx[ii]] = false;
            std::vector<int> idxNot;
            idxNot.reserve(n);
            for (std::size_t i = 0; i < n; ++i)
                if (isNotInIdx[i])
                    idxNot.push_back(static_cast<int>(i));
            return marginal(idx, idxNot);
        }

        // -------------------------------------------------------------------------
        // Conditional  p(x(idxA) | x(idxB) = xB)
        // -------------------------------------------------------------------------

        /**
         * @brief Conditional density p(x(idxA) | x(idxB) = xB).
         *
         * Forms [Xi(:,idxA), Xi(:,idxB), nu], QR gives [R1 R2 nu1; 0 R3 nu2].
         * Returns N^{-0.5}(x(idxA); nu1 - R2*xB, R1).
         */
        template <typename IndexTypeA, typename IndexTypeB>
        GaussianInfo conditional(const IndexTypeA& idxA,
                                 const IndexTypeB& idxB,
                                 const Eigen::VectorX<Scalar>& xB) const {
            const std::size_t nA = idxA.size();
            const std::size_t nB = idxB.size();
            const std::size_t n  = nA + nB;
            assert(static_cast<Eigen::Index>(n) == dim());

            Eigen::MatrixX<Scalar> RR(static_cast<Eigen::Index>(n),
                                      static_cast<Eigen::Index>(nA + nB + 1));
            RR.block(0, 0, n, nA) = Xi_(Eigen::all, idxA);
            RR.block(0, nA, n, nB) = Xi_(Eigen::all, idxB);
            RR.col(static_cast<Eigen::Index>(nA + nB)) = nu_;

            Eigen::HouseholderQR<Eigen::MatrixX<Scalar>> qr(RR);
            Eigen::MatrixX<Scalar> R = qr.matrixQR().template triangularView<Eigen::Upper>();

            GaussianInfo out(nA);
            out.Xi_ = R.block(0, 0, nA, nA);
            out.nu_ = R.block(0, nA + nB, nA, 1) - R.block(0, nA, nA, nB) * xB;
            return out;
        }

        /**
         * @brief Conditional density p(x(idxA) | y) given p(x(idxB) | y) via two QR steps.
         */
        template <typename IndexTypeA, typename IndexTypeB>
        GaussianInfo conditional(const IndexTypeA& idxA,
                                 const IndexTypeB& idxB,
                                 const GaussianInfo& pxB_y) const {
            const std::size_t nA = idxA.size();
            const std::size_t nB = idxB.size();
            const std::size_t n  = nA + nB;
            assert(static_cast<Eigen::Index>(n) == dim());

            // First QR: [Xi(:,idxA), Xi(:,idxB), nu]
            Eigen::MatrixX<Scalar> RR(n, n + 1);
            RR << Xi_(Eigen::all, idxA), Xi_(Eigen::all, idxB), nu_;
            Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr1(RR);

            // Second QR: [R2, R1, nu1; pxB_y.Xi, 0, pxB_y.nu]
            Eigen::MatrixX<Scalar> SS(n, n + 1);
            SS.topLeftCorner(nA, nB)     = RR.block(0, nA, nA, nB);
            SS.block(0, nB, nA, nA)      = RR.topLeftCorner(nA, nA).template triangularView<Eigen::Upper>();
            SS.topRightCorner(nA, 1)     = RR.block(0, n, nA, 1);
            SS.bottomLeftCorner(nB, nB)  = pxB_y.Xi_;
            SS.block(nA, nB, nB, nA).setZero();
            SS.bottomRightCorner(nB, 1)  = pxB_y.nu_;
            Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr2(SS);

            GaussianInfo out(nA);
            out.nu_ = SS.block(nB, n, nA, 1);
            out.Xi_ = SS.block(nB, nB, nA, nA).template triangularView<Eigen::Upper>();
            return out;
        }

        // -------------------------------------------------------------------------
        // Affine/nonlinear transform  p(y) where y = h(x)
        // -------------------------------------------------------------------------

        /**
         * @brief Propagate p(x) through y = h(x) via linearisation.
         *
         * h must have signature: VectorX<Scalar> h(const VectorX<Scalar>& x, MatrixX<Scalar>& J)
         * where J = d(h)/d(x) evaluated at x = mean.
         *
         * Uses SVD to handle rank-deficient Jacobians correctly.
         */
        template <typename Func>
        GaussianInfo affineTransform(Func h) const {
            Eigen::MatrixX<Scalar> J;
            Eigen::VectorX<Scalar> mux = mean();
            Eigen::VectorX<Scalar> muy = h(mux, J);

            const Eigen::Index m = J.rows();
            const Eigen::Index n = J.cols();
            assert(m == muy.size());
            assert(n == dim());

            // Linearise: y ≈ J*x + b where b = h(mux) - J*mux
            Eigen::VectorX<Scalar> b = muy - J * mux;

            // SVD of J to handle rank deficiency
            Eigen::JacobiSVD<Eigen::MatrixX<Scalar>> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::VectorX<Scalar> s = svd.singularValues();
            Eigen::MatrixX<Scalar> U = svd.matrixU();
            Eigen::MatrixX<Scalar> V = svd.matrixV();

            // Rank determination
            Scalar tol = std::max(m, n) * Eigen::NumTraits<Scalar>::epsilon() * s.array().abs().maxCoeff();
            Eigen::Index r = (s.array() > tol).count();

            Eigen::VectorX<Scalar> s1 = s.head(r);
            Eigen::MatrixX<Scalar> U1 = U.leftCols(r);
            Eigen::MatrixX<Scalar> U2 = U.rightCols(m - r);
            Eigen::MatrixX<Scalar> V1 = V.leftCols(r);
            Eigen::MatrixX<Scalar> V2 = V.rightCols(n - r);

            // Pseudoinverse: Jp = V1 * S^{-1} * U1^T
            Eigen::MatrixX<Scalar> Jp = V1 * s1.cwiseInverse().asDiagonal() * U1.transpose();

            Eigen::MatrixX<Scalar> X = Xi_ * V2;
            Eigen::MatrixX<Scalar> Y = Xi_ * Jp;
            Scalar kappa             = Scalar(1e7) * std::sqrt(X.array().square().sum() + Y.array().square().sum());

            // Augmented system [(n + m - r) rows, ((n-r) + m + 1) cols]
            Eigen::MatrixX<Scalar> RR(n + m - r, (n - r) + m + 1);
            RR.setZero();
            // Top block
            RR.block(0, 0, n, n - r)           = X;
            if (r > 0)
                RR.block(0, n - r, n, m)        = Y;
            RR.block(0, (n - r) + m, n, 1)     = nu_ + Y * b;
            // Bottom block (only if output is rank-deficient: m - r > 0)
            if (m - r > 0) {
                RR.block(n, n - r, m - r, m)        = kappa * U2.transpose();
                RR.block(n, (n - r) + m, m - r, 1)  = kappa * U2.transpose() * b;
            }

            Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(RR);

            GaussianInfo out(m);
            out.Xi_ = RR.block(n - r, n - r, m, m).template triangularView<Eigen::Upper>();
            out.nu_ = RR.block(n - r, (n - r) + m, m, 1);
            return out;
        }

        // -------------------------------------------------------------------------
        // Log-likelihood
        // -------------------------------------------------------------------------

        /** @brief log p(x) = -n/2 * log(2π) + log|det Xi| - ½ ||Xi*x - nu||² */
        Scalar log(const Eigen::VectorX<Scalar>& x) const {
            assert(x.size() == dim());
            static const Scalar halflog2pi = std::log(Scalar(2) * std::numbers::pi_v<Scalar>) / Scalar(2);
            const Eigen::Index n           = dim();
            const Scalar logdetXi          = Xi_.diagonal().array().abs().log().sum();
            const Eigen::VectorX<Scalar> r = Xi_ * x - nu_;
            return -Scalar(n) * halflog2pi + logdetXi - Scalar(0.5) * r.squaredNorm();
        }

        /** @brief log p(x) and gradient g = -Xi^T (Xi*x - nu). */
        Scalar log(const Eigen::VectorX<Scalar>& x, Eigen::VectorX<Scalar>& g) const {
            g.resize(dim());
            const Eigen::VectorX<Scalar> r = Xi_ * x - nu_;
            g.noalias()                    = -(Xi_.transpose() * r);
            return log(x);
        }

        /** @brief log p(x), gradient, and Hessian H = -Xi^T*Xi. */
        Scalar log(const Eigen::VectorX<Scalar>& x, Eigen::VectorX<Scalar>& g, Eigen::MatrixX<Scalar>& H) const {
            H.resize(dim(), dim());
            H = -(Xi_.transpose() * Xi_);
            return log(x, g);
        }

        // -------------------------------------------------------------------------
        // Joint distribution (block-diagonal, assumes independence)
        // -------------------------------------------------------------------------

        GaussianInfo join(const GaussianInfo& other) const {
            const Eigen::Index n1 = dim();
            const Eigen::Index n2 = other.dim();
            GaussianInfo out(static_cast<std::size_t>(n1 + n2));
            out.nu_.head(n1) = nu_;
            out.nu_.tail(n2) = other.nu_;
            out.Xi_.setZero();
            out.Xi_.topLeftCorner(n1, n1)     = Xi_;
            out.Xi_.bottomRightCorner(n2, n2) = other.Xi_;
            return out;
        }

        GaussianInfo operator*(const GaussianInfo& other) const {
            return join(other);
        }

        GaussianInfo& operator*=(const GaussianInfo& other) {
            const Eigen::Index n1 = dim();
            const Eigen::Index n2 = other.dim();
            nu_.conservativeResize(n1 + n2);
            nu_.tail(n2) = other.nu_;
            Xi_.conservativeResizeLike(Eigen::MatrixX<Scalar>::Zero(n1 + n2, n1 + n2));
            Xi_.bottomRightCorner(n2, n2) = other.Xi_;
            return *this;
        }

    protected:
        Eigen::VectorX<Scalar> nu_;  ///< Square-root information vector
        Eigen::MatrixX<Scalar> Xi_;  ///< Square-root information matrix (upper triangular)
    };

}  // namespace utility::math

#endif  // UTILITY_MATH_GAUSSIANINFO_HPP
