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

#ifndef UTILITY_SLAM_GAUSSIAN_GAUSSIAN_INFO_HPP
#define UTILITY_SLAM_GAUSSIAN_GAUSSIAN_INFO_HPP

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>  // TODO: Remove this header after fixing GaussianInfo::affineTransform to pass all unit tests
#include <Eigen/QR>
#include <Eigen/SVD>
#include <cmath>
#include <cstddef>
#include <numbers>

#include "GaussianBase.hpp"

namespace utility::gaussian_filtering::gaussian {

    /**
     * @brief Represents a Gaussian distribution using the information form.
     *
     * @tparam Scalar The scalar type used for calculations (default: double).
     */
    template <typename Scalar = double>
    class GaussianInfo : public GaussianBase<Scalar> {
    public:
        virtual ~GaussianInfo() override = default;

    protected:
        /**
         * @brief Default constructor.
         */
        GaussianInfo() : GaussianBase<Scalar>() {}

        /**
         * @brief Constructor with dimension.
         * @param n The dimension of the Gaussian distribution.
         */
        explicit GaussianInfo(std::size_t n) : GaussianBase<Scalar>(), nu_(n), Xi_(n, n) {}

        /**
         * @brief Constructor with square root information matrix.
         * @param Xi The square root information matrix.
         */
        explicit GaussianInfo(const Eigen::MatrixX<Scalar>& Xi)
            : GaussianBase<Scalar>(), nu_(Eigen::VectorX<Scalar>::Zero(Xi.cols())), Xi_(Xi) {
            assert(nu_.size() == Xi_.cols());
            assert(Xi_.isUpperTriangular());
        }

        /**
         * @brief Constructor with square root information vector and matrix.
         * @param nu The square root information vector.
         * @param Xi The square root information matrix.
         */
        GaussianInfo(const Eigen::VectorX<Scalar>& nu, const Eigen::MatrixX<Scalar>& Xi)
            : GaussianBase<Scalar>(), nu_(nu), Xi_(Xi) {
            assert(nu_.size() == Xi_.cols());
            assert(Xi_.isUpperTriangular());
        }

        /**
         * @brief Friend declaration to allow access to protected members for type conversion.
         */
        template <typename OtherScalar>
        friend class GaussianInfo;

        /**
         * @brief Copy constructor from a different scalar type.
         * @tparam OtherScalar The scalar type of the source GaussianInfo.
         * @param p The source GaussianInfo to copy from.
         */
        template <typename OtherScalar>
        explicit GaussianInfo(const GaussianInfo<OtherScalar>& p)
            : GaussianBase<Scalar>(), nu_(p.nu_.template cast<Scalar>()), Xi_(p.Xi_.template cast<Scalar>()) {
            assert(nu_.size() == Xi_.cols());
            assert(Xi_.isUpperTriangular());
        }

    public:
        /**
         * @brief Casts the GaussianInfo to a different scalar type.
         *
         * @tparam OtherScalar The target scalar type.
         * @return The casted GaussianInfo<OtherScalar> object.
         */
        template <typename OtherScalar>
        GaussianInfo<OtherScalar> cast() const {
            return GaussianInfo<OtherScalar>(*this);
        }

        //
        // Two-argument factories
        //

        /**
         * @brief Creates a GaussianInfo object from square root moment parameters.
         *
         * @param mu The mean vector.
         * @param S The square root of the covariance matrix (upper triangular).
         * @return The resulting GaussianInfo object.
         */
        static GaussianInfo fromSqrtMoment(const Eigen::VectorX<Scalar>& mu, const Eigen::MatrixX<Scalar>& S) {
            assert(mu.size() == S.cols());
            assert(S.isUpperTriangular());

            GaussianInfo out(S.cols());

            // qr(S^{-T})
            out.Xi_ = S.template triangularView<Eigen::Upper>().transpose().solve(
                Eigen::MatrixX<Scalar>::Identity(S.cols(), S.cols()));
            Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(out.Xi_);  // In-place QR decomposition
            out.Xi_ = out.Xi_.template triangularView<Eigen::Upper>();             // Safe aliasing

            // Xi*mu = nu
            out.nu_ = out.Xi_ * mu;

            return out;
        }

        /**
         * @brief Creates a GaussianInfo object from moment parameters.
         *
         * @param mu The mean vector.
         * @param P The covariance matrix.
         * @return The resulting GaussianInfo object.
         */
        static GaussianInfo fromMoment(const Eigen::VectorX<Scalar>& mu, const Eigen::MatrixX<Scalar>& P) {
            assert(mu.size() == P.cols());
            assert(P.rows() == P.cols());

            // Let S be an upper-triangular matrix such that S^T*S = P
            Eigen::LLT<Eigen::MatrixX<Scalar>, Eigen::Upper> llt(P);
            Eigen::MatrixX<Scalar> S = llt.matrixU();

            return fromSqrtMoment(mu, S);
        }

        /**
         * @brief Creates a GaussianInfo object from square root information parameters.
         *
         * @param nu The square root information vector.
         * @param Xi The square root information matrix (upper triangular).
         * @return The resulting GaussianInfo object.
         */
        static GaussianInfo fromSqrtInfo(const Eigen::VectorX<Scalar>& nu, const Eigen::MatrixX<Scalar>& Xi) {
            assert(nu.size() == Xi.cols());
            assert(Xi.isUpperTriangular());

            GaussianInfo out(Xi.cols());
            out.nu_ = nu;
            out.Xi_ = Xi;
            return out;
        }

        //
        // One-argument factories
        //

        /**
         * @brief Creates a GaussianInfo object from the square root information matrix.
         *
         * This static factory method creates a GaussianInfo object with zero mean and
         * the given square root information matrix.
         *
         * @param Xi The square root information matrix (upper triangular).
         * @return The resulting GaussianInfo object.
         */
        static GaussianInfo fromSqrtInfo(const Eigen::MatrixX<Scalar>& Xi) {
            return fromSqrtInfo(Eigen::VectorX<Scalar>::Zero(Xi.cols()), Xi);
        }

        /**
         * @brief Get the dimension of the Gaussian distribution.
         *
         * @return The dimension of the distribution.
         */
        virtual Eigen::Index dim() const override {
            return Xi_.cols();
        }

        /**
         * @brief Get the mean of the Gaussian distribution.
         *
         * @return The mean vector of the distribution.
         */
        virtual Eigen::VectorX<Scalar> mean() const override {
            // Solve Xi*mu = nu for mu
            return Xi_.template triangularView<Eigen::Upper>().solve(nu_);
        }

        /**
         * @brief Get the square root of the covariance matrix.
         *
         * This method computes and returns the upper triangular square root of the covariance matrix,
         * also known as the Cholesky factor.
         *
         * @return The upper triangular square root of the covariance matrix.
         */
        virtual Eigen::MatrixX<Scalar> sqrtCov() const override {
            // S = qr(Xi^{-T})
            Eigen::MatrixX<Scalar> S = Xi_.template triangularView<Eigen::Upper>().transpose().solve(
                Eigen::MatrixX<Scalar>::Identity(Xi_.cols(), Xi_.cols()));
            Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(S);  // In-place QR decomposition
            S = S.template triangularView<Eigen::Upper>();                   // Safe aliasing
            return S;
        }

        /**
         * @brief Get the covariance matrix of the Gaussian distribution.
         *
         * This method computes and returns the covariance matrix by multiplying
         * the square root of the covariance matrix with its transpose.
         *
         * @return The covariance matrix of the distribution.
         */
        virtual Eigen::MatrixX<Scalar> cov() const override {
            const Eigen::MatrixX<Scalar>& S = sqrtCov();
            return S.transpose() * S;
        }

        /**
         * @brief Get the square root of the information matrix.
         *
         * This method returns the square root of the information matrix,
         * which is stored internally as Xi_.
         *
         * @return The square root of the information matrix.
         */
        virtual Eigen::MatrixX<Scalar> sqrtInfoMat() const override {
            return Xi_;
        }

        /**
         * @brief Given joint density p(x), return marginal density p(x(idx))
         *
         * This method computes the marginal density for a subset of variables specified by idx.
         *
         * @tparam IndexType The type of the index container
         * @tparam NotIndexType The type of the complementary index container
         * @param idx The indices of the variables to keep in the marginal
         * @param idxNot The indices of the variables to marginalize out
         * @return The marginal Gaussian distribution
         */
        template <typename IndexType, typename NotIndexType>
        GaussianInfo marginal(const IndexType& idx, const NotIndexType& idxNot) const {
            const std::size_t& nI    = idx.size();
            const std::size_t& nNotI = idxNot.size();
            const std::size_t n      = nI + nNotI;
            assert(n == static_cast<std::size_t>(dim()));

            // Form [Xi(:, idxNot), Xi(:, idx), nu]
            Eigen::MatrixX<Scalar> RR(n, n + 1);
            RR << Xi_(Eigen::all, idxNot), Xi_(Eigen::all, idx), nu_;

            // Q-less QR yields
            // [R1, R2, nu1;
            //   0, R3, nu2]
            Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(RR);  // In-place QR decomposition

            // p(x(idx)) = N^-0.5(x(idx); nu2, R3)
            GaussianInfo out(nI);
            out.nu_ = RR.block(nNotI, n, nI, 1);
            out.Xi_ = RR.block(nNotI, nNotI, nI, nI).template triangularView<Eigen::Upper>();
            return out;
        }

        /**
         * @brief Compute the marginal density for a subset of variables.
         *
         * This method computes the marginal density for a subset of variables specified by idx.
         * It automatically computes the complementary indices.
         *
         * @tparam IndexType The type of the index container
         * @param idx The indices of the variables to keep in the marginal
         * @return The marginal Gaussian distribution
         */
        template <typename IndexType>
        GaussianInfo marginal(const IndexType& idx) const {
            const std::size_t& n = dim();
            std::vector<bool> isNotInIdx(n, true);
            for (Eigen::Index ii = 0; ii < idx.size(); ++ii) {
                std::size_t i = idx[ii];
                isNotInIdx[i] = false;
            }

            // Complementary indices
            std::vector<int> idxNot;
            idxNot.reserve(n);  // Reserve maximum possible size to avoid reallocation
            for (std::size_t i = 0; i < n; ++i) {
                if (isNotInIdx[i]) {
                    idxNot.push_back(i);
                }
            }

            return marginal(idx, idxNot);
        }

        /**
         * @brief Propagate the Gaussian distribution through a nonlinear function.
         *
         * This method transforms the current Gaussian distribution p(x) through a given function y = h(x)
         * by propagating information through the affine transformation. It returns a new Gaussian distribution
         * representing p(y).
         *
         * @tparam Func The type of the function object.
         * @param h The function object representing the nonlinear transformation.
         *          It should take two arguments: the input vector and a reference to the Jacobian matrix.
         *          The function should return the transformed vector and populate the Jacobian matrix.
         * @return A new Gaussian distribution representing p(y).
         */
        template <typename Func>
        GaussianInfo affineTransform(Func h) const {
            Eigen::MatrixX<Scalar> J;
            Eigen::VectorX<Scalar> mux = mean();
            Eigen::VectorX<Scalar> muy = h(mux, J);  // Evaluate function at mean value
            const std::size_t m        = J.rows();
            const std::size_t n        = J.cols();
            assert(m == static_cast<std::size_t>(muy.size()));
            assert(n == static_cast<std::size_t>(dim()));

            // Linearise y = h(x) about x = mux
            // y ~= h(mux) + J*(x - mux)
            //    = J*x + h(mux) - J*mux
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

        /**
         * @brief Compute the log-likelihood of a given vector.
         *
         * This method calculates the log-likelihood of the vector x under the current
         * Gaussian distribution represented in information form.
         *
         * @param x The input vector for which to compute the log-likelihood.
         * @return The log-likelihood value.
         */
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

        /**
         * @brief Compute the log-likelihood of a given vector and its gradient.
         *
         * This method calculates the log-likelihood of the vector x under the current
         * Gaussian distribution represented in information form. It also computes
         * the gradient of the log-likelihood with respect to x.
         *
         * @param x The input vector for which to compute the log-likelihood.
         * @param g Reference to a vector where the gradient will be stored.
         * @return The log-likelihood value.
         */
        Scalar log(const Eigen::VectorX<Scalar>& x, Eigen::VectorX<Scalar>& g) const {
            // Compute gradient g
            assert(x.size() == dim());  // ensure x matches the same dimension as the Gaussian distribution
            g.resize(x.size());         // resize g to match the dimension of x
            const Eigen::VectorX<Scalar> diff = Xi_ * x - nu_;
            g                                 = -Xi_.transpose() * diff;
            return log(x);
        }

        /**
         * @brief Compute the log-likelihood of a given vector, its gradient, and its Hessian.
         *
         * This method calculates the log-likelihood of the vector x under the current
         * Gaussian distribution represented in information form. It also computes
         * the gradient and Hessian of the log-likelihood with respect to x.
         *
         * @param x The input vector for which to compute the log-likelihood.
         * @param g Reference to a vector where the gradient will be stored.
         * @param H Reference to a matrix where the Hessian will be stored.
         * @return The log-likelihood value.
         */
        Scalar log(const Eigen::VectorX<Scalar>& x, Eigen::VectorX<Scalar>& g, Eigen::MatrixX<Scalar>& H) const {
            // Compute Hessian H
            assert(x.size() == dim());     // ensure x matches the same dimension as the Gaussian distribution
            g.resize(x.size());            // resize g to match the dimension of x
            H.resize(x.size(), x.size());  // resize H to match the dimension of x
            H = -Xi_.transpose() * Xi_;
            return log(x, g);
        }

        /**
         * @brief Join two Gaussian distributions into a joint distribution.
         *
         * This method combines the current Gaussian distribution with another one,
         * creating a joint distribution. The resulting distribution represents
         * the joint probability of both input distributions, assuming they are independent.
         *
         * @param other The other GaussianInfo object to join with.
         * @return A new GaussianInfo object representing the joint distribution.
         */
        GaussianInfo join(const GaussianInfo& other) const {
            const Eigen::Index& n1 = dim();
            const Eigen::Index& n2 = other.dim();
            GaussianInfo out(n1 + n2);
            out.nu_ << nu_, other.nu_;
            out.Xi_ << Xi_, Eigen::MatrixX<Scalar>::Zero(n1, n2), Eigen::MatrixX<Scalar>::Zero(n2, n1), other.Xi_;
            return out;
        }

        /**
         * @brief Multiply two Gaussian distributions to create a joint distribution.
         *
         * This operator overload combines the current Gaussian distribution with another one,
         * creating a joint distribution. The resulting distribution represents
         * the joint probability of both input distributions, assuming they are independent.
         *
         * @param other The other GaussianInfo object to multiply with.
         * @return A new GaussianInfo object representing the joint distribution.
         */
        GaussianInfo operator*(const GaussianInfo& other) const {
            return join(other);
        }

        /**
         * @brief Multiply and assign another Gaussian distribution to the current one.
         *
         * This operator multiplies the current Gaussian distribution with another one,
         * creating a joint distribution. The resulting distribution represents
         * the joint probability of both input distributions, assuming they are independent.
         * The result is stored in the current object.
         *
         * @param other The other GaussianInfo object to multiply with.
         * @return A reference to the current object after multiplication.
         */
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
        Eigen::VectorX<Scalar> nu_;  ///< The square root information vector.
        Eigen::MatrixX<Scalar> Xi_;  ///< The square root information matrix.
    };

}  // namespace utility::gaussian_filtering::gaussian

#endif  // UTILITY_SLAM_GAUSSIAN_GAUSSIAN_INFO_HPP
