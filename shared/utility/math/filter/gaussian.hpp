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

#ifndef UTILITY_MATH_FILTER_GAUSSIAN_HPP
#define UTILITY_MATH_FILTER_GAUSSIAN_HPP

#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
    #define M_PI 3.14159265358979323846264338327950288
#endif
#include <Eigen/Core>
#include <Eigen/QR>
#include <cassert>
#include <functional>

#include "fmin.hpp"


namespace utility::math::filter::gaussian {
    /**
     * @brief conditionGaussianOnMarginal
     */

    void conditionGaussianOnMarginal(const Eigen::VectorXd& muyx,
                                     const Eigen::MatrixXd& Syx,
                                     const Eigen::VectorXd& y,
                                     Eigen::VectorXd& muxGy,
                                     Eigen::MatrixXd& SxGy) {
        int ny              = y.rows();
        int nx              = Syx.cols() - ny;
        Eigen::MatrixXd S1  = Syx.topLeftCorner(ny, ny);
        Eigen::MatrixXd S2  = Syx.topRightCorner(ny, nx);
        Eigen::MatrixXd S3  = Syx.bottomRightCorner(nx, nx);
        Eigen::MatrixXd mux = muyx.tail(nx);
        Eigen::MatrixXd muy = muyx.head(ny);
        muxGy               = mux + S2.transpose() * (S1.triangularView<Eigen::Upper>().transpose().solve(y - muy));
        SxGy                = S3;
    }


    /**
     * @brief gaussianConfidenceEllipse
     */

    void gaussianConfidenceEllipse3Sigma(const Eigen::VectorXd& mu, const Eigen::MatrixXd& S, Eigen::MatrixXd& x) {
        assert(mu.rows() == 2);
        assert(S.rows() == 2);
        assert(S.cols() == 2);

        int nsamples = 100;
        x.resize(2, nsamples);
        Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(nsamples, 0, 2 * M_PI);
        Eigen::MatrixXd Z;
        Z.resize(2, nsamples);
        double r = sqrt(11.8292);
        Eigen::MatrixXd zx;
        zx.resize(nsamples, 1);
        zx = r * t.array().cos();
        Eigen::MatrixXd zy;
        zy.resize(nsamples, 1);
        zy = r * t.array().sin();
        Z << zx.transpose(), zy.transpose();
        x = (S.transpose() * Z).colwise() + mu;
        assert(x.cols() == nsamples);
        assert(x.rows() == 2);
    }


    void gaussianConfidenceQuadric3Sigma(const Eigen::VectorXd& mu, const Eigen::MatrixXd& S, Eigen::MatrixXd& Q) {
        const int nx = 3;
        assert(mu.rows() == nx);
        assert(S.rows() == nx);
        assert(S.cols() == nx);
        Eigen::MatrixXd z = S.triangularView<Eigen::Upper>().transpose().solve(
                                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(S.rows(), S.rows()))
                            * mu;
        Eigen::MatrixXd topLeft =
            S.triangularView<Eigen::Upper>().solve(S.triangularView<Eigen::Upper>().transpose().solve(
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(S.rows(), S.rows())));
        Eigen::MatrixXd topRight =
            -S.triangularView<Eigen::Upper>().solve(
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(S.rows(), S.rows()))
            * z;
        Eigen::MatrixXd bottomLeft =
            -z.transpose()
            * S.triangularView<Eigen::Upper>().transpose().solve(
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(S.rows(), S.rows()));
        Eigen::MatrixXd zTz = z.transpose() * z;
        double bottomRight  = zTz(0) - 14.1564;
        Q.resize(4, 4);
        Q << topLeft, topRight, bottomLeft, bottomRight;
    }
    /**
     * @brief logGaussian
     */
    template <typename Scalar>
    Scalar logGaussian(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& x,
                       const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& mu,
                       const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& S) {
        assert(x.cols() == 1);
        assert(mu.cols() == 1);
        assert(x.size() == mu.size());
        assert(S.rows() == S.cols());
        assert(S.rows() == x.size());

        Scalar n       = x.rows();
        Scalar log_sum = S.diagonal().array().abs().log().sum();
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Z =
            S.template triangularView<Eigen::Upper>().transpose().solve(x - mu);

        return -0.5 * Z.squaredNorm() - n / 2 * std::log(2 * M_PI) - log_sum;
    }

    template <typename Scalar>
    Scalar logGaussian(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& x,
                       const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& mu,
                       const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& S,
                       Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& g) {
        //  Compute gradient of log N(x;mu,P) w.r.t x
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Z =
            S.template triangularView<Eigen::Upper>().transpose().solve(x - mu);
        g = -S.template triangularView<Eigen::Upper>().solve(Z);
        return logGaussian(x, mu, S);
    }

    template <typename Scalar>
    Scalar logGaussian(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& x,
                       const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& mu,
                       const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& S,
                       Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& g,
                       Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& H) {
        //  Compute Hessian of log N(x;mu,P) w.r.t x
        // S\(S.'\I)
        H = -S.template triangularView<Eigen::Upper>().solve(
            S.template triangularView<Eigen::Upper>().transpose().solve(
                Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Identity(S.rows(), S.rows())));
        return logGaussian(x, mu, S, g);
    }


    /**
     * @brief affineTransform
     */
    template <typename Func>
    void affineTransform(const Eigen::VectorXd& mux,  // Input
                         const Eigen::MatrixXd& Sxx,  // Input
                         Func h,                      // Model
                         Eigen::VectorXd& muy,        // Output
                         Eigen::MatrixXd& Syy         // Output
    ) {
        assert(mux.size() > 0);
        assert(mux.cols() == 1);
        assert(Sxx.cols() == Sxx.rows());
        assert(mux.rows() == Sxx.rows());

        //  Transform function
        Eigen::MatrixXd SR;
        Eigen::MatrixXd C;
        h(mux, muy, SR, C);

        //  Check outputs of h
        assert(muy.cols() == 1);
        assert(muy.rows() > 0);

        assert(SR.rows() == muy.size());
        assert(SR.cols() == muy.size());

        assert(C.rows() == muy.size());
        assert(C.cols() == mux.size());

        // QR Decomp
        Eigen::MatrixXd A(Sxx.rows() + SR.rows(), SR.cols());
        A.setZero();
        A << Sxx * C.transpose(), SR;
        Eigen::HouseholderQR<Eigen::MatrixXd> qr(A);
        Eigen::MatrixXd R;
        R.setZero();
        R = qr.matrixQR().triangularView<Eigen::Upper>();
        Syy.resize(SR.rows(), SR.cols());
        Syy = R.block(0, 0, SR.rows(), SR.cols());
    }

    /**
     * @brief Augment Gradients
     */
    template <typename ProcessFunc, typename ParamStruct>
    void augmentGradients(ProcessFunc func,
                          const Eigen::MatrixXd& X,
                          const Eigen::VectorXd& u,
                          ParamStruct& param,
                          Eigen::MatrixXd& dX) {
        assert(X.size() > 0);
        int nx = X.rows();
        assert(X.cols() == (2 * nx + 1));

        Eigen::VectorXd x, f;
        Eigen::MatrixXd SQ, Jx;
        x = X.col(0);

        func(x, u, param, f, SQ, Jx);
        assert(f.rows() == nx);
        assert(SQ.rows() == nx);
        assert(Jx.rows() == nx);

        dX.resize(nx, 2 * nx + 1);
        dX << f, Jx * X.block(0, 1, nx, 2 * nx);
    }


    /**
     * @brief RK4SDEHelper
     */
    template <typename ProcessFunc, typename ParamStruct>
    struct RK4SDEHelper {
        void operator()(ProcessFunc func,
                        const Eigen::VectorXd& xdw,
                        const Eigen::VectorXd& u,
                        ParamStruct& param,
                        double dt,
                        Eigen::VectorXd& xnext) {
            // Check that dimension of augmented state is even
            assert(xdw.size() > 0);
            assert(xdw.size() % 2 == 0);
            int nx = xdw.size() / 2;
            Eigen::VectorXd x(nx), dw(nx), f1, f2, f3, f4;

            x  = xdw.head(nx);
            dw = xdw.tail(nx);

            func(x, u, param, f1);

            // Check that the output works for the first instance
            assert(f1.size() > 0);
            assert(f1.cols() == 1);
            assert(f1.rows() == nx);

            func(x + (f1 * dt + dw) / 2, u, param, f2);
            func(x + (f2 * dt + dw) / 2, u, param, f3);
            func(x + f3 * dt + dw, u, param, f4);

            xnext = x + (f1 + 2 * f2 + 2 * f3 + f4) * dt / 6 + dw;
        }
        void operator()(ProcessFunc func,
                        const Eigen::VectorXd& xdw,
                        const Eigen::VectorXd& u,
                        ParamStruct& param,
                        double dt,
                        Eigen::VectorXd& xnext,
                        Eigen::MatrixXd& SR) {
            assert(xdw.size() > 0);
            assert(xdw.size() % 2 == 0);

            int nx = xdw.size() / 2;
            SR     = Eigen::MatrixXd::Zero(nx, nx);
            operator()(func, xdw, u, param, dt, xnext);
        }
        void operator()(ProcessFunc func,
                        const Eigen::VectorXd& xdw,
                        const Eigen::VectorXd& u,
                        ParamStruct& param,
                        double dt,
                        Eigen::VectorXd& xnext,
                        Eigen::MatrixXd& SR,
                        Eigen::MatrixXd& J) {
            assert(xdw.size() > 0);
            assert(xdw.size() % 2 == 0);
            int nxdx = xdw.size();
            int nx   = nxdx / 2;
            Eigen::VectorXd x(nx), dw(nx);

            x  = xdw.head(nx);
            dw = xdw.tail(nx);

            typedef Eigen::MatrixXd Matrix;

            Matrix X(nx, nxdx + 1), dW(nx, nxdx + 1);
            X << x, Matrix::Identity(nx, nx), Matrix::Zero(nx, nx);
            dW << dw, Matrix::Zero(nx, nx), Matrix::Identity(nx, nx);

            Matrix F1, F2, F3, F4, Xnext;
            augmentGradients(func, X, u, param, F1);
            augmentGradients(func, X + (F1 * dt + dW) / 2, u, param, F2);
            augmentGradients(func, X + (F2 * dt + dW) / 2, u, param, F3);
            augmentGradients(func, X + F3 * dt + dW, u, param, F4);

            Xnext = X + (F1 + 2 * F2 + 2 * F3 + F4) * dt / 6 + dW;
            xnext = Xnext.col(0);
            J     = Xnext.block(0, 1, nx, 2 * nx);
            SR    = Matrix::Zero(nx, nx);
        }
    };

    /**
     * @brief AugmentIdentityAdapter
     */
    template <typename Func, typename ParamStruct>
    struct AugmentIdentityAdapter {
        void operator()(Func h,
                        const Eigen::VectorXd& x,
                        const Eigen::VectorXd& u,
                        const ParamStruct param,
                        Eigen::VectorXd& yx) {
            assert(x.size() > 0);
            Eigen::VectorXd y;
            h(x, u, param, y);
            assert(y.size() > 0);

            int nx  = x.size();
            int ny  = y.size();
            int nyx = ny + nx;

            yx.resize(nyx);

            yx.head(ny) = y;
            yx.tail(nx) = x;
        }

        void operator()(Func h,
                        const Eigen::VectorXd& x,
                        const Eigen::VectorXd& u,
                        const ParamStruct param,
                        Eigen::VectorXd& yx,
                        Eigen::MatrixXd& SRR) {
            assert(x.size() > 0);

            Eigen::VectorXd y;
            Eigen::MatrixXd SR;

            h(x, u, param, y, SR);
            assert(y.size() > 0);
            assert(SR.size() > 0);

            int nx  = x.size();
            int ny  = y.size();
            int nyx = nx + ny;

            SRR.resize(nyx, nyx);
            yx.resize(nyx);

            yx.head(ny) = y;
            yx.tail(nx) = x;

            SRR.fill(0);
            SRR.topLeftCorner(ny, ny) = SR;
        }

        void operator()(Func h,
                        const Eigen::VectorXd& x,
                        const Eigen::VectorXd& u,
                        const ParamStruct param,
                        Eigen::VectorXd& yx,
                        Eigen::MatrixXd& SRR,
                        Eigen::MatrixXd& CI) {
            assert(x.size() > 0);

            Eigen::VectorXd y;
            Eigen::MatrixXd SR;
            Eigen::MatrixXd C;

            h(x, u, param, y, SR, C);
            assert(y.size() > 0);
            assert(SR.size() > 0);
            assert(C.size() > 0);

            int nx  = x.size();
            int ny  = y.size();
            int nyx = nx + ny;

            CI.resize(nyx, nx);
            SRR.resize(nyx, nyx);
            yx.resize(nyx);

            yx.head(ny) = y;
            yx.tail(nx) = x;

            SRR.fill(0);
            SRR.topLeftCorner(ny, ny) = SR;

            CI.fill(0);
            CI.topLeftCorner(ny, nx)    = C;
            CI.bottomLeftCorner(nx, nx) = Eigen::MatrixXd::Identity(nx, nx);
        }
    };


    /**
     * @brief timeUpdateContinuous
     */
    template <typename ProcessFunc, typename ParamStruct>
    void timeUpdateContinuous(const Eigen::VectorXd& mukm1,  // Input
                              const Eigen::MatrixXd& Skm1,   // Input
                              const Eigen::VectorXd& u,      // Input
                              ProcessFunc pm,                // Process model
                              ParamStruct& param,            // Model parameters
                              double timestep,               // Time step
                              Eigen::VectorXd& muk,          // Output
                              Eigen::MatrixXd& Sk            // Output
    ) {

        // Noise mean
        assert(mukm1.size() > 0);
        Eigen::VectorXd muxdw(2 * mukm1.size());
        muxdw.fill(0.);
        muxdw.head(mukm1.size()) = mukm1;

        // Noise covariance
        Eigen::VectorXd f;
        Eigen::MatrixXd SQ;

        pm(mukm1, u, param, f, SQ);

        assert(f.size() > 0);
        assert(SQ.size() > 0);

        Eigen::MatrixXd Sxdw(2 * mukm1.size(), 2 * mukm1.size());
        Sxdw.fill(0.);
        Sxdw.topLeftCorner(mukm1.size(), mukm1.size())     = Skm1;
        Sxdw.bottomRightCorner(mukm1.size(), mukm1.size()) = SQ * std::sqrt(timestep);

        // RK4SDEHelper::operator()(func, xdw, u, param, dt, f, SR, J)
        // https://www.cplusplus.com/reference/functional/bind/
        RK4SDEHelper<ProcessFunc, ParamStruct> func;
        auto h = std::bind(func,
                           pm,
                           std::placeholders::_1,
                           u,
                           param,
                           timestep,
                           std::placeholders::_2,
                           std::placeholders::_3,
                           std::placeholders::_4);

        affineTransform(muxdw, Sxdw, h, muk, Sk);
    }


    /**
     * @brief measurementUpdateEKF
     */
    template <typename Func, typename ParamStruct>
    void measurementUpdateEKF(const Eigen::VectorXd& mux,  // Input
                              const Eigen::MatrixXd& Sxx,  // Input
                              const Eigen::VectorXd& u,    // Input
                              const Eigen::VectorXd& y,    // Input
                              Func measurementModel,       // Model
                              const ParamStruct& param,    // Input
                              Eigen::VectorXd& muxGy,      // Output
                              Eigen::MatrixXd& SxxGy       // Output
    ) {
        assert(mux.size() > 0);
        assert(Sxx.size() > 0);
        assert(y.size() > 0);

        AugmentIdentityAdapter<Func, ParamStruct> aia;

        // Create joint function with the following prototype
        // jointFunc(x, h, SR, H)
        auto jointFunc = std::bind(aia,
                                   measurementModel,
                                   std::placeholders::_1,
                                   u,
                                   param,
                                   std::placeholders::_2,
                                   std::placeholders::_3,
                                   std::placeholders::_4);

        Eigen::VectorXd muyx;
        Eigen::MatrixXd Syx;
        affineTransform(mux, Sxx, jointFunc, muyx, Syx);
        conditionGaussianOnMarginal(muyx, Syx, y, muxGy, SxxGy);
    }

    /**
     * @brief measurementUpdateIEKF
     */
    template <typename LogLikFunc, typename ParamStruct>
    struct CostJointDensity {
        double operator()(LogLikFunc logLikelihood,
                          const Eigen::VectorXd& y,
                          const Eigen::VectorXd& x,
                          const Eigen::VectorXd& u,
                          const ParamStruct& param,
                          const Eigen::VectorXd& mu,
                          const Eigen::MatrixXd& S) {
            double logprior = logGaussian(x, mu, S);
            double loglik   = logLikelihood(y, x, u, param);
            return -(logprior + loglik);
        }

        double operator()(LogLikFunc logLikelihood,
                          const Eigen::VectorXd& y,
                          const Eigen::VectorXd& x,
                          const Eigen::VectorXd& u,
                          const ParamStruct& param,
                          const Eigen::VectorXd& mu,
                          const Eigen::MatrixXd& S,
                          Eigen::VectorXd& g) {
            Eigen::VectorXd logpriorGrad(x.size());
            double logprior = logGaussian(x, mu, S, logpriorGrad);
            Eigen::VectorXd loglikGrad(x.size());
            double loglik = logLikelihood(y, x, u, param, loglikGrad);
            g             = -(logpriorGrad + loglikGrad);
            return -(logprior + loglik);
        }

        double operator()(LogLikFunc logLikelihood,
                          const Eigen::VectorXd& y,
                          const Eigen::VectorXd& x,
                          const Eigen::VectorXd& u,
                          const ParamStruct& param,
                          const Eigen::VectorXd& mu,
                          const Eigen::MatrixXd& S,
                          Eigen::VectorXd& g,
                          Eigen::MatrixXd& H) {
            Eigen::VectorXd logpriorGrad(x.size());
            Eigen::MatrixXd logpriorHess(x.size(), x.size());
            double logprior = logGaussian(x, mu, S, logpriorGrad, logpriorHess);
            Eigen::VectorXd loglikGrad(x.size());
            Eigen::MatrixXd loglikHess(x.size(), x.size());
            double loglik = logLikelihood(y, x, u, param, loglikGrad, loglikHess);
            g             = -(logpriorGrad + loglikGrad);
            H             = -(logpriorHess + loglikHess);
            return -(logprior + loglik);
        }
    };

    template <typename Func, typename ParamStruct>
    void measurementUpdateIEKF(const Eigen::VectorXd& mux,  // Input
                               const Eigen::MatrixXd& Sxx,  // Input
                               const Eigen::VectorXd& u,    // Input
                               const Eigen::VectorXd& y,    // Input
                               Func logLikelihood,          // Model
                               const ParamStruct& param,    // Input
                               Eigen::VectorXd& muxGy,      // Output
                               Eigen::MatrixXd& SxxGy       // Output
    ) {
        assert(mux.size() > 0);

        // Create cost function with prototype
        // V = cost(x, g, H)
        CostJointDensity<Func, ParamStruct> cjd;
        using namespace std::placeholders;
        auto costFunc = std::bind(cjd, logLikelihood, y, _1, u, param, mux, Sxx, _2, _3);

        // Minimise cost
        Eigen::MatrixXd Q(mux.size(), mux.size());
        Eigen::VectorXd v(mux.size());
        Eigen::VectorXd g(mux.size());
        constexpr int verbosity = 1;    // 0:none, 1:dots, 2:summary, 3:iter
        muxGy                   = mux;  // Start optimisation at prior mean

        fminNewtonTrustEig(costFunc, muxGy, g, Q, v, verbosity);
        // std::cout << "Q" << Q << std::endl;
        // std::cout << "Q.rows()" << Q.rows() << std::endl;

        // H = Q*diag(v)*Q.'
        // S.'*S = P = inv(H) = Q*diag(1./v)*Q.' = Q*diag(1./realsqrt(v))*diag(1./realsqrt(v))*Q.'
        // S = triu(qr(diag(1./realsqrt(v))*Q.'))
        if (v.hasNaN()) {
            std::cout << "v has nans" << std::endl;
        }

        if (Q.hasNaN()) {
            std::cout << "Q has nans" << std::endl;
        }

        SxxGy = v.cwiseSqrt().cwiseInverse().asDiagonal() * Q.transpose();
        // std::cout << "SxxGy before = \n" << SxxGy << std::endl;
        Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixXd>> qr(SxxGy);  // decomposition in place
        SxxGy = qr.matrixQR().triangularView<Eigen::Upper>();


        if (SxxGy.hasNaN() || muxGy.hasNaN() || g.hasNaN() || Q.hasNaN() || v.hasNaN() || v.hasNaN()) {
            std::cout << "(╯°□°）╯︵ ┻━┻ " << std::endl;
            std::cout << "NaNs encountered in muxGy. muxGy = \n" << muxGy << std::endl;
            std::cout << "NaNs encountered in SxxGy. SxxGy = \n" << SxxGy << std::endl;
            std::cout << "NaNs encountered in g. g = \n" << g << std::endl;
            std::cout << "NaNs encountered in Q. Q = \n" << Q << std::endl;
            std::cout << "NaNs encountered in v. v = \n" << v << std::endl;
            std::cout << "NaNs encountered in g. g.rows() = \n" << g.rows() << std::endl;
            std::cout << "NaNs encountered in Q. Q.rows() = \n" << Q.rows() << std::endl;
            std::cout << "NaNs encountered in v. v.rows() = \n" << v.rows() << std::endl;
            std::cout << "(╯°□°）╯︵ ┻━┻ " << std::endl;
        }
    }

#include <Eigen/Eigenvalues>

    template <typename Func, typename ParamStruct>
    void measurementUpdateIEKFSR1(const Eigen::VectorXd& mux,  // Input
                                  const Eigen::MatrixXd& Sxx,  // Input
                                  const Eigen::VectorXd& u,    // Input
                                  const Eigen::VectorXd& y,    // Input
                                  Func logLikelihood,          // Model
                                  const ParamStruct& param,    // Input
                                  Eigen::VectorXd& muxGy,      // Output
                                  Eigen::MatrixXd& SxxGy       // Output
    ) {
        assert(mux.size() > 0);

        // Create cost function with prototype
        // V = cost(x, g)
        CostJointDensity<Func, ParamStruct> cjd;
        using namespace std::placeholders;
        auto costFunc = std::bind(cjd, logLikelihood, y, _1, u, param, mux, Sxx, _2);

        // Minimise cost
        Eigen::MatrixXd Q(mux.size(), mux.size());
        Eigen::VectorXd v(mux.size());
        Eigen::VectorXd g(mux.size());
        constexpr int verbosity = 1;    // 0:none, 1:dots, 2:summary, 3:iter
        muxGy                   = mux;  // Start optimisation at prior mean

        // Eigendecomposition of initial Hessian (inverse of prior covariance)
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenH(Sxx.transpose() * Sxx);
        v = eigenH.eigenvalues().cwiseInverse();
        Q = eigenH.eigenvectors();

        fminSR1TrustEig(costFunc, muxGy, g, Q, v, verbosity);

        // Post-calculate posterior square-root covariance from Hessian approximation
        SxxGy = v.cwiseSqrt().cwiseInverse().asDiagonal() * Q.transpose();
        Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixXd>> qr(SxxGy);  // decomposition in place
        SxxGy = qr.matrixQR().triangularView<Eigen::Upper>();
    }
}  // namespace utility::math::filter::gaussian


#endif
