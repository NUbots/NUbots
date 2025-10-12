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

#include "Measurement.hpp"

#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/SVD>

#include "../gaussian/GaussianInfo.hpp"
#include "../funcmin.hpp"

namespace utility::slam::measurement {

    Measurement::Measurement(double time) : Event(time), updateMethod_(UpdateMethod::BFGSTRUSTSQRT) {}

    Measurement::Measurement(double time, int verbosity)
        : Event(time, verbosity), updateMethod_(UpdateMethod::BFGSTRUSTSQRT) {}

    Measurement::~Measurement() = default;

    double Measurement::costJointDensity(const Eigen::VectorXd& x, const SystemEstimator& system) const {
        double logprior = system.density.log(x);
        double loglik   = logLikelihood(x, system);
        return -(logprior + loglik);
    }

    double Measurement::costJointDensity(const Eigen::VectorXd& x,
                                        const SystemEstimator& system,
                                        Eigen::VectorXd& g) const {
        Eigen::VectorXd logpriorGrad(x.size());
        double logprior = system.density.log(x, logpriorGrad);

        Eigen::VectorXd loglikGrad(x.size());
        double loglik = logLikelihood(x, system, loglikGrad);

        g = -(logpriorGrad + loglikGrad);
        return -(logprior + loglik);
    }

    double Measurement::costJointDensity(const Eigen::VectorXd& x,
                                        const SystemEstimator& system,
                                        Eigen::VectorXd& g,
                                        Eigen::MatrixXd& H) const {
        Eigen::VectorXd logpriorGrad(x.size());
        Eigen::MatrixXd logpriorHess(x.size(), x.size());
        double logprior = system.density.log(x, logpriorGrad, logpriorHess);

        Eigen::VectorXd loglikGrad(x.size());
        Eigen::MatrixXd loglikHess(x.size(), x.size());
        double loglik = logLikelihood(x, system, loglikGrad, loglikHess);

        g = -(logpriorGrad + loglikGrad);
        H = -(logpriorHess + loglikHess);
        return -(logprior + loglik);
    }

    void Measurement::update(SystemBase& system_) {
        // Downcast since we know that Measurement events only occur to SystemEstimator objects
        SystemEstimator& system = dynamic_cast<SystemEstimator&>(system_);

        const Eigen::Index& nx = system.density.dim();

        // Second-order iterated update
        Eigen::VectorXd g(nx);
        Eigen::VectorXd x  = system.density.mean();  // Set initial decision variable to prior mean
        Eigen::MatrixXd Xi = system.density.sqrtInfoMat();

        switch (updateMethod_) {
            case UpdateMethod::BFGSTRUSTSQRT: {
                // Create cost function with prototype V = costFunc(x, g)
                auto costFunc = [&](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
                    return costJointDensity(x, system, g);
                };

                // Minimise cost
                int ret = funcmin::BFGSTrustSqrt(costFunc, x, g, Xi, verbosity_);
                assert(ret == 0);
                break;
            }
            case UpdateMethod::BFGSLMSQRT: {
                // Create cost function with prototype V = costFunc(x, g)
                auto costFunc = [&](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
                    return costJointDensity(x, system, g);
                };

                // Minimise cost
                int ret = funcmin::BFGSLMSqrt(costFunc, x, g, Xi, verbosity_);
                assert(ret == 0);
                break;
            }
            case UpdateMethod::SR1TRUSTEIG: {
                // Generate eigendecomposition of initial Hessian (prior information matrix)
                // via an SVD of Xi = U*D*V.', i.e., Xi.'*Xi = V*D*U.'*U*D*V.' = V*D^2*V.'
                // This avoids the loss of precision associated with directly computing the eigendecomposition of
                // Xi.'*Xi
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(Xi, Eigen::ComputeFullV);
                Eigen::MatrixXd Q = svd.matrixV();
                Eigen::VectorXd v = svd.singularValues().array().square();

                assert(Q.rows() == nx);
                assert(Q.cols() == nx);
                assert(v.size() == nx);

                // Foreshadowing: If we were doing landmark SLAM with a quasi-Newton method,
                //                we can purposely introduce negative eigenvalues for newly
                //                initialised landmarks to force the Hessian and hence
                //                posterior sqrt information matrix to be approximated correctly.

                // Create cost function with prototype V = costFunc(x, g)
                auto costFunc = [&](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
                    return costJointDensity(x, system, g);
                };

                // Minimise cost
                int ret = funcmin::SR1TrustEig(costFunc, x, g, Q, v, verbosity_);
                assert(ret == 0);

                // Post-calculate posterior square-root information matrix from Hessian eigendecomposition
                Xi = v.array().sqrt().matrix().asDiagonal() * Q.transpose();
                Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixXd>> qr(Xi);  // In-place QR decomposition
                Xi = Xi.triangularView<Eigen::Upper>();                    // Safe aliasing
                break;
            }
            case UpdateMethod::NEWTONTRUSTEIG: {
                // Create cost function with prototype V = costFunc(x, g, H)
                auto costFunc = [&](const Eigen::VectorXd& x, Eigen::VectorXd& g, Eigen::MatrixXd& H) {
                    return costJointDensity(x, system, g, H);
                };

                // Minimise cost
                Eigen::MatrixXd Q(nx, nx);
                Eigen::VectorXd v(nx);
                int ret = funcmin::NewtonTrustEig(costFunc, x, g, Q, v, verbosity_);
                assert(ret == 0);

                // Post-calculate posterior square-root information matrix from Hessian eigendecomposition
                Xi = v.array().sqrt().matrix().asDiagonal() * Q.transpose();
                Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixXd>> qr(Xi);  // In-place QR decomposition
                Xi = Xi.triangularView<Eigen::Upper>();                    // Safe aliasing
                break;
            }
            default: throw std::invalid_argument("Invalid update method");
        }

        // Set posterior mean to maximum a posteriori (MAP) estimate
        Eigen::VectorXd mu = x;
        system.density     = gaussian::GaussianInfo<double>::fromSqrtInfo(Xi * mu, Xi);
    }

}  // namespace utility::slam::measurement
