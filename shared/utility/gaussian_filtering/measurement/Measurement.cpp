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

#include <Eigen/Core>
#include <Eigen/SVD>
#include <stdexcept>

#include "utility/gaussian_filtering/funcmin.hpp"
#include "utility/gaussian_filtering/gaussian/GaussianInfo.hpp"

namespace utility::gaussian_filtering::measurement {

    Measurement::Measurement(double time) : Event(time), update_method_(UpdateMethod::BFGSTRUSTSQRT) {}

    Measurement::~Measurement() = default;

    double Measurement::cost_joint_density(const Eigen::VectorXd& x, const SystemEstimator& system) const {
        double logprior = system.density.log(x);
        double loglik   = log_likelihood(x, system);
        return -(logprior + loglik);
    }

    double Measurement::cost_joint_density(const Eigen::VectorXd& x,
                                           const SystemEstimator& system,
                                           Eigen::VectorXd& g) const {
        Eigen::VectorXd logprior_grad(x.size());
        double logprior = system.density.log(x, logprior_grad);

        Eigen::VectorXd loglik_grad(x.size());
        double loglik = log_likelihood(x, system, loglik_grad);

        g = -(logprior_grad + loglik_grad);
        return -(logprior + loglik);
    }

    double Measurement::cost_joint_density(const Eigen::VectorXd& x,
                                           const SystemEstimator& system,
                                           Eigen::VectorXd& g,
                                           Eigen::MatrixXd& H) const {
        Eigen::VectorXd logprior_grad(x.size());
        Eigen::MatrixXd logprior_hess(x.size(), x.size());
        double logprior = system.density.log(x, logprior_grad, logprior_hess);

        Eigen::VectorXd loglik_grad(x.size());
        Eigen::MatrixXd loglik_hess(x.size(), x.size());
        double loglik = log_likelihood(x, system, loglik_grad, loglik_hess);

        g = -(logprior_grad + loglik_grad);
        H = -(logprior_hess + loglik_hess);
        return -(logprior + loglik);
    }

    void Measurement::update(SystemBase& system_) {
        // Measurement events only occur to SystemEstimator objects
        SystemEstimator& system = dynamic_cast<SystemEstimator&>(system_);

        const Eigen::Index& nx = system.density.dim();

        // Second-order iterated update
        Eigen::VectorXd g(nx);
        Eigen::VectorXd x  = system.density.mean();  // Start from the prior mean
        Eigen::MatrixXd Xi = system.density.sqrt_info_mat();

        switch (update_method_) {
            case UpdateMethod::BFGSTRUSTSQRT: {
                auto cost_func = [&](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
                    return cost_joint_density(x, system, g);
                };

                [[maybe_unused]] int ret = funcmin::bfgs_trust_sqrt(cost_func, x, g, Xi);
                assert(ret == 0);
                break;
            }
            case UpdateMethod::BFGSLMSQRT: {
                auto cost_func = [&](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
                    return cost_joint_density(x, system, g);
                };

                [[maybe_unused]] int ret = funcmin::bfgs_lm_sqrt(cost_func, x, g, Xi);
                assert(ret == 0);
                break;
            }
            case UpdateMethod::SR1TRUSTEIG: {
                // Eigendecomposition of the prior information matrix via an SVD of Xi, which is
                // more precise than decomposing Xi.'*Xi
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(Xi, Eigen::ComputeFullV);
                Eigen::MatrixXd Q = svd.matrixV();
                Eigen::VectorXd v = svd.singularValues().array().square();

                assert(Q.rows() == nx);
                assert(Q.cols() == nx);
                assert(v.size() == nx);

                auto cost_func = [&](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
                    return cost_joint_density(x, system, g);
                };

                [[maybe_unused]] int ret = funcmin::sr1_trust_eig(cost_func, x, g, Q, v);
                assert(ret == 0);

                // Posterior square-root information matrix from the Hessian eigendecomposition
                Xi = v.array().sqrt().matrix().asDiagonal() * Q.transpose();
                Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixXd>> qr(Xi);
                Xi = Xi.triangularView<Eigen::Upper>();
                break;
            }
            case UpdateMethod::NEWTONTRUSTEIG: {
                auto cost_func = [&](const Eigen::VectorXd& x, Eigen::VectorXd& g, Eigen::MatrixXd& H) {
                    return cost_joint_density(x, system, g, H);
                };

                Eigen::MatrixXd Q(nx, nx);
                Eigen::VectorXd v(nx);
                [[maybe_unused]] int ret = funcmin::newton_trust_eig(cost_func, x, g, Q, v);
                assert(ret == 0);

                // Posterior square-root information matrix from the Hessian eigendecomposition
                Xi = v.array().sqrt().matrix().asDiagonal() * Q.transpose();
                Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixXd>> qr(Xi);
                Xi = Xi.triangularView<Eigen::Upper>();
                break;
            }
            default: throw std::invalid_argument("Invalid update method");
        }

        // The posterior mean is the MAP estimate
        Eigen::VectorXd mu = x;

        // Laplace approximation of the log evidence, which needs the prior in system.density:
        // log p(y) ~= -V(x*) + (n/2) log(2 pi) - sum(log|diag(Xi)|)
        const double V          = cost_joint_density(mu, system);
        const double log_det_Xi = Xi.diagonal().array().abs().log().sum();
        log_evidence_           = -V + 0.5 * nx * std::log(2.0 * M_PI) - log_det_Xi;

        system.density = gaussian::GaussianInfo<double>::from_sqrt_info(Xi * mu, Xi);
    }

}  // namespace utility::gaussian_filtering::measurement
