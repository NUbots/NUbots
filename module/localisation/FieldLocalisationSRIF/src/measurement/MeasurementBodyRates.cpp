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
#include "MeasurementBodyRates.hpp"

#include <Eigen/Core>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

#include "utility/gaussian_filtering/measurement/Measurement.hpp"

// Both models are linear in the state, so the Newton update converges in one step

namespace module::localisation::measurement {

    using srif::SystemLocalisation;

    // MeasurementGyroscope

    MeasurementGyroscope::MeasurementGyroscope(double time, const Eigen::Vector3d& gyroscope, double sigma)
        : Measurement(time), y_(gyroscope), sigma_(sigma) {
        update_method_ = UpdateMethod::NEWTONTRUSTEIG;
    }

    Eigen::VectorXd MeasurementGyroscope::simulate(const Eigen::VectorXd& x, const SystemEstimator& /*system*/) const {
        return x.segment<3>(SystemLocalisation::i_omega) + x.segment<3>(SystemLocalisation::i_gyro_bias);
    }

    double MeasurementGyroscope::log_likelihood(const Eigen::VectorXd& x, const SystemEstimator& /*system*/) const {
        return log_likelihood_impl<double>(x);
    }

    double MeasurementGyroscope::log_likelihood(const Eigen::VectorXd& x,
                                                const SystemEstimator& /*system*/,
                                                Eigen::VectorXd& g) const {
        using autodiff::at;
        using autodiff::dual;
        using autodiff::gradient;
        using autodiff::wrt;

        Eigen::VectorX<dual> xdual = x.cast<dual>();
        dual fdual;
        auto func = [this](const Eigen::VectorX<dual>& xd) -> dual {
            return this->template log_likelihood_impl<dual>(xd);
        };
        g = gradient(func, wrt(xdual), at(xdual), fdual);
        return static_cast<double>(fdual);
    }

    double MeasurementGyroscope::log_likelihood(const Eigen::VectorXd& x,
                                                const SystemEstimator& /*system*/,
                                                Eigen::VectorXd& g,
                                                Eigen::MatrixXd& H) const {
        using autodiff::at;
        using autodiff::dual2nd;
        using autodiff::hessian;
        using autodiff::wrt;

        g.resize(x.size());
        H.resize(x.size(), x.size());

        Eigen::VectorX<dual2nd> xdual = x.cast<dual2nd>();
        dual2nd fdual;
        auto func = [this](const Eigen::VectorX<dual2nd>& xd) -> dual2nd {
            return this->template log_likelihood_impl<dual2nd>(xd);
        };
        H = hessian(func, wrt(xdual), at(xdual), fdual, g);
        return static_cast<double>(fdual);
    }

    // MeasurementBodyVelocity

    MeasurementBodyVelocity::MeasurementBodyVelocity(double time, const Eigen::Vector3d& velocity, double sigma)
        : Measurement(time), y_(velocity), sigma_(sigma) {
        update_method_ = UpdateMethod::NEWTONTRUSTEIG;
    }

    Eigen::VectorXd MeasurementBodyVelocity::simulate(const Eigen::VectorXd& x,
                                                      const SystemEstimator& /*system*/) const {
        return x.segment<3>(SystemLocalisation::i_vel);
    }

    double MeasurementBodyVelocity::log_likelihood(const Eigen::VectorXd& x, const SystemEstimator& /*system*/) const {
        return log_likelihood_impl<double>(x);
    }

    double MeasurementBodyVelocity::log_likelihood(const Eigen::VectorXd& x,
                                                   const SystemEstimator& /*system*/,
                                                   Eigen::VectorXd& g) const {
        using autodiff::at;
        using autodiff::dual;
        using autodiff::gradient;
        using autodiff::wrt;

        Eigen::VectorX<dual> xdual = x.cast<dual>();
        dual fdual;
        auto func = [this](const Eigen::VectorX<dual>& xd) -> dual {
            return this->template log_likelihood_impl<dual>(xd);
        };
        g = gradient(func, wrt(xdual), at(xdual), fdual);
        return static_cast<double>(fdual);
    }

    double MeasurementBodyVelocity::log_likelihood(const Eigen::VectorXd& x,
                                                   const SystemEstimator& /*system*/,
                                                   Eigen::VectorXd& g,
                                                   Eigen::MatrixXd& H) const {
        using autodiff::at;
        using autodiff::dual2nd;
        using autodiff::hessian;
        using autodiff::wrt;

        g.resize(x.size());
        H.resize(x.size(), x.size());

        Eigen::VectorX<dual2nd> xdual = x.cast<dual2nd>();
        dual2nd fdual;
        auto func = [this](const Eigen::VectorX<dual2nd>& xd) -> dual2nd {
            return this->template log_likelihood_impl<dual2nd>(xd);
        };
        H = hessian(func, wrt(xdual), at(xdual), fdual, g);
        return static_cast<double>(fdual);
    }

}  // namespace module::localisation::measurement
