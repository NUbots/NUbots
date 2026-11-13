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
#include "MeasurementGravity.hpp"

#include <Eigen/Core>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

#include "utility/gaussian_filtering/measurement/Measurement.hpp"

namespace module::localisation::measurement {

    MeasurementGravity::MeasurementGravity(double time, const Eigen::Vector3d& accelerometer, double sigma)
        : Measurement(time), y_(accelerometer), sigma_(sigma) {
        update_method_ = UpdateMethod::NEWTONTRUSTEIG;
    }

    Eigen::VectorXd MeasurementGravity::simulate(const Eigen::VectorXd& x, const SystemEstimator& /*system*/) const {
        const Eigen::Matrix3d Rfb = quat2rot(Eigen::Vector4d(x.segment<4>(srif::SystemLocalisation::i_quat)));
        return Rfb.transpose() * Eigen::Vector3d(0, 0, gravity_);
    }

    double MeasurementGravity::log_likelihood(const Eigen::VectorXd& x, const SystemEstimator& /*system*/) const {
        return log_likelihood_impl<double>(x);
    }

    double MeasurementGravity::log_likelihood(const Eigen::VectorXd& x,
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

    double MeasurementGravity::log_likelihood(const Eigen::VectorXd& x,
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
