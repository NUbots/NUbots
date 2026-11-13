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
#include "MeasurementKinematicHeight.hpp"

#include <Eigen/Core>
#include <cmath>

#include "utility/gaussian_filtering/measurement/Measurement.hpp"

namespace module::localisation::measurement {
    MeasurementKinematicHeight::MeasurementKinematicHeight(double time, double height, double sigma)
        : Measurement(time), y_(height), sigma_(sigma) {
        // Linear-Gaussian in the state
        update_method_ = UpdateMethod::NEWTONTRUSTEIG;
    }

    Eigen::VectorXd MeasurementKinematicHeight::simulate(const Eigen::VectorXd& x,
                                                         const SystemEstimator& /*system*/) const {
        Eigen::VectorXd y(1);
        y << x(2);
        return y;
    }

    double MeasurementKinematicHeight::log_likelihood(const Eigen::VectorXd& x,
                                                      const SystemEstimator& /*system*/) const {
        const double sigma2 = sigma_ * sigma_;
        const double e      = y_ - x(2);
        return -0.5 * std::log(2.0 * M_PI * sigma2) - 0.5 * e * e / sigma2;
    }

    double MeasurementKinematicHeight::log_likelihood(const Eigen::VectorXd& x,
                                                      const SystemEstimator& system,
                                                      Eigen::VectorXd& g) const {
        const double sigma2 = sigma_ * sigma_;
        g                   = Eigen::VectorXd::Zero(x.size());
        g(2)                = (y_ - x(2)) / sigma2;
        return log_likelihood(x, system);
    }

    double MeasurementKinematicHeight::log_likelihood(const Eigen::VectorXd& x,
                                                      const SystemEstimator& system,
                                                      Eigen::VectorXd& g,
                                                      Eigen::MatrixXd& H) const {
        const double sigma2 = sigma_ * sigma_;
        H                   = Eigen::MatrixXd::Zero(x.size(), x.size());
        H(2, 2)             = -1.0 / sigma2;
        return log_likelihood(x, system, g);
    }

}  // namespace module::localisation::measurement
