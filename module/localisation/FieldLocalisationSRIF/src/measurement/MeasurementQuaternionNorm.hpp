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
/**
 * @file MeasurementQuaternionNorm.hpp
 * @brief Unit-norm pseudo-measurement pinning the redundant quaternion DOF.
 */
#ifndef MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTQUATERNIONNORM_HPP
#define MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTQUATERNIONNORM_HPP

#include <Eigen/Core>
#include <cmath>

#include "srif/SystemLocalisation.hpp"

#include "utility/gaussian_filtering/measurement/Measurement.hpp"
#include "utility/gaussian_filtering/system/SystemEstimator.hpp"

namespace module::localisation::measurement {

    using srif::SystemLocalisation;
    using utility::gaussian_filtering::quat2rot;
    using utility::gaussian_filtering::measurement::Measurement;
    using utility::gaussian_filtering::system::SystemEstimator;

    /**
     * @class MeasurementQuaternionNorm
     * @brief Pseudo-measurement asserting |q| = 1.
     *
     * quat2rot normalises, so no other measurement constrains |q| and the MAP Hessian
     * would be singular along it. This is a soft constraint, and the mean is renormalised
     * in SystemLocalisation::normalise_quaternion.
     */
    class MeasurementQuaternionNorm : public Measurement {
    public:
        /**
         * @brief Construct the unit-norm pseudo-measurement.
         * @param time Event time [s]
         * @param sigma Std dev on |q| (dimensionless)
         */
        explicit MeasurementQuaternionNorm(double time, double sigma = 1e-3) : Measurement(time), sigma_(sigma) {
            update_method_ = UpdateMethod::NEWTONTRUSTEIG;
        }

        virtual Eigen::VectorXd simulate(const Eigen::VectorXd& x, const SystemEstimator& /*system*/) const override {
            return Eigen::VectorXd::Constant(1, x.segment<4>(srif::SystemLocalisation::i_quat).norm());
        }

        virtual double log_likelihood(const Eigen::VectorXd& x, const SystemEstimator& /*system*/) const override {
            return log_likelihood_impl<double>(x);
        }

        virtual double log_likelihood(const Eigen::VectorXd& x,
                                      const SystemEstimator& system,
                                      Eigen::VectorXd& g) const override;
        virtual double log_likelihood(const Eigen::VectorXd& x,
                                      const SystemEstimator& system,
                                      Eigen::VectorXd& g,
                                      Eigen::MatrixXd& H) const override;

        /// @brief Templated log-likelihood for autodiff.
        template <typename Scalar>
        Scalar log_likelihood_impl(const Eigen::VectorX<Scalar>& x) const {
            using std::sqrt;
            const Eigen::Vector4<Scalar> q = x.segment(srif::SystemLocalisation::i_quat, 4);
            const Scalar n                 = sqrt(q(0) * q(0) + q(1) * q(1) + q(2) * q(2) + q(3) * q(3));
            const Scalar e                 = n - Scalar(1);
            const double sigma2            = sigma_ * sigma_;
            return Scalar(-0.5 * std::log(2.0 * M_PI * sigma2)) - Scalar(0.5) * e * e / Scalar(sigma2);
        }

    protected:
        double sigma_;  ///< Std dev on |q|
    };

}  // namespace module::localisation::measurement

#endif  // MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTQUATERNIONNORM_HPP
