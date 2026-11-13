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
 * @file MeasurementGravity.h
 * @brief Accelerometer gravity-direction measurement of torso roll/pitch.
 */
#ifndef MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTGRAVITY_HPP
#define MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTGRAVITY_HPP

#include <Eigen/Core>

#include "srif/SystemLocalisation.hpp"

#include "utility/gaussian_filtering/measurement/Measurement.hpp"
#include "utility/gaussian_filtering/rotation.hpp"
#include "utility/gaussian_filtering/system/SystemEstimator.hpp"

/**
 * @class MeasurementGravity
 * @brief Quasi-static accelerometer measurement of the gravity direction.
 *
 * When the torso is not accelerating the accelerometer measures
 *   y = Rfb(Theta)^T * [0, 0, g] + v,   v ~ N(0, sigma^2 I3)
 * which informs roll and pitch. sigma is larger than the sensor noise to cover walking
 * accelerations.
 */

namespace module::localisation::measurement {

    using srif::SystemLocalisation;
    using utility::gaussian_filtering::quat2rot;
    using utility::gaussian_filtering::measurement::Measurement;
    using utility::gaussian_filtering::system::SystemEstimator;

    class MeasurementGravity : public Measurement {
    public:
        /**
         * @brief Construct an accelerometer gravity measurement.
         * @param time Event time [s]
         * @param accelerometer Measured specific force in torso frame [m/s^2]
         * @param sigma Noise standard deviation per axis [m/s^2]
         */
        MeasurementGravity(double time, const Eigen::Vector3d& accelerometer, double sigma = 1.0);

        virtual Eigen::VectorXd simulate(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double log_likelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double log_likelihood(const Eigen::VectorXd& x,
                                      const SystemEstimator& system,
                                      Eigen::VectorXd& g) const override;
        virtual double log_likelihood(const Eigen::VectorXd& x,
                                      const SystemEstimator& system,
                                      Eigen::VectorXd& g,
                                      Eigen::MatrixXd& H) const override;

        /// @brief Templated log-likelihood for autodiff
        template <typename Scalar>
        Scalar log_likelihood_impl(const Eigen::VectorX<Scalar>& x) const;

    protected:
        Eigen::Vector3d y_;  ///< Measured specific force in torso frame [m/s^2]
        double sigma_;       ///< Noise standard deviation [m/s^2]

        static constexpr double gravity_ = 9.80665;  ///< Standard gravity [m/s^2]
    };

    template <typename Scalar>
    Scalar MeasurementGravity::log_likelihood_impl(const Eigen::VectorX<Scalar>& x) const {
        const Eigen::Vector4<Scalar> q   = x.segment(srif::SystemLocalisation::i_quat, 4);
        const Eigen::Matrix3<Scalar> Rfb = quat2rot(q);

        Eigen::Vector3<Scalar> gf(Scalar(0), Scalar(0), Scalar(gravity_));
        Eigen::Vector3<Scalar> yhat = Rfb.transpose() * gf;

        const double sigma2      = sigma_ * sigma_;
        Eigen::Vector3<Scalar> e = y_.cast<Scalar>() - yhat;
        return Scalar(-1.5 * std::log(2.0 * M_PI * sigma2)) - Scalar(0.5) * e.squaredNorm() / Scalar(sigma2);
    }
}  // namespace module::localisation::measurement

#endif  // MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTGRAVITY_HPP
