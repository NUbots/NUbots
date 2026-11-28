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
 * @file MeasurementBodyRates.hpp
 * @brief Direct measurements of the body-fixed velocity states.
 *
 * As measurements rather than inputs they carry their own noise and can be gated, and the
 * gyroscope bias can be estimated. Nothing upstream estimates the yaw-rate bias (Sensors takes
 * its yaw from the walk controller's odometry), so unestimated it becomes steady heading drift.
 */
#ifndef MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTBODYRATES_HPP
#define MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTBODYRATES_HPP

#include <Eigen/Core>
#include <cmath>

#include "srif/SystemLocalisation.hpp"

#include "utility/gaussian_filtering/measurement/Measurement.hpp"
#include "utility/gaussian_filtering/system/SystemEstimator.hpp"

namespace module::localisation::measurement {

    using srif::SystemLocalisation;
    using utility::gaussian_filtering::measurement::Measurement;
    using utility::gaussian_filtering::system::SystemEstimator;

    /**
     * @class MeasurementGyroscope
     * @brief Torso gyroscope as a measurement of the body angular velocity.
     *
     *   y = omegaBb + bGyro + v,   v ~ N(0, sigma^2 I3)
     *
     */
    class MeasurementGyroscope : public Measurement {
    public:
        /**
         * @brief Construct a gyroscope measurement.
         * @param time Event time [s]
         * @param gyroscope Measured angular velocity in the torso frame [rad/s]
         * @param sigma Noise standard deviation per axis [rad/s]
         */
        MeasurementGyroscope(double time, const Eigen::Vector3d& gyroscope, double sigma = 0.02);

        virtual Eigen::VectorXd simulate(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double log_likelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
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
            const Eigen::Vector3<Scalar> yhat =
                Eigen::Vector3<Scalar>(x.segment(srif::SystemLocalisation::i_omega, 3))
                + Eigen::Vector3<Scalar>(x.segment(srif::SystemLocalisation::i_gyro_bias, 3));
            const Eigen::Vector3<Scalar> e = y_.cast<Scalar>() - yhat;
            const double sigma2            = sigma_ * sigma_;
            return Scalar(-1.5 * std::log(2.0 * M_PI * sigma2)) - Scalar(0.5) * e.squaredNorm() / Scalar(sigma2);
        }

    protected:
        Eigen::Vector3d y_;  ///< Measured angular velocity [rad/s]
        double sigma_;       ///< Noise standard deviation [rad/s]
    };

    /**
     * @class MeasurementBodyVelocity
     * @brief Body-fixed linear velocity, from odometry or a zero-velocity update.
     *
     *   y = vBb + v,   v ~ N(0, sigma^2 I3)
     *
     */
    class MeasurementBodyVelocity : public Measurement {
    public:
        /**
         * @brief Construct a body-velocity measurement.
         * @param time Event time [s]
         * @param velocity Measured body-fixed linear velocity [m/s]
         * @param sigma Noise standard deviation per axis [m/s]
         */
        MeasurementBodyVelocity(double time, const Eigen::Vector3d& velocity, double sigma = 0.15);

        /// @brief A zero-velocity update: the robot is known not to be travelling.
        static MeasurementBodyVelocity stationary(double time, double sigma = 0.02) {
            return MeasurementBodyVelocity(time, Eigen::Vector3d::Zero(), sigma);
        }

        virtual Eigen::VectorXd simulate(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double log_likelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
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
            const Eigen::Vector3<Scalar> e =
                y_.cast<Scalar>() - Eigen::Vector3<Scalar>(x.segment(srif::SystemLocalisation::i_vel, 3));
            const double sigma2 = sigma_ * sigma_;
            return Scalar(-1.5 * std::log(2.0 * M_PI * sigma2)) - Scalar(0.5) * e.squaredNorm() / Scalar(sigma2);
        }

    protected:
        Eigen::Vector3d y_;  ///< Measured body velocity [m/s]
        double sigma_;       ///< Noise standard deviation [m/s]
    };

    /**
     * @class MeasurementRest
     * @brief The robot is at rest (see srif::ImuPreFilter), after VQF's rest bias estimation.
     *
     * At rest the body angular velocity is zero, and the low-pass filtered gyroscope reads the bias:
     *
     *   0         = omegaBb + v1,   v1 ~ N(0, sigma_omega^2 I3)
     *   gyro_mean = bGyro   + v2,   v2 ~ N(0, sigma_bias^2 I3)
     *
     * This is what makes the yaw-rate bias observable without vision. The gyro_mean of successive
     * vision frames are strongly correlated (they share the low-pass filter's memory), so sigma_bias
     * is per frame and deliberately loose.
     */
    class MeasurementRest : public Measurement {
    public:
        /**
         * @brief Construct a rest measurement.
         * @param time Event time [s]
         * @param gyroscope_mean Low-pass filtered gyroscope reading [rad/s]
         * @param sigma_omega Noise standard deviation of the zero angular velocity per axis [rad/s]
         * @param sigma_bias Noise standard deviation of the bias reading per axis [rad/s]
         */
        MeasurementRest(double time, const Eigen::Vector3d& gyroscope_mean, double sigma_omega, double sigma_bias);

        virtual Eigen::VectorXd simulate(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double log_likelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
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
            const Eigen::Vector3<Scalar> e_omega =
                Eigen::Vector3<Scalar>(x.segment(srif::SystemLocalisation::i_omega, 3));
            const Eigen::Vector3<Scalar> e_bias =
                y_.cast<Scalar>() - Eigen::Vector3<Scalar>(x.segment(srif::SystemLocalisation::i_gyro_bias, 3));
            const double so2 = sigma_omega_ * sigma_omega_;
            const double sb2 = sigma_bias_ * sigma_bias_;
            return Scalar(-1.5 * std::log(2.0 * M_PI * so2) - 1.5 * std::log(2.0 * M_PI * sb2))
                   - Scalar(0.5) * e_omega.squaredNorm() / Scalar(so2)
                   - Scalar(0.5) * e_bias.squaredNorm() / Scalar(sb2);
        }

    protected:
        Eigen::Vector3d y_;   ///< Low-pass filtered gyroscope reading [rad/s]
        double sigma_omega_;  ///< Zero angular velocity noise standard deviation [rad/s]
        double sigma_bias_;   ///< Bias reading noise standard deviation [rad/s]
    };

}  // namespace module::localisation::measurement

#endif  // MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTBODYRATES_HPP
