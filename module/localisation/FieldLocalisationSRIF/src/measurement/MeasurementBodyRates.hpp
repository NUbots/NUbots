/**
 * @file MeasurementBodyRates.hpp
 * @brief Direct measurements of the body-fixed velocity states.
 *
 * These are the two signals that used to be a known input to the process model.
 * Making them measurements is the point of carrying vBb and omegaBb as states:
 *
 *  - an input asserts its value as truth, so its noise can only be expressed as
 *    process noise on whatever it drives, and any bias it carries is unmodelled;
 *  - a measurement carries its own sigma and can be gated, suppressed or
 *    contradicted by the rest of the belief, and a bias on it can be a state.
 *
 * That last point is what the gyroscope bias needed. It is unobservable to the
 * upstream Mahony filter (whose bias integrator is driven by the gravity error, a
 * cross product of two near-vertical vectors, which has no component about the
 * vertical), so nothing in the system was estimating the yaw-rate bias -- the one
 * that turns into steady heading drift.
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
        virtual double logLikelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x,
                                     const SystemEstimator& system,
                                     Eigen::VectorXd& g) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x,
                                     const SystemEstimator& system,
                                     Eigen::VectorXd& g,
                                     Eigen::MatrixXd& H) const override;

        /// @brief Templated log-likelihood for autodiff.
        template <typename Scalar>
        Scalar logLikelihoodImpl(const Eigen::VectorX<Scalar>& x) const {
            const Eigen::Vector3<Scalar> yhat =
                Eigen::Vector3<Scalar>(x.segment(srif::SystemLocalisation::iOmega, 3))
                + Eigen::Vector3<Scalar>(x.segment(srif::SystemLocalisation::iGyroBias, 3));
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
     * @brief Body-fixed linear velocity, from walk-engine odometry or a zero-velocity update.
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
        virtual double logLikelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x,
                                     const SystemEstimator& system,
                                     Eigen::VectorXd& g) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x,
                                     const SystemEstimator& system,
                                     Eigen::VectorXd& g,
                                     Eigen::MatrixXd& H) const override;

        /// @brief Templated log-likelihood for autodiff.
        template <typename Scalar>
        Scalar logLikelihoodImpl(const Eigen::VectorX<Scalar>& x) const {
            const Eigen::Vector3<Scalar> e =
                y_.cast<Scalar>() - Eigen::Vector3<Scalar>(x.segment(srif::SystemLocalisation::iVel, 3));
            const double sigma2 = sigma_ * sigma_;
            return Scalar(-1.5 * std::log(2.0 * M_PI * sigma2)) - Scalar(0.5) * e.squaredNorm() / Scalar(sigma2);
        }

    protected:
        Eigen::Vector3d y_;  ///< Measured body velocity [m/s]
        double sigma_;       ///< Noise standard deviation [m/s]
    };

}  // namespace module::localisation::measurement

#endif  // MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTBODYRATES_HPP
