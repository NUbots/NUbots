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
 * Under the quasi-static assumption the accelerometer measures the specific
 * force in the torso frame:
 *   y = Rfb(Theta)^T * [0, 0, g] + v,   v ~ N(0, sigma^2 I3)
 * which informs the roll and pitch states. The noise should be inflated
 * relative to the sensor datasheet to account for walking-induced
 * accelerations violating the quasi-static assumption.
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
        virtual double logLikelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x,
                                     const SystemEstimator& system,
                                     Eigen::VectorXd& g) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x,
                                     const SystemEstimator& system,
                                     Eigen::VectorXd& g,
                                     Eigen::MatrixXd& H) const override;

        /**
         * @brief Templated log-likelihood for autodiff.
         */
        template <typename Scalar>
        Scalar logLikelihoodImpl(const Eigen::VectorX<Scalar>& x) const;

    protected:
        Eigen::Vector3d y_;  ///< Measured specific force in torso frame [m/s^2]
        double sigma_;       ///< Noise standard deviation [m/s^2]

        static constexpr double gravity_ = 9.80665;  ///< Standard gravity [m/s^2]
    };

    template <typename Scalar>
    Scalar MeasurementGravity::logLikelihoodImpl(const Eigen::VectorX<Scalar>& x) const {
        const Eigen::Vector4<Scalar> q   = x.segment(srif::SystemLocalisation::iQuat, 4);
        const Eigen::Matrix3<Scalar> Rfb = quat2rot(q);

        Eigen::Vector3<Scalar> gf(Scalar(0), Scalar(0), Scalar(gravity_));
        Eigen::Vector3<Scalar> yhat = Rfb.transpose() * gf;

        const double sigma2      = sigma_ * sigma_;
        Eigen::Vector3<Scalar> e = y_.cast<Scalar>() - yhat;
        return Scalar(-1.5 * std::log(2.0 * M_PI * sigma2)) - Scalar(0.5) * e.squaredNorm() / Scalar(sigma2);
    }
}  // namespace module::localisation::measurement

#endif  // MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTGRAVITY_HPP
