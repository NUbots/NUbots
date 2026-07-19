/**
 * @file MeasurementKinematicHeight.h
 * @brief Forward-kinematic torso height pseudo-measurement.
 */
#ifndef MEASUREMENTKINEMATICHEIGHT_HPP
#define MEASUREMENTKINEMATICHEIGHT_HPP

#include <Eigen/Core>

#include "../system/SystemEstimator.hpp"
#include "Measurement.hpp"

/**
 * @class MeasurementKinematicHeight
 * @brief Torso height above the ground plane from forward kinematics.
 *
 * The support-leg kinematic chain gives the torso height above the ground.
 * Since the field frame origin lies on the ground plane,
 *   y = x_z + v,   v ~ N(0, sigma^2)
 * This anchors the otherwise weakly observable z state.
 */

namespace utility::slam::measurement {
    class MeasurementKinematicHeight : public Measurement {
    public:
        /**
         * @brief Construct a kinematic height measurement.
         * @param time Event time [s]
         * @param height Torso height above ground from kinematics [m]
         * @param sigma Noise standard deviation [m]
         */
        MeasurementKinematicHeight(double time, double height, double sigma = 0.02);

        virtual Eigen::VectorXd simulate(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x,
                                     const SystemEstimator& system,
                                     Eigen::VectorXd& g) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x,
                                     const SystemEstimator& system,
                                     Eigen::VectorXd& g,
                                     Eigen::MatrixXd& H) const override;

    protected:
        double y_;      ///< Measured torso height [m]
        double sigma_;  ///< Noise standard deviation [m]
    };
}  // namespace utility::slam::measurement
#endif
