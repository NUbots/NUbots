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
 * @file MeasurementKinematicHeight.h
 * @brief Forward-kinematic torso height pseudo-measurement.
 */
#ifndef MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTKINEMATICHEIGHT_HPP
#define MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTKINEMATICHEIGHT_HPP

#include <Eigen/Core>

#include "utility/gaussian_filtering/measurement/Measurement.hpp"
#include "utility/gaussian_filtering/system/SystemEstimator.hpp"

/**
 * @class MeasurementKinematicHeight
 * @brief Torso height above the ground plane from forward kinematics.
 *
 * The field frame origin is on the ground plane, so
 *   y = x_z + v,   v ~ N(0, sigma^2)
 */

namespace module::localisation::measurement {

    using utility::gaussian_filtering::measurement::Measurement;
    using utility::gaussian_filtering::system::SystemEstimator;

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
        virtual double log_likelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double log_likelihood(const Eigen::VectorXd& x,
                                      const SystemEstimator& system,
                                      Eigen::VectorXd& g) const override;
        virtual double log_likelihood(const Eigen::VectorXd& x,
                                      const SystemEstimator& system,
                                      Eigen::VectorXd& g,
                                      Eigen::MatrixXd& H) const override;

    protected:
        double y_;      ///< Measured torso height [m]
        double sigma_;  ///< Noise standard deviation [m]
    };
}  // namespace module::localisation::measurement
#endif  // MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTKINEMATICHEIGHT_HPP
