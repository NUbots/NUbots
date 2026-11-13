/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

#ifndef UTILITY_GAUSSIAN_FILTERING_SYSTEM_ESTIMATOR_HPP
#define UTILITY_GAUSSIAN_FILTERING_SYSTEM_ESTIMATOR_HPP

#include <Eigen/Core>
#include <vector>

#include "../gaussian/GaussianInfo.hpp"
#include "SystemBase.hpp"

namespace utility::gaussian_filtering::system {

    /// @brief A system whose state is estimated as a Gaussian in square-root information form
    class SystemEstimator : public SystemBase {
    public:
        /// @param density Initial state density
        SystemEstimator(const gaussian::GaussianInfo<double>& density);

        virtual ~SystemEstimator() override;

        /// @brief Predict the state density to a given time
        virtual void predict(double time) override;

        gaussian::GaussianInfo<double> density;  ///< The current state density estimate

        /**
         * @brief Compute the estimated system dynamics and its Jacobian.
         * @param t Time.
         * @param x State vector.
         * @param J Output parameter for the Jacobian matrix.
         * @return The computed dynamics (state derivative).
         */
        virtual Eigen::VectorXd dynamics_est(double t, const Eigen::VectorXd& x, Eigen::MatrixXd& J) const;

    protected:
        /**
         * @brief Compute the augmented dynamics estimate.
         * @param t Time.
         * @param X The augmented state matrix.
         * @return Eigen::MatrixXd The augmented dynamics.
         */
        Eigen::MatrixXd augmented_dynamics_est(double t, const Eigen::MatrixXd& X) const;

        /**
         * @brief One RK4 step of the SDE.
         * @param xdw The state and noise vector.
         * @param dt The time step.
         * @param J Output parameter for the Jacobian matrix.
         * @return The updated state vector.
         */
        Eigen::VectorXd rk4_sde_helper(const Eigen::VectorXd& xdw, double dt, Eigen::MatrixXd& J) const;

        /// @brief The density of the process noise increment over a time step
        virtual gaussian::GaussianInfo<double> process_noise_density(double dt) const = 0;

        /// @brief The indices of the states affected by process noise
        virtual std::vector<Eigen::Index> process_noise_index() const = 0;

        double dt_max_est = 1e-2;  ///< Maximum time step for process model prediction
    };

}  // namespace utility::gaussian_filtering::system

#endif  // UTILITY_GAUSSIAN_FILTERING_SYSTEM_ESTIMATOR_HPP
