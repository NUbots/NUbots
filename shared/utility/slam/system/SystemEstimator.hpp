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

#ifndef UTILITY_SLAM_SYSTEM_ESTIMATOR_HPP
#define UTILITY_SLAM_SYSTEM_ESTIMATOR_HPP

#include <vector>

#include <Eigen/Core>

#include "../gaussian/GaussianInfo.hpp"
#include "SystemBase.hpp"

namespace utility::slam::system {

    /**
     * @brief Class for system state estimation.
     *
     * This class extends SystemBase to provide state estimation functionality.
     */
    class SystemEstimator : public SystemBase {
    public:
        /**
         * @brief Construct a new SystemEstimator object with initial density.
         * @param density Initial state density.
         */
        SystemEstimator(const gaussian::GaussianInfo<double>& density);

        /**
         * @brief Destroy the SystemEstimator object.
         */
        virtual ~SystemEstimator() override;

        /**
         * @brief Predict the system state at a given time.
         * @param time The time to predict the system state for.
         */
        virtual void predict(double time) override;

        gaussian::GaussianInfo<double> density;  ///< The current state density estimate.

        /**
         * @brief Compute the estimated system dynamics.
         * @param t Time.
         * @param x State vector.
         * @return The computed dynamics (state derivative).
         */
        virtual Eigen::VectorXd dynamicsEst(double t, const Eigen::VectorXd& x) const;

        /**
         * @brief Compute the estimated system dynamics and its Jacobian.
         * @param t Time.
         * @param x State vector.
         * @param J Output parameter for the Jacobian matrix.
         * @return The computed dynamics (state derivative).
         */
        virtual Eigen::VectorXd dynamicsEst(double t, const Eigen::VectorXd& x, Eigen::MatrixXd& J) const;

    protected:
        /**
         * @brief Compute the augmented dynamics estimate.
         * @param t Time.
         * @param X The augmented state matrix.
         * @return Eigen::MatrixXd The augmented dynamics.
         */
        Eigen::MatrixXd augmentedDynamicsEst(double t, const Eigen::MatrixXd& X) const;

        /**
         * @brief Helper function for Runge-Kutta 4th order method for SDEs.
         * @param xdw The state and noise vector.
         * @param dt The time step.
         * @param J Output parameter for the Jacobian matrix.
         * @return The updated state vector.
         */
        Eigen::VectorXd RK4SDEHelper(const Eigen::VectorXd& xdw, double dt, Eigen::MatrixXd& J) const;

        /**
         * @brief Compute the process noise density.
         * @param dt The time step.
         * @return The process noise density.
         */
        virtual gaussian::GaussianInfo<double> processNoiseDensity(double dt) const = 0;

        /**
         * @brief Get the indices of state variables affected by process noise.
         * @return The indices of affected state variables.
         */
        virtual std::vector<Eigen::Index> processNoiseIndex() const = 0;

        double dtMaxEst = 1e-2;  ///< Maximum time step for process model prediction
    };

}  // namespace utility::slam::system

#endif  // UTILITY_SLAM_SYSTEM_ESTIMATOR_HPP
