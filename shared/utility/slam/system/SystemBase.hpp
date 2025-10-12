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

#ifndef UTILITY_SLAM_SYSTEM_BASE_HPP
#define UTILITY_SLAM_SYSTEM_BASE_HPP

#include <Eigen/Core>

namespace utility::slam::system {

    /**
     * @brief Base class for system representation.
     *
     * This class provides a basic interface for system dynamics and prediction.
     */
    class SystemBase {
    public:
        /**
         * @brief Construct a new SystemBase object.
         */
        SystemBase();

        /**
         * @brief Destroy the SystemBase object.
         */
        virtual ~SystemBase();

        /**
         * @brief Predict the system state at a given time.
         * @param time The time to predict the system state for.
         */
        virtual void predict(double time) = 0;

        /**
         * @brief Compute the system dynamics.
         * @param t The time to evaluate the dynamics at.
         * @param x The state to evaluate the dynamics at.
         * @param u The input to evaluate the dynamics at.
         * @return The computed dynamics (state derivative).
         */
        virtual Eigen::VectorXd dynamics(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const = 0;

        /**
         * @brief Compute the system dynamics and its Jacobian.
         * @param t The time to evaluate the dynamics at.
         * @param x The state to evaluate the dynamics at.
         * @param u The input to evaluate the dynamics at.
         * @param J Output parameter for the Jacobian matrix.
         * @return The computed dynamics (state derivative).
         */
        virtual Eigen::VectorXd dynamics(double t,
                                        const Eigen::VectorXd& x,
                                        const Eigen::VectorXd& u,
                                        Eigen::MatrixXd& J) const = 0;

        /**
         * @brief Compute the system input
         * @param t The time to evaluate the input at.
         * @param x The state to evaluate the input at.
         * @return The computed input signal.
         */
        virtual Eigen::VectorXd input(double t, const Eigen::VectorXd& x) const = 0;

    protected:
        double time_;  ///< The current system time.
    };

}  // namespace utility::slam::system

#endif  // UTILITY_SLAM_SYSTEM_BASE_HPP
