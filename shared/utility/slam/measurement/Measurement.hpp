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

#ifndef UTILITY_SLAM_MEASUREMENT_MEASUREMENT_HPP
#define UTILITY_SLAM_MEASUREMENT_MEASUREMENT_HPP

#include <Eigen/Core>

#include "../Event.hpp"
#include "../system/SystemBase.hpp"
#include "../system/SystemEstimator.hpp"

namespace utility::slam::measurement {

    // Bring types into scope
    using system::SystemBase;
    using system::SystemEstimator;

    /**
     * @brief Base class for measurement events in the system.
     *
     * This class represents a measurement event that can be processed by the system.
     * It inherits from the Event class and adds functionality specific to measurements.
     */
    class Measurement : public Event {
    public:
        /**
         * @brief Construct a new Measurement object.
         * @param time The time at which the measurement occurs.
         */
        Measurement(double time);

        /**
         * @brief Construct a new Measurement object.
         * @param time The time at which the measurement occurs.
         * @param verbosity The verbosity level for logging and output.
         */
        Measurement(double time, int verbosity);

        /**
         * @brief Destroy the Measurement object.
         */
        virtual ~Measurement() override;

        /**
         * @brief Simulate a measurement given a state and system.
         * @param x The state vector.
         * @param system The system estimator.
         * @return The simulated measurement.
         */
        virtual Eigen::VectorXd simulate(const Eigen::VectorXd& x, const SystemEstimator& system) const = 0;

        /**
         * @brief Calculate the log-likelihood of a measurement.
         * @param x The state vector.
         * @param system The system estimator.
         * @return The log-likelihood value.
         */
        virtual double logLikelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const = 0;

        /**
         * @brief Calculate the log-likelihood and its gradient.
         * @param x The state vector.
         * @param system The system estimator.
         * @param g Output parameter for the gradient.
         * @return The log-likelihood value.
         */
        virtual double logLikelihood(const Eigen::VectorXd& x,
                                    const SystemEstimator& system,
                                    Eigen::VectorXd& g) const = 0;

        /**
         * @brief Calculate the log-likelihood, its gradient, and Hessian.
         * @param x The state vector.
         * @param system The system estimator.
         * @param g Output parameter for the gradient.
         * @param H Output parameter for the Hessian.
         * @return The log-likelihood value.
         */
        virtual double logLikelihood(const Eigen::VectorXd& x,
                                    const SystemEstimator& system,
                                    Eigen::VectorXd& g,
                                    Eigen::MatrixXd& H) const = 0;

    protected:
        /**
         * @brief Calculate the cost of the joint density.
         * @param x The state vector.
         * @param system The system estimator.
         * @return The cost value.
         */
        double costJointDensity(const Eigen::VectorXd& x, const SystemEstimator& system) const;

        /**
         * @brief Calculate the cost of the joint density and its gradient.
         * @param x The state vector.
         * @param system The system estimator.
         * @param g Output parameter for the gradient.
         * @return The cost value.
         */
        double costJointDensity(const Eigen::VectorXd& x, const SystemEstimator& system, Eigen::VectorXd& g) const;

        /**
         * @brief Calculate the cost of the joint density, its gradient, and Hessian.
         * @param x The state vector.
         * @param system The system estimator.
         * @param g Output parameter for the gradient.
         * @param H Output parameter for the Hessian.
         * @return The cost value.
         */
        double costJointDensity(const Eigen::VectorXd& x,
                            const SystemEstimator& system,
                            Eigen::VectorXd& g,
                            Eigen::MatrixXd& H) const;

        /**
         * @brief Update the system based on this measurement.
         * @param system The system to update.
         */
        virtual void update(SystemBase& system) override;

        /**
         * @brief Enumeration of update methods.
         */
        enum class UpdateMethod { BFGSTRUSTSQRT, BFGSLMSQRT, SR1TRUSTEIG, NEWTONTRUSTEIG, AFFINE, GAUSSNEWTON, LEVENBERGMARQUARDT };

        UpdateMethod updateMethod_;  ///< The method used for updating the system.
    };

}  // namespace utility::slam::measurement

#endif  // UTILITY_SLAM_MEASUREMENT_MEASUREMENT_HPP
