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

#ifndef UTILITY_GAUSSIAN_FILTERING_MEASUREMENT_MEASUREMENT_HPP
#define UTILITY_GAUSSIAN_FILTERING_MEASUREMENT_MEASUREMENT_HPP

#include <Eigen/Core>

#include "utility/gaussian_filtering/Event.hpp"
#include "utility/gaussian_filtering/system/SystemBase.hpp"
#include "utility/gaussian_filtering/system/SystemEstimator.hpp"

namespace utility::gaussian_filtering::measurement {

    using system::SystemBase;
    using system::SystemEstimator;

    /// @brief Base class for measurement events, which update a system's density by a MAP
    ///        optimisation over the measurement log-likelihood
    class Measurement : public Event {
    public:
        /// @param time The time at which the measurement occurs
        Measurement(double time);

        virtual ~Measurement() override;

        /// @brief Simulate a measurement given a state
        virtual Eigen::VectorXd simulate(const Eigen::VectorXd& x, const SystemEstimator& system) const = 0;

        /// @brief The log-likelihood of the measurement at a state
        virtual double log_likelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const = 0;

        /// @brief The log-likelihood and its gradient g
        virtual double log_likelihood(const Eigen::VectorXd& x,
                                      const SystemEstimator& system,
                                      Eigen::VectorXd& g) const = 0;

        /// @brief The log-likelihood, its gradient g and its Hessian H
        virtual double log_likelihood(const Eigen::VectorXd& x,
                                      const SystemEstimator& system,
                                      Eigen::VectorXd& g,
                                      Eigen::MatrixXd& H) const = 0;


        /**
         * @brief Laplace approximation of the log evidence log p(y) from the last update:
         *        log p(y) ~= -V(x*) + (n/2) log(2 pi) - sum(log|diag(Xi)|),
         *        with V the negative log joint density at the MAP estimate x* and Xi the posterior
         *        sqrt information matrix. NaN until update() has run.
         */
        double log_evidence() const {
            return log_evidence_;
        }

        /**
         * @brief Re-run data association against the system's current density. Called once per
         *        hypothesis. Does nothing by default.
         */
        virtual void reassociate(const SystemEstimator& /*system*/) {}

    protected:
        /// @brief The negative log joint density of the state and measurement
        double cost_joint_density(const Eigen::VectorXd& x, const SystemEstimator& system) const;

        /// @brief The negative log joint density and its gradient g
        double cost_joint_density(const Eigen::VectorXd& x, const SystemEstimator& system, Eigen::VectorXd& g) const;

        /// @brief The negative log joint density, its gradient g and its Hessian H
        double cost_joint_density(const Eigen::VectorXd& x,
                                  const SystemEstimator& system,
                                  Eigen::VectorXd& g,
                                  Eigen::MatrixXd& H) const;

        /// @brief Update the system's density with this measurement
        virtual void update(SystemBase& system) override;

        /// @brief The optimiser used by update()
        enum class UpdateMethod {
            BFGSTRUSTSQRT,
            BFGSLMSQRT,
            SR1TRUSTEIG,
            NEWTONTRUSTEIG,
            AFFINE,
            GAUSSNEWTON,
            LEVENBERGMARQUARDT
        };

        UpdateMethod update_method_;  ///< The optimiser used by update()

        double log_evidence_ = std::numeric_limits<double>::quiet_NaN();  ///< Laplace log evidence from the last update
    };

}  // namespace utility::gaussian_filtering::measurement

#endif  // UTILITY_GAUSSIAN_FILTERING_MEASUREMENT_MEASUREMENT_HPP
