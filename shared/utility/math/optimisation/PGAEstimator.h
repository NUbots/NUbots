/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2014 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_OPTIMISATION_PGA_H
#define UTILITY_MATH_OPTIMISATION_PGA_H

#include <armadillo>
#include <cmath>

#include "message/support/optimisation/OptimiserTypes.h"

namespace utility {
namespace math {
    namespace optimisation {
        using message::support::optimisation::OptimiserEstimate;
        using message::support::optimisation::OptimiserParameters;

        /**
         * The PGAEstimator provides a steady-state solution to the PI2 path integral RL algorithm.
         * This can be used as a black-box, gradient-free local optimiser.
         */
        class PGAEstimator {
        private:
            double c = 7.0;

        public:
            /**
             * Initialise the estimator with a new starting point in parameter space
             *
             * @param params - the starting optimisation point for the algorithm
             *
             * @author Josiah Walker
             */
            PGAEstimator(const OptimiserParameters& /*params*/) {
                // XXX: still no good place to set c!
            }

            void clear() {}

            OptimiserEstimate updateEstimate(const arma::mat& samples,
                                             const arma::vec& fitnesses,
                                             const OptimiserEstimate& previousEstimate) {

                // create a vector of normed fitnesses
                const double min = arma::min(fitnesses);
                const double max = arma::max(fitnesses);
                const arma::vec normedFitnesses =
                    (max - fitnesses) / (max - min + std::numeric_limits<double>::epsilon());

                // create a set of weights per sample which specifies the likelihood that they are near the best
                // estimate
                const arma::vec sampleWeights = arma::exp(-c * normedFitnesses);

                // calculate the probabilistically weighted result estimate
                arma::vec bestEstimate =
                    arma::sum(samples % arma::repmat(sampleWeights / arma::accu(sampleWeights), 1, samples.n_cols), 0)
                        .t();

                // calculate the covariance matrix
                arma::mat s2 = samples;
                s2.each_row() -= bestEstimate.t();
                arma::mat covmat = s2 * s2.t();

                return OptimiserEstimate(
                    previousEstimate.generation + 1, convert<double>(bestEstimate), previousEstimate.covariance);
            }
        };
    }  // namespace optimisation
}  // namespace math
}  // namespace utility


#endif  // UTILITY_MATH_COORDINATES_H
