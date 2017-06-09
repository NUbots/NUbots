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
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_OPTIMISATION_PGPE_H
#define UTILITY_MATH_OPTIMISATION_PGPE_H

#include <cmath>

#include "message/support/optimisation/OptimiserTypes.h"

namespace utility {
    namespace math {
        namespace optimisation {
            using message::support::optimisation::OptimiserParameters;
            using message::support::optimisation::OptimiserEstimate;

            class PGPEEstimator {
            private:
                double baseline = 0.0;
                double learningRate = 0.1;
                double sigmaLearningRate = 0.2;
                bool firstRun = true;

            public:
                /**
                 * Initialise the estimator with a new starting point in parameter space
                 *
                 * @param params - the starting optimisation point for the algorithm
                 *
                 * @author Josiah Walker
                 */
                PGPEEstimator(const OptimiserParameters& /*params*/) {
                     //XXX: in the future, set learning rate through params
                };

                void clear() {
                    firstRun = true;
                }

                /**
                 * Generate a new best-estimate using the parameter samples and fitnesses provided.
                 * Takes a row-wise list of sample parameters, a corresponding vector of fitnesses
                 *
                 * @param samples - the tested samples in the matrix format returned by getSamples below (one sample per row)
                 * @param fitnesses - a vector of fitnesses corresponding to each sample
                 * @param variances - a vector of variances corresponding to estimated gradient change in each dimension
                 *
                 * @returns currentEstimate - an updated best parameter estimate vector to re-sample from
                 *
                 * @author Josiah Walker
                 */
                OptimiserEstimate updateEstimate(const arma::mat& samples, const arma::vec& fitnesses, OptimiserEstimate& previousEstimate) {
                    Eigen::VectorXd bestEstimate = previousEstimate.estimate;
                    Eigen::VectorXd covEstimate = arma::diagvec(previousEstimate.covariance);

                    if (firstRun) {
                        firstRun = false;
                        baseline = fitnesses.mean();
                    }

                    Eigen::VectorXd alpha = covEstimate * learningRate;
                    Eigen::VectorXd alphaCov = covEstimate * sigmaLearningRate;
                    Eigen::VectorXd update(covEstimate.n_elem,arma::fill::zeros);
                    Eigen::VectorXd updateCov(covEstimate.n_elem,arma::fill::zeros);
                    for(uint64_t i = 0; i < fitnesses.n_elem; ++i) {
                        update += alpha * (fitnesses[i]-baseline) % (samples.row(i).transpose() - bestEstimate);
                        Eigen::VectorXd x = Eigen::square((samples.row(i).transpose() - bestEstimate).array()).matrix() - covEstimate;
                        updateCov += alphaCov * (fitnesses[i] - baseline).cwiseProduct(x).cwiseQuotient(Eigen::sqrt(covEstimate.array()).matrix());
                    }

                    baseline = baseline * 0.9 + 0.1 * fitnesses.mean();

                    bestEstimate += update;
                    covEstimate += updateCov;

                    return OptimiserEstimate(previousEstimate.generation + 1, bestEstimate, arma::mat(diagmat(covEstimate)));
                }
            };

        }
    }
}


#endif // UTILITY_MATH_COORDINATES_H

