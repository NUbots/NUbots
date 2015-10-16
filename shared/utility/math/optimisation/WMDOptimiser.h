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
 * Copyright 2014 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_OPTIMISATION_WMD_H
#define UTILITY_MATH_OPTIMISATION_WMD_H

#include <armadillo>
#include <cmath>

namespace utility {
    namespace math {
        namespace optimisation {

            /**
             * XXX TODO Josiah, put the  description of this optimiser here
             */
            class WMDEstimator {
            private:
                arma::vec bestEstimate;
                arma::vec startPoint;
                const double c = 7.0;

            public:
                /**
                 * Initialise the estimator with a new starting point in parameter space
                 *
                 * @param params - the starting optimisation point for the algorithm
                 *
                 * @author Josiah Walker
                 */
                WMDEstimator(const arma::vec& params): startPoint(params), bestEstimate(params) {
                     //XXX: get c constant from inputs
                };

                void reset() {
                    bestEstimate = startPoint * 1.0;
                }

                arma::vec currentEstimate() {
                    return bestEstimate;
                }

                /**
                 * Generate a new best-estimate using the parameter samples and fitnesses provided.
                 * Takes a row-wise list of sample parameters, a corresponding vector of fitnesses
                 *
                 * @param samples - the tested samples in the matrix format returned by getSamples below (one sample per row)
                 * @param fitnesses - a vector of fitnesses corresponding to each sample
                 *
                 * @returns currentEstimate - an updated best parameter estimate vector to re-sample from
                 *
                 * @author Josiah Walker
                 */
                arma::vec updateEstimate(const arma::mat& samples, const arma::vec& fitnesses, const arma::vec& unused) {

                    //create a vector of normed fitnesses
                    const double min = arma::min(fitnesses);
                    const double max = arma::max(fitnesses);
                    const arma::vec normedFitnesses = (max-fitnesses)/(max-min+std::numeric_limits<double>::epsilon());

                    //create a set of weights per sample which specifies the likelihood that they are near the best estimate
                    const arma::vec sampleWeights = arma::exp(-c*normedFitnesses);

                    //NOTE: here we deviate from PGA: use the geometric median
                    //this is the iterative least squares solution to the
                    //Weber problem, modified slightly to behave around 0.
                    for (uint64_t j = 0; j < 50; ++j) {
                        arma::vec update(bestEstimate.n_elem,arma::fill::zeros);
                        double div = 0.0;
                        for (uint64_t i = 0; i < samples.n_rows; ++i) {
                            const double scale = sampleWeights[i]/(arma::norm(bestEstimate-samples.row(i).t()) + 0.01);
                            update += scale*samples.row(i).t();
                            div += scale;
                        }
                        bestEstimate = update/div;
                    }
                }
            };

            /**
             * XXX TODO Josiah, put the  description of this optimiser here
             */
            class WMDSampler {
            //NOTE: thise is identical to PGASampler
            private:
                arma::vec sigmaWeights;

            public:
                /**
                 * Initialise the estimator with a new starting point in parameter space
                 *
                 * @param params - the starting sigmas for each dimension (ie the scaling factor to step in each dimension)
                 *
                 * @author Josiah Walker
                 */
                WMDSampler(const arma::vec& params)
                : sigmaWeights(params) {}


                void reset() {
                }

                arma::vec getVariances() {
                    return arma::square(sigmaWeights);
                }

                updateEstimate(const arma::mat& samples, const arma::vec& fitnesses, const arma::vec& currentEstimate) {
                    ; //unused
                }

                /**
                 * Function to create a new set of parameter samples from a best estimate and
                 * a vec of per-dimension scales for the gaussian noise additive (sigmaweights).
                 * @param bestEstimate - the current estimated best parameter set
                 *
                 * @param numSamples - the number of test parameter sets to generate (this varies, but 5-12 is usually good)
                 *
                 * @returns newSamples - a numSamples rows by numParams cols matrix of sample parameter sets to try
                 *
                 * @author Josiah walker
                 */
                arma::mat getSamples(const arma::vec& bestEstimate, const size_t& numSamples) {
                    return   arma::randn<arma::mat>(numSamples,bestEstimate.n_elem)
                           % arma::repmat(sigmaWeights, 1, numSamples).t()
                           + arma::repmat(bestEstimate, 1, numSamples).t();
                }
            };
        }
    }
}


#endif // UTILITY_MATH_COORDINATES_H

