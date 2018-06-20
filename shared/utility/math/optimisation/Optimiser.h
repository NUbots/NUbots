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

#ifndef UTILITY_MATH_OPTIMISER_H
#define UTILITY_MATH_OPTIMISER_H

#include <armadillo>
#include <cmath>

#include "message/support/optimisation/OptimiserTypes.h"

#include "CholeskySampler.h"
#include "GaussianSampler.h"
#include "MirroredSampler.h"

#include "PGAEstimator.h"
#include "PGPEEstimator.h"
//#include "WMDOptimiser.h"

namespace utility {
namespace math {
    namespace optimisation {
        using message::support::optimisation::OptimiserEstimate;
        using message::support::optimisation::OptimiserParameters;

        class Optimiser {
        public:
            virtual const OptimiserEstimate& estimate()                                      = 0;
            virtual OptimiserEstimate updateEstimate(arma::mat samples, arma::vec fitnesses) = 0;
            virtual arma::mat getSamples(const uint& numSamples = 7)                         = 0;
            virtual bool validSample(...)                                                    = 0;
            virtual void reset()                                                             = 0;
            virtual void reset(const OptimiserEstimate& est)                                 = 0;
            virtual ~Optimiser()                                                             = default;
        };

        /**
         *  This class is a generic container for direct policy
         *  optimisation methods. It supports splitting of
         *  optimisation and sampling steps to allow recombining
         *  for best results.
         *
         * @author Josiah Walker
         */
        template <typename OptMethod, typename SampleMethod>
        class OptimiserSet : public Optimiser {
        private:
            OptMethod estimator;
            SampleMethod sampler;
            OptimiserEstimate startValues;
            OptimiserEstimate currentValues;

        public:
            OptimiserSet(const OptimiserParameters& params)
                : estimator(params), sampler(params), startValues(params.initial), currentValues(params.initial) {}

            /**
             * Generate a new best-estimate using the parameter samples and fitnesses provided.
             * Takes a row-wise list of sample parameters, a corresponding vector of fitnesses
             *
             * @param samples - the tested samples in the matrix format returned by getSamples below (one sample per
             * row)
             * @param fitnesses - a vector of fitnesses corresponding to each sample
             *
             * @returns currentEstimate - an updated best parameter estimate vector to re-sample from
             *
             * @author Josiah Walker
             */
            virtual OptimiserEstimate updateEstimate(arma::mat samples, arma::vec fitnesses) {

                currentValues = estimator.updateEstimate(samples, fitnesses, currentValues);
                return currentValues;
            }

            virtual const OptimiserEstimate& estimate() {
                return currentValues;
            }


            virtual bool validSample(...) {
                return true;
            }

            /**
             * Draw a new set of parameter samples and return them to be evaluated.
             *
             * @param numSamples - the number of test parameter sets to generate (this varies, but 5-12 is usually good)
             *
             * @returns newSamples - a numSamples rows by numParams cols matrix of sample parameter sets to evaluate
             *
             * @author Josiah walker
             */
            virtual arma::mat getSamples(const uint& numSamples = 7) {
                return sampler.getSamples(currentValues, numSamples);
            }

            virtual void reset() {
                currentValues = startValues;
                sampler.clear();
                estimator.clear();
            }

            virtual void reset(const OptimiserEstimate& est) {
                currentValues = est;
                sampler.clear();
                estimator.clear();
            }

            /*
            XXX: unimplemented - get internal data to be able to save/resume optimisation
            std::pair<arma::mat,arma::mat> getInternalData() {
                return;
            }

            void setInternalData(const arma::mat& optData, const arma::mat& sampleData) {
                return;
            }*/
        };

        // easy typedefs for working with the included algorithms
        using PGAOptimiser = OptimiserSet<PGAEstimator, GaussianSampler>;
        // using WMDOptimiser  =  OptimiserSet<WMDEstimator, GaussianSampler>;
        using PGPEOptimiser = OptimiserSet<PGPEEstimator, GaussianSampler>;
    }  // namespace optimisation
}  // namespace math
}  // namespace utility


#endif  // UTILITY_MATH_OPTIMISER_H
