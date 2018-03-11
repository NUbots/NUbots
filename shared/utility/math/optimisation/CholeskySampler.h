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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_OPTIMISATION_CHOLESKYSAMPLER_H
#define UTILITY_MATH_OPTIMISATION_CHOLESKYSAMPLER_H

#include <armadillo>
#include <cmath>

#include "message/support/optimisation/OptimiserTypes.h"
#include "utility/support/eigen_armadillo.h"

namespace utility {
namespace math {
    namespace optimisation {
        using message::support::optimisation::OptimiserEstimate;
        using message::support::optimisation::OptimiserParameters;

        class CholeskySampler {
        private:
            uint64_t batchSize;
            uint64_t sampleCount = 0;
            int generation       = -1;
            arma::vec upperBound;
            arma::vec lowerBound;
            arma::mat samples;

        public:
            CholeskySampler(const OptimiserParameters& params)
                : batchSize(params.batchSize)
                , generation(params.initial.generation)
                , upperBound(convert<double>(params.upperBound))
                , lowerBound(convert<double>(params.lowerBound))
                , samples() {}

            void clear() {
                generation = -1;
            }

            arma::mat getSamples(const OptimiserEstimate& bestParams, uint64_t numSamples) {
                if (bestParams.generation != generation || sampleCount + numSamples > batchSize) {
                    arma::mat projection = arma::chol(convert<double>(bestParams.covariance));
                    samples = (arma::randn(convert<double>(bestParams.estimate).n_elem, batchSize) * projection).t();
                    samples.each_col() += convert<double>(bestParams.estimate);

                    // out of bounds check
                    if (lowerBound.n_elem > 0 and upperBound.n_elem > 0) {
                        arma::uvec outOfBounds = arma::sum(samples > arma::repmat(upperBound, samples.n_cols, 1), 1);
                        outOfBounds += arma::sum(samples < arma::repmat(lowerBound, samples.n_cols, 1), 1);
                        samples = samples.rows(arma::find(outOfBounds == 0));

                        while (samples.n_rows < batchSize) {
                            arma::mat samples2 = arma::randn(convert<double>(bestParams.estimate).n_elem, batchSize);
                            samples2 =
                                (arma::randn(convert<double>(bestParams.estimate).n_elem, batchSize) * projection).t();
                            samples2.each_col() += convert<double>(bestParams.estimate);

                            outOfBounds = arma::sum(samples2 > arma::repmat(upperBound, samples2.n_cols, 1), 1);
                            outOfBounds += arma::sum(samples2 < arma::repmat(lowerBound, samples2.n_cols, 1), 1);
                            samples2 = samples2.rows(arma::find(outOfBounds == 0));

                            samples = join_rows(samples, samples2);
                        }

                        if (samples.n_cols >= batchSize) {
                            samples = samples.rows(0, batchSize - 1);
                        }
                    }

                    // reset required variables
                    sampleCount = 0;
                }
                sampleCount += numSamples;
                return samples.cols(sampleCount - numSamples, sampleCount - 1);
            }
        };
    }  // namespace optimisation
}  // namespace math
}  // namespace utility

#endif  // UTILITY_MATH_OPTIMISATION_CHOLESKYSAMPLER_H
