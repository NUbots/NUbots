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

#ifndef UTILITY_MATH_OPTIMISATION_MIRROREDSAMPLER_H
#define UTILITY_MATH_OPTIMISATION_MIRROREDSAMPLER_H

#include <armadillo>
#include <cmath>

#include "message/support/optimisation/OptimiserTypes.h"

namespace utility {
namespace math {
    namespace optimisation {
        using message::support::optimisation::OptimiserEstimate;
        using message::support::optimisation::OptimiserParameters;


        template <typename OriginalSampler>
        class MirroredSampler {
        private:
            uint64_t batchSize;
            uint64_t sampleCount = 0;
            int64_t generation   = -1;
            arma::mat samples;
            OriginalSampler sampler;

        public:
            MirroredSampler(const OptimiserParameters& params)
                : batchSize(params.batchSize), generation(params.initial.generation) {
                auto params2      = params;
                params2.batchSize = (params.batchSize + 1) / 2;
                sampler(params2);
            }

            void clear() {
                generation = -1;
            }

            arma::mat getSamples(OptimiserEstimate& bestParams, uint64_t numSamples) {
                // note: bestParams.covariance is possibly mutable in this step, do not const or copy it!
                if (bestParams.generation != generation || sampleCount + numSamples > batchSize) {
                    samples = sampler.getSamples(bestParams, (batchSize + 1) / 2);
                    samples = join_rows(samples, -samples);

                    // XXX: rearrange samples to be better for robot rollouts!

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

#endif  // UTILITY_MATH_OPTIMISATION_MIRROREDSAMPLER_H
