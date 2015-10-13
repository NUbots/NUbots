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

#ifndef UTILITY_MATH_OPTIMISER_H
#define UTILITY_MATH_OPTIMISER_H

#include <armadillo>
#include <cmath>

#include "PGAOptimiser.h"
#include "PGPEOptimiser.h"
#include "WMDOptimiser.h"


namespace utility {
    namespace math {
        namespace optimisation {
            template<typename OptMethod, typename SampleMethod>
            class Optimiser {
                /**
                 * 
                 *  This class is a generic container for direct policy
                 *  optimisation methods. It supports splitting of
                 *  optimisation and sampling steps to allow recombining 
                 *  for best results.
                 *
                 * @author Josiah Walker
                 */
                
                private:
                    OptMethod estimator;
                    SampleMethod sampler;
                
                public:
                    
                    Optimiser(const arma::vec& optParams, const arma::vec& sampleParams): estimator(optParams), sampler(sampleParams) {
                        ;
                    }
                    
                    arma::vec updateEstimate(arma::mat samples, arma::vec fitnesses) {
                        /**
                         * Generate a new best-estimate using the parameter samples and fitnesses provided.
                         * Takes a row-wise list of sample parameters, a corresponding vector of fitnesses
                         * 
                         * @param samples - the tested samples in the matrix format returned by getSamples below (one sample per row)
                         * @param fitnesses - a vector of fitnesses corresponding to each sample
                         * 
                         * @returns currentEstimate - an updated best parameter estimate vector to re-sample from
                         *
                         * 
                         * @author Josiah Walker
                         */
                        const auto variances = sampler.getVariances(); //variances are useful to determine noise in estimates
                        sampler.updateParams(samples,fitnesses,estimator.currentEstimate());
                        estimator.updateEstimate(samples,fitnesses,sampler.getVariances());
                        return estimator.currentEstimate();
                    }
                    
                    arma::mat getSamples(const uint& numSamples = 7) {
                        /**
                         * Draw a new set of parameter samples and return them to be evaluated.
                         *
                         * @param numSamples - the number of test parameter sets to generate (this varies, but 5-12 is usually good)
                         * 
                         * @returns newSamples - a numSamples rows by numParams cols matrix of sample parameter sets to evaluate
                         * 
                         * @author Josiah walker
                         */
                        return sampler.sample(estimator.currentEstimate(),numSamples);
                    }
                    
                    void reset() {
                        sampler.reset();
                        estimator.reset();
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
            
            
            //easy typedefs for working with the included algorithms
            typedef PGAOptimiser Optimiser<PGAEstimator,PGASampler>;
            typedef WMDOptimiser Optimiser<WMDEstimator,WMDSampler>;
            typedef PGPEOptimiser Optimiser<PGPEEstimator,PGPESampler>;
        }
    }
}


#endif // UTILITY_MATH_OPTIMISER_H

