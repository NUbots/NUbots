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

#ifndef UTILITY_MATH_OPTIMISATION_PGPE_H
#define UTILITY_MATH_OPTIMISATION_PGPE_H

#include <armadillo>
#include <cmath>

namespace utility {
    namespace math {
        namespace optimisation {
            
            class PGPEEstimator {
                private:
                    arma::vec bestEstimate;
                    double baseline = 0.0;
                    double learningRate = 0.1;
                    bool firstRun = true;
                    arma::vec startPoint;
                public:
                    PGPEEstimator(const arma::vec& params): startPoint(params), bestEstimate(params) {
                        /**
                         * Initialise the estimator with a new starting point in parameter space
                         * 
                         * @param params - the starting optimisation point for the algorithm
                         *
                         * @author Josiah Walker
                         */
                         firstRun = true;
                         //XXX: in the future, set learning rate through params
                    };
                
                    void reset() {
                        bestEstimate = startPoint * 1.0;
                        firstRun = true;
                    }
                    
                    arma::vec currentEstimate() {
                        return bestEstimate;
                    }
                    
                    updateEstimate(const arma::mat& samples, const arma::vec& fitnesses, const arma::vec& variances) {
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
                         
                        if (firstRun) {
                            firstRun = false;
                            baseline = arma::mean(fitnesses);
                        }
                        
                        arma::vec alpha = variances * learnRate;
                        arma::vec update(variances.n_elem,arma::fill::zeros);
                        for(uint64_t i = 0; i < fitnesses.n_elem; ++i) {
                            update += alpha * (fitnesses[i]-baseline) % (samples.row(i).t() - bestEstimate);
                        }
                        
                        baseline = baseline * 0.9 + 0.1*arma::mean(fitnesses);
                        
                        bestEstimate += update;
                    }
            };
            
            
            class PGPESampler {
                private:
                    arma::vec bestEstimate;
                    double baseline = 0.0;
                    double learningRate = 0.2;
                    bool firstRun = true;
                    arma::vec startPoint;
                public:
                    PGPESampler(const arma::vec& params): startPoint(params), bestEstimate(params) {
                        /**
                         * Initialise the estimator with a new starting point in parameter space
                         * 
                         * @param params - the starting optimisation point for the algorithm
                         *
                         * @author Josiah Walker
                         */
                         firstRun = true;
                         //XXX: in the future, set learning rate through params
                    };
                
                    void reset() {
                        bestEstimate = startPoint * 1.0;
                        firstRun = true;
                    }
                    
                    arma::vec getVariances() {
                        return arma::square(bestEstimate);
                    }
                    
                    updateEstimate(const arma::mat& samples, const arma::vec& fitnesses, const arma::vec& currentEstimate) {
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
                         
                        if (firstRun) {
                            firstRun = false;
                            baseline = arma::mean(fitnesses);
                        }
                        
                        arma::vec alpha = variances * learnRate;
                        arma::vec update(variances.n_elem,arma::fill::zeros);
                        for(uint64_t i = 0; i < fitnesses.n_elem; ++i) {
                            update += alpha * (fitnesses[i]-baseline) % (arma::square(samples.row(i).t() - bestEstimate) - variances) / arma::sqrt(variances);
                        }
                        
                        baseline = baseline * 0.9 + 0.1*arma::mean(fitnesses);
                        
                        bestEstimate += update;
                    }
                
                    arma::mat getSamples(const arma::vec& currentEstimate, const size_t& numSamples) {
                        //generate half the samples
                        arma::mat samples = arma::randn<arma::mat>(numSamples/2,bestEstimate.n_elem) 
                                            % arma::repmat(bestEstimate, 1, numSamples/2).t() 
                                            + arma::repmat(currentEstimate, 1, numSamples/2).t();
                        //mirror the samples
                        return arma::join_cols(samples,-samples);
                    }
                        
            };
        }
    }
}


#endif // UTILITY_MATH_COORDINATES_H

