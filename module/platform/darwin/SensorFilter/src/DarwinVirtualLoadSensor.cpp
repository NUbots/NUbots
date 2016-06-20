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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "DarwinVirtualLoadSensor.h"
#include <nuclear>

namespace module {
    namespace platform {
        namespace darwin {
            DarwinVirtualLoadSensor::DarwinVirtualLoadSensor() {

            }

            DarwinVirtualLoadSensor::DarwinVirtualLoadSensor(arma::vec hiddenLayer,
                                    arma::vec hiddenBias,
                                    arma::vec outputWeights,
                                    double outputBias,
                                    double intercept,
                                    double noiseFactor,
                                    double certaintyThreshold,
                                    double uncertaintyThreshold)
                    : hiddenLayer(hiddenLayer)
                    , hiddenBias(hiddenBias)
                    , outputWeights(outputWeights)
                    , outputBias(outputBias) 
                    , currentNoise(2.0 * noiseFactor)
                    , noiseFactor(noiseFactor)
                    , intercept(intercept)
                    , certaintyThreshold(certaintyThreshold)
                    , uncertaintyThreshold(uncertaintyThreshold) {
            }

            bool DarwinVirtualLoadSensor::updateFoot(const arma::vec& features) {

                //double linResult = 1.0 / (std::exp(-(arma::dot(features, featureWeights) + intercept)) + 1.0);

                double linResult = std::max(double((hiddenLayer * features + hiddenBias).t() * outputWeights + outputBias), 0.);

                //do the bayes update (1D kalman filter thing)
                double k = currentNoise / (currentNoise + noiseFactor);
                state += k*(linResult-state);
                currentNoise *= 1.0 - k;

                if (state >= certaintyThreshold) {

                   outputState = true;
                } else if (state < uncertaintyThreshold) {

                    outputState = false;
                }
                return outputState;
            }
        }
    }
}
