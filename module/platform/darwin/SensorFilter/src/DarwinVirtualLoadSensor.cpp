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

namespace module {
    namespace platform {
        namespace darwin {
            DarwinVirtualLoadSensor::DarwinVirtualLoadSensor() {

            }

            DarwinVirtualLoadSensor::DarwinVirtualLoadSensor(arma::vec fWeights,
                                    double intercept,
                                    double noiseFactor,
                                    double certaintyThreshold,
                                    double uncertaintyThreshold)
                    : currentNoise(2 * noiseFactor)
                    , noiseFactor(noiseFactor)
                    , intercept(intercept)
                    , certaintyThreshold(certaintyThreshold)
                    , uncertaintyThreshold(uncertaintyThreshold)
                    , featureWeights(featureWeights) {
            }

            bool DarwinVirtualLoadSensor::updateFoot(const arma::vec& features) {

                //do the probability based prediction
                double linResult = arma::dot(features, featureWeights) + intercept;
                arma::vec probs = {linResult,1-linResult};
                probs = arma::exp(probs-arma::max(probs));
                probs /= arma::sum(probs);

                //do the bayes update (1D kalman filter thing)
                double k = currentNoise / (currentNoise + noiseFactor);
                state += k*(probs[0]-state);
                currentNoise *= 1-k;

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
