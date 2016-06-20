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

            DarwinVirtualLoadSensor::DarwinVirtualLoadSensor(arma::mat hiddenWeights,
                                    arma::vec hiddenBias,
                                    arma::mat outputWeights,
                                    arma::vec outputBias,
                                    double noiseFactor,
                                    double certaintyThreshold,
                                    double uncertaintyThreshold)
                    : hiddenWeights(hiddenWeights)
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

                double linResult =
                std::max(
                    (
                        arma::vec(
                            arma::clamp(
                                hiddenWeights * features + hiddenBias, 0.0, std::numeric_limits<double>::max()
                            )
                        ).t() * outputWeights + outputBias
                    )[0], 0.0
                );

                //do the bayes update (1D kalman filter thing)
                double k = currentNoise / (currentNoise + noiseFactor);
                state += k * (linResult - state);
                currentNoise *= 1.0 - k;
                currentNoise += 1.0;

                if (state >= certaintyThreshold) {
                   outputState = true;
                }
                else if (state < uncertaintyThreshold) {
                    outputState = false;
                }

                return outputState;
            }
        }
    }
}
