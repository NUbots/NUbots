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
        DarwinVirtualLoadSensor::DarwinVirtualLoadSensor()
            : noiseFactor(0.0)
            , currentNoise(2.0 * noiseFactor)
            , certaintyThreshold(0.0)
            , uncertaintyThreshold(0.0)
            , hiddenWeights()
            , hiddenBias()
            , outputWeights()
            , outputBias() {}

        DarwinVirtualLoadSensor::DarwinVirtualLoadSensor(arma::mat hiddenWeights,
                                                         arma::vec hiddenBias,
                                                         arma::mat outputWeights,
                                                         arma::vec outputBias,
                                                         double noiseFactor,
                                                         double certaintyThreshold,
                                                         double uncertaintyThreshold)
            : noiseFactor(noiseFactor)
            , currentNoise(2.0 * noiseFactor)
            , certaintyThreshold(certaintyThreshold)
            , uncertaintyThreshold(uncertaintyThreshold)
            , hiddenWeights(hiddenWeights)
            , hiddenBias(hiddenBias)
            , outputWeights(outputWeights)
            , outputBias(outputBias) {}

        bool DarwinVirtualLoadSensor::updateFoot(const arma::vec& features) {

            double linResult =
                (arma::vec(arma::clamp(hiddenWeights * features + hiddenBias, 0.0, std::numeric_limits<double>::max()))
                         .t()
                     * outputWeights
                 + outputBias)[0];

            linResult = std::tanh(linResult * 0.5) * 0.5 + 0.5;

            // do the bayes update (1D kalman filter thing)
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
