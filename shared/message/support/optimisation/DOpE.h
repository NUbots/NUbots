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
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGE_SUPPORT_OPTIMISATION_DOPE_H
#define MESSAGE_SUPPORT_OPTIMISATION_DOPE_H

#include <string>
#include <armadillo>

#include "utility/math/optimisation/OptimiserTypes.h"

namespace message {
    namespace support {
        namespace optimisation {

            struct RegisterOptimisation {
                std::string group;
                bool network;
                utility::math::optimisation::OptimiserParameters parameters;
            };

            struct RequestParameters {
                std::string group;
                int nSamples;
            };

            struct Parameters {
                std::string group;
                int generation;
                arma::mat samples;
                arma::mat covariance;
            };

        }
    }
}

#endif  // MESSAGE_SUPPORT_OPTIMISATION_DOPE_H
