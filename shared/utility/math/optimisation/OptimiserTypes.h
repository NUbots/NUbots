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

#ifndef UTILITY_MATH_OPTIMISATION_OPTIMISERTYPES_H
#define UTILITY_MATH_OPTIMISATION_OPTIMISERTYPES_H

#include <armadillo>
#include <cmath>

namespace utility {
    namespace math {
        namespace optimisation {
            struct OptimiserEstimate {
                int generation;
                arma::vec estimate;
                arma::mat covariance;
            };

            struct OptimiserParameters {
                OptimiserEstimate initial;
                arma::vec upperBound;
                arma::vec lowerBound;
                uint batchSize;
                //TODO: add extra params!
            };
        }
    }
}

#endif // UTILITY_MATH_OPTIMISER_H
