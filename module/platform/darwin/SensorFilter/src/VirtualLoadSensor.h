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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_PLATFORM_DARWIN_VIRTUALLOADSENSOR_H
#define MODULES_PLATFORM_DARWIN_VIRTUALLOADSENSOR_H

#include <armadillo>
#include <array>

#include <extension/Configuration.h>

namespace module {
namespace platform {
    namespace darwin {

        class VirtualLoadSensor {
            // this implements a linear model (trained by logistic regression) with a bayes filter on the output
        private:
            float noise_factor;
            float current_noise;
            float certainty_threshold;
            float uncertainty_threshold;

            arma::fmat::fixed<12, 8> W1;
            arma::frowvec::fixed<8> b1;
            arma::fmat::fixed<8, 8> W2;
            arma::frowvec::fixed<8> b2;
            arma::fmat::fixed<8, 4> W3;
            arma::frowvec::fixed<4> b3;

            arma::frowvec::fixed<2> state;
            std::array<bool, 2> output_state;

            arma::frowvec::fixed<2> softmax(const arma::frowvec::fixed<4>& x);

        public:
            VirtualLoadSensor()
                : noise_factor(0.0f)
                , current_noise(0.0f)
                , certainty_threshold(0.0f)
                , uncertainty_threshold(0.0f)
                , W1(arma::fill::zeros)
                , b1(arma::fill::zeros)
                , W2(arma::fill::zeros)
                , b2(arma::fill::zeros)
                , W3(arma::fill::zeros)
                , b3(arma::fill::zeros)
                , state(arma::fill::zeros)
                , output_state({true, true}) {}
            VirtualLoadSensor(const ::extension::Configuration& network);

            std::array<bool, 2> updateFeet(const arma::frowvec::fixed<12>& features);
        };
    }  // namespace darwin
}  // namespace platform
}  // namespace module

#endif
