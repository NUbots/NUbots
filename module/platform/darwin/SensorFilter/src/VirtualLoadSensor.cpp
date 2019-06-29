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

#include "VirtualLoadSensor.h"
#include <nuclear>

namespace module {
namespace platform {
    namespace darwin {

        using extension::Configuration;


        VirtualLoadSensor::VirtualLoadSensor(const Configuration& network) {
            noise_factor          = network["noise_factor"].as<float>();
            current_noise         = 2.0f * noise_factor;
            certainty_threshold   = network["certainty_threshold"].as<float>();
            uncertainty_threshold = network["uncertainty_threshold"].as<float>();

            // Initialize network
            for (size_t row = 0; row < network["input_layer"]["weights"].config.size(); row++) {
                for (size_t col = 0; col < network["input_layer"]["weights"][row].config.size(); col++) {
                    W1(row, col) = network["input_layer"]["weights"][row][col].as<float>();
                }
            }
            for (size_t row = 0; row < network["input_layer"]["bias"].config.size(); row++) {
                b1(row) = network["input_layer"]["bias"][row].as<float>();
            }

            for (size_t row = 0; row < network["hidden_layer"]["weights"].config.size(); row++) {
                for (size_t col = 0; col < network["hidden_layer"]["weights"][row].config.size(); col++) {
                    W2(row, col) = network["hidden_layer"]["weights"][row][col].as<float>();
                }
            }
            for (size_t row = 0; row < network["hidden_layer"]["bias"].config.size(); row++) {
                b2(row) = network["hidden_layer"]["bias"][row].as<float>();
            }

            for (size_t row = 0; row < network["output_layer"]["weights"].config.size(); row++) {
                for (size_t col = 0; col < network["output_layer"]["weights"][row].config.size(); col++) {
                    W3(row, col) = network["output_layer"]["weights"][row][col].as<float>();
                }
            }
            for (size_t row = 0; row < network["output_layer"]["bias"].config.size(); row++) {
                b3(row) = network["output_layer"]["bias"][row].as<float>();
            }
        }

        arma::frowvec::fixed<2> VirtualLoadSensor::softmax(const arma::frowvec::fixed<4>& x) {
            arma::frowvec::fixed<4> expx = arma::exp(x);
            return {expx(0) / arma::sum(expx.head(2)), expx(2) / arma::sum(expx.tail(2))};
        }

        std::array<bool, 2> VirtualLoadSensor::updateFeet(const arma::frowvec::fixed<12>& input) {

            auto SELU = [](float x) -> float {
                static constexpr float alpha  = 1.6732632423543772848170429916717;
                static constexpr float lambda = 1.0507009873554804934193349852946;
                static constexpr float la     = lambda * alpha;

                if (x < 0) {
                    return la * std::exp(x) - la;
                }
                else {
                    return lambda * x;
                }
            };

            auto out = softmax(((input * W1 + b1).eval().for_each(SELU) * W2 + b2).eval().for_each(SELU) * W3 + b3);

            // Do the bayes update (1D kalman filter thing)
            float k = current_noise / (current_noise + noise_factor);
            state += k * (out - state);
            current_noise *= 1.0f - k;
            current_noise += 1.0f;

            for (size_t leg = 0; leg < 2; leg++) {
                // We have some certainty in our measurement
                if (state[leg] > certainty_threshold) {
                    output_state[leg] = true;
                }
                if (state[leg] < uncertainty_threshold) {
                    output_state[leg] = false;
                }
            }

            return output_state;
        }
    }  // namespace darwin
}  // namespace platform
}  // namespace module
