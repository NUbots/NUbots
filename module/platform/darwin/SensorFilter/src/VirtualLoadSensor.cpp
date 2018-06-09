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

        Eigen::Matrix<float, 1, 4> VirtualLoadSensor::softmax(const Eigen::Matrix<float, 1, 4>& x) {
            auto expx = x.array().exp().matrix();
            return expx / expx.sum();
        }

        Eigen::Vector2f VirtualLoadSensor::threshold(const Eigen::Matrix<float, 1, 4>& x) {
            return Eigen::Vector2f(x(0) > x(1), x(2) > x(3));
        }

        std::pair<bool, bool> VirtualLoadSensor::updateFeet(const Eigen::Matrix<float, 1, 12>& features) {

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

            auto out = threshold(softmax(((features * W1 + b1).unaryExpr(SELU) * W2 + b2).unaryExpr(SELU) * W3 + b3));

            // Do the bayes update (1D kalman filter thing)
            float k = current_noise / (current_noise + noise_factor);
            state += k * (out - state);
            current_noise *= 1.0f - k;
            current_noise += 1.0f;

            if (state(0) >= certainty_threshold) {
                output_state.first = true;
            }
            else if (state(0) < uncertainty_threshold) {
                output_state.first = false;
            }

            if (state(1) >= certainty_threshold) {
                output_state.second = true;
            }
            else if (state(1) < uncertainty_threshold) {
                output_state.second = false;
            }

            return output_state;
        }
    }  // namespace darwin
}  // namespace platform
}  // namespace module
