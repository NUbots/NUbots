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

#include <Eigen/Core>

#include "extension/Configuration.h"
#include "message/input/Sensors.h"
#include "utility/input/ServoID.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace platform {
    namespace darwin {

        template <typename Scalar>
        class VirtualLoadSensor {
        public:
            VirtualLoadSensor()
                : noise_factor(0.0f), current_noise(0.0f), certainty_threshold(0.0f), uncertainty_threshold(0.0f) {}
            VirtualLoadSensor(const ::extension::Configuration& config) {
                // Bayes settings
                noise_factor          = config["filter"]["noise_factor"].as<Scalar>();
                certainty_threshold   = config["filter"]["certainty_threshold"].as<Scalar>();
                uncertainty_threshold = config["filter"]["uncertainty_threshold"].as<Scalar>();


                // Add the servos
                for (const auto& field : config["network"]["input"]["servos"].config) {
                    servos.emplace_back("R_" + field.as<std::string>());
                    servos.emplace_back("L_" + field.as<std::string>());
                }

                // Add the fields
                for (const auto& field : config["network"]["input"]["fields"].config) {
                    if (field.as<std::string>() == "POSITION") {
                        fields.emplace_back(POSITION);
                    }
                    else if (field.as<std::string>() == "LOAD") {
                        fields.emplace_back(LOAD);
                    }
                    else if (field.as<std::string>() == "VELOCITY") {
                        fields.emplace_back(VELOCITY);
                    }
                }

                accelerometer = config["network"]["input"]["accelerometer"].as<bool>();
                gyroscope     = config["network"]["input"]["gyroscope"].as<bool>();

                for (const auto& layer : config["network"]["layers"].config) {

                    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> weights =
                        layer["weights"].as<utility::support::Expression>();
                    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> bias = layer["biases"].as<utility::support::Expression>();

                    layers.emplace_back(weights, bias);
                }
            }

            std::array<bool, 2> updateFeet(const ::message::input::Sensors& sensors) {

                // Build our input data
                int index = 0;
                Eigen::Matrix<Scalar, Eigen::Dynamic, 1> logits((servos.size() * fields.size())
                                                                + (accelerometer ? 3 : 0) + (gyroscope ? 3 : 0));
                for (const auto& servo_id : servos) {
                    const auto& s = sensors.servo[servo_id];
                    for (const auto& f : fields) {
                        switch (f) {
                            case POSITION: logits[index++] = s.present_position; break;
                            case LOAD: logits[index++] = s.load; break;
                            case VELOCITY: logits[index++] = s.present_velocity; break;
                        }
                    }
                }
                if (accelerometer) {
                    logits[index++] = sensors.accelerometer.x();
                    logits[index++] = sensors.accelerometer.y();
                    logits[index++] = sensors.accelerometer.z();
                }
                if (gyroscope) {
                    logits[index++] = sensors.gyroscope.x();
                    logits[index++] = sensors.gyroscope.y();
                    logits[index++] = sensors.gyroscope.z();
                }

                // Run the neural network
                for (size_t i = 0; i < layers.size(); ++i) {

                    // Weights and bias
                    logits = logits.transpose() * layers[i].first + layers[i].second.transpose();

                    // RELU except last layer
                    if (i + 1 < layers.size()) {
                        logits = (logits.array() > 0.0f).select(logits, 0.0f);
                    }
                    // Sigmoid final layer
                    else {
                        logits = logits.unaryExpr(
                            [](const Scalar& value) { return Scalar(1.0 / (1.0 + std::exp(-value))); });
                    }
                }

                // Do the bayes update (1D kalman filter thing)
                Scalar k = current_noise / (current_noise + noise_factor);
                state += k * (logits - state);
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

            Eigen::Matrix<Scalar, 2, 1, Eigen::DontAlign> state;

        public:
            enum Field { POSITION, VELOCITY, LOAD };

            Scalar noise_factor;
            Scalar current_noise;
            Scalar certainty_threshold;
            Scalar uncertainty_threshold;

            std::vector<::utility::input::ServoID> servos;
            std::vector<Field> fields;

            bool accelerometer;
            bool gyroscope;

            std::vector<std::pair<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>,
                                  Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>>
                layers;

            std::array<bool, 2> output_state;
        };
    }  // namespace darwin
}  // namespace platform
}  // namespace module

#endif
