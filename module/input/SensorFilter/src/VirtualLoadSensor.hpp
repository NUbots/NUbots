/*
 * MIT License
 *
 * Copyright (c) 2016 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MODULES_INPUT_VIRTUALLOADSENSOR_HPP
#define MODULES_INPUT_VIRTUALLOADSENSOR_HPP

#include <Eigen/Core>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    template <typename Scalar>
    class VirtualLoadSensor {
    public:
        VirtualLoadSensor() = default;
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

        [[nodiscard]] std::array<bool, 2> updateFeet(const ::message::input::Sensors& sensors) {

            // Build our input data
            int index = 0;
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1> logits((servos.size() * fields.size()) + (accelerometer ? 3 : 0)
                                                            + (gyroscope ? 3 : 0));
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
            const auto LAYERS_SIZE = layers.size();
            for (size_t i = 0; i < LAYERS_SIZE; ++i) {

                // Weights and bias
                logits = logits.transpose() * layers[i].first + layers[i].second.transpose();

                // RELU except last layer
                if (i + 1 < layers.size()) {
                    logits = (logits.array() > Scalar(0.0)).select(logits, Scalar(0.0));
                }
                // Sigmoid final layer
                else {
                    logits =
                        logits.unaryExpr([](const Scalar& value) { return Scalar(1.0 / (1.0 + std::exp(-value))); });
                }
            }

            // Do the bayes update (1D kalman filter thing)
            Scalar k = current_noise / (current_noise + noise_factor);
            state += k * (logits - state);
            current_noise *= Scalar(1.0) - k;
            current_noise += Scalar(1.0);

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

        Eigen::Matrix<Scalar, 2, 1> state = Eigen::Matrix<Scalar, 2, 1>::Zero();


        enum Field { POSITION, VELOCITY, LOAD };

        Scalar noise_factor          = Scalar(0.0);
        Scalar current_noise         = Scalar(0.0);
        Scalar certainty_threshold   = Scalar(0.0);
        Scalar uncertainty_threshold = Scalar(0.0);

        std::vector<::utility::input::ServoID> servos{};
        std::vector<Field> fields{};

        bool accelerometer = false;
        bool gyroscope     = false;

        std::vector<
            std::pair<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>>
            layers{};

        std::array<bool, 2> output_state = {false, false};
    };
}  // namespace module::input

#endif  // MODULES_INPUT_VIRTUALLOADSENSOR_HPP
