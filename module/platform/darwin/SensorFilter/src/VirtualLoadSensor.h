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
#include <utility>

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

            Eigen::Matrix<float, 12, 8> W1;
            Eigen::Matrix<float, 1, 8> b1;
            Eigen::Matrix<float, 8, 8> W2;
            Eigen::Matrix<float, 1, 8> b2;
            Eigen::Matrix<float, 8, 4> W3;
            Eigen::Matrix<float, 1, 4> b3;

            Eigen::Vector2f state;
            std::pair<bool, bool> output_state = {true, true};

            Eigen::Matrix<float, 1, 4> softmax(const Eigen::Matrix<float, 1, 4>& x);
            Eigen::Vector2f threshold(const Eigen::Matrix<float, 1, 4>& x);

        public:
            VirtualLoadSensor()
                : noise_factor(0.0f)
                , current_noise(0.0f)
                , certainty_threshold(0.0f)
                , uncertainty_threshold(0.0f)
                , W1()
                , b1()
                , W2()
                , b2()
                , W3()
                , b3()
                , state(0.5f, 0.5f) {}

            VirtualLoadSensor(const ::extension::Configuration& network);

            std::pair<bool, bool> updateFeet(const Eigen::Matrix<float, 1, 12>& features);
        };
    }  // namespace darwin
}  // namespace platform
}  // namespace module

#endif
