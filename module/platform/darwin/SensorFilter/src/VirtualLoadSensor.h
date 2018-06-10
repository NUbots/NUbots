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
#include <array>

#include <extension/Configuration.h>

namespace module {
namespace platform {
    namespace darwin {

        class VirtualLoadSensor {
            // this implements a linear model (trained by logistic regression) with a bayes filter on the output
        private:
            float noise_factor          = 0.0f;
            float current_noise         = 0.0f;
            float certainty_threshold   = 0.0f;
            float uncertainty_threshold = 0.0f;

            Eigen::Matrix<float, 12, 8> W1 = Eigen::Matrix<float, 12, 8>::Zero();
            Eigen::Matrix<float, 8, 1> b1  = Eigen::Matrix<float, 8, 1>::Zero();
            Eigen::Matrix<float, 8, 8> W2  = Eigen::Matrix<float, 8, 8>::Zero();
            Eigen::Matrix<float, 8, 1> b2  = Eigen::Matrix<float, 8, 1>::Zero();
            Eigen::Matrix<float, 8, 4> W3  = Eigen::Matrix<float, 8, 4>::Zero();
            Eigen::Matrix<float, 4, 1> b3  = Eigen::Matrix<float, 4, 1>::Zero();

            Eigen::Matrix<float, 4, 1> state = Eigen::Matrix<float, 4, 1>::Constant(0.5f);
            std::array<bool, 2> output_state = {true, true};

            Eigen::Matrix<float, 4, 1> softmax(const Eigen::Matrix<float, 4, 1>& x);

        public:
            VirtualLoadSensor() {}
            VirtualLoadSensor(const ::extension::Configuration& network);

            std::array<bool, 2> updateFeet(const Eigen::Matrix<float, 12, 1>& features);

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }  // namespace darwin
}  // namespace platform
}  // namespace module

#endif
