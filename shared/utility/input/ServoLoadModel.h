/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_INPUT_SERVOLOADMODEL_H
#define MODULES_INPUT_SERVOLOADMODEL_H

#include <nuclear>
#include <chrono>
#include <yaml-cpp/yaml.h>

namespace utility
{
namespace input
{
    class ServoLoadModel
    {
       public:
        static constexpr size_t size = 1;

        ServoLoadModel() {} // empty constructor

        Eigen::Matrix<double, size, 1> timeUpdate(const Eigen::Matrix<double, size, 1>& state, double /*deltaT*/) {
            return state;
        }

        Eigen::Matrix<double, size, 1> predictedObservation(const Eigen::Matrix<double, size, 1>& state) {
            return state;
        }

        Eigen::VectorXd observationDifference(const arma::vec& a, const arma::vec& b) {
            return a - b;
        }

        Eigen::Matrix<double, size, 1> limitState(const Eigen::Matrix<double, size, 1>& state) {
            return state;
        }

        Eigen::Matrix<double, size, size> processNoise() {
            return Eigen::Matrix<double, ServoLoadModel::size, ServoLoadModel::size>::Identity() * 0.001;
        }
    };
}
}

#endif  // MODULES_INPUT_SERVOLOADMODEL_H
