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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_LOCALISATION_BALLMODEL_H
#define MODULES_LOCALISATION_BALLMODEL_H

#include "message/localisation/FieldObject.h"

namespace module {
namespace localisation {
namespace ball {
    // Number of dimensions
    // The state consists of 3 components:
    //    1. The x position in robot space
    //    2. The y position in robot space
    //    3. The x component of velocity
    //    4. The y component of velocity
    enum BallModelStateComponents {
        kX = 0,
        kY = 1,
        kVx = 2,
        kVy = 3,
    };

    class BallModel {
    public:
        static constexpr size_t size = 4;

        BallModel() : cfg_() {}

        // Eigen::Matrix<double, size, 1> timeUpdate(
        //     const Eigen::Matrix<double, size, 1>& state, double deltaT,
        //     const message::localisation::FakeOdometry& odom);

        Eigen::Matrix<double, size, 1> timeUpdate(
            const Eigen::Matrix<double, size, 1>& state, double deltaT);

        Eigen::VectorXd predictedObservation(const Eigen::Matrix<double, size, 1>& state, double ballAngle);

        Eigen::VectorXd observationDifference(const arma::vec& a, const arma::vec& b);

        Eigen::Matrix<double, size, 1> limitState(const Eigen::Matrix<double, size, 1>& state);

        Eigen::Matrix<double, size, size> processNoise();

        Eigen::Matrix<double, BallModel::size, 1> ApplyVelocity(
            const Eigen::Matrix<double, BallModel::size, 1>& state,
            double deltaT);

        struct Config {
            double ballDragCoefficient = 0.1;
            double processNoisePositionFactor = 1e-3;
            double processNoiseVelocityFactor = 1e-3;
            double ballHeight = 0.05;
        } cfg_;
    };
}
}
}
#endif
