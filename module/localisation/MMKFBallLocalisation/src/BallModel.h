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

#ifndef MODULES_LOCALISATION_NEWBALLMODEL_H
#define MODULES_LOCALISATION_NEWBALLMODEL_H

#include <armadillo>
#include "message/localisation/FieldObject.h"

namespace module {
namespace localisation {
namespace mmball {
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
        
        arma::mat processNoiseMatrix;
        double ballDragCoefficient;


        BallModel() {} // empty constructor

        // arma::vec::fixed<size> timeUpdate(
        //     const arma::vec::fixed<size>& state, double deltaT,
        //     const message::localisation::FakeOdometry& odom);

        arma::vec::fixed<size> timeUpdate(
            const arma::vec::fixed<size>& state, double deltaT);

        arma::vec predictedObservation(const arma::vec::fixed<size>& state, double ballAngle);

        arma::vec observationDifference(const arma::vec& a, const arma::vec& b);

        arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);

        arma::mat::fixed<size, size> processNoise();

        arma::vec::fixed<BallModel::size> ApplyVelocity(
            const arma::vec::fixed<BallModel::size>& state,
            double deltaT);
    };
}
}
}
#endif
