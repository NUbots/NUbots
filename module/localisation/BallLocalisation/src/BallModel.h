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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_LOCALISATION_BALLMODEL_H
#define MODULE_LOCALISATION_BALLMODEL_H


#include "message/input/Sensors.h"
#include "message/support/FieldDescription.h"

namespace module {
    namespace localisation {

        class BallModel {
        public:

            // The indicies for our vector
            static constexpr uint PX = 0;
            static constexpr uint PY = 1;

            static constexpr size_t size = 2;

            struct MeasurementType {
                struct BALL {};
            };

            Eigen::Vector2d processNoiseDiagonal;


            BallModel() : processNoiseDiagonal(arma::fill::eye) {} // empty constructor

            Eigen::Matrix<double, size, 1> timeUpdate(const Eigen::Matrix<double, size, 1>& state, double deltaT);

            Eigen::Vector3d predictedObservation(const Eigen::Matrix<double, size, 1>& state
                , const message::support::FieldDescription& field
                , const message::input::Sensors& sensors
                , const MeasurementType::BALL&) const;

            Eigen::VectorXd observationDifference(const arma::vec& measurement
                , const Eigen::Vector3d& rBCc
                , const message::support::FieldDescription& field
                , const message::input::Sensors& sensors
                , const MeasurementType::BALL&) const;

            Eigen::Matrix<double, size, 1> limitState(const Eigen::Matrix<double, size, 1>& state) const;

            arma::mat::fixed<size, size> processNoise() const;
        };

    }
}
#endif  // MODULE_LOCALISATION_BALLMODEL_H
