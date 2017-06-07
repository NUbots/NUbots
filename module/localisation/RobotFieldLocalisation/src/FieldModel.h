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

#ifndef MODULE_LOCALISATION_FIELDMODEL_H
#define MODULE_LOCALISATION_FIELDMODEL_H


#include "message/input/Sensors.h"
#include "message/vision/VisionObjects.h"
#include "message/support/FieldDescription.h"

namespace module {
    namespace localisation {

        class FieldModel {
        public:

            // The indicies for our vector
            static constexpr uint DPX = 0;
            static constexpr uint DPY = 1;
            static constexpr uint DTHETA = 2;

            static constexpr size_t size = 3;

            struct MeasurementType {
                struct GOAL {};
            };

            Eigen::Vector3d processNoiseDiagonal;


            FieldModel() : processNoiseDiagonal(arma::fill::eye) {} // empty constructor

            Eigen::Matrix<double, size, 1> timeUpdate(const Eigen::Matrix<double, size, 1>& state, double deltaT);

            Eigen::VectorXd predictedObservation(const Eigen::Matrix<double, size, 1>& state
                , const std::vector<std::tuple<message::vision::Goal::Team::Value,
                                               message::vision::Goal::Side::Value,
                                               message::vision::Goal::MeasurementType>>& measurements
                , const message::support::FieldDescription& field
                , const message::input::Sensors& sensors
                , const MeasurementType::GOAL&);

            Eigen::VectorXd observationDifference(const arma::vec& a, const arma::vec& b) const;

            Eigen::Matrix<double, size, 1> limitState(const Eigen::Matrix<double, size, 1>& state) const;

            arma::mat::fixed<size, size> processNoise() const;
        };

    }
}
#endif  // MODULE_LOCALISATION_FIELDMODEL_H
