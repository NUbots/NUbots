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

#include <armadillo>

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

            arma::vec3 processNoiseDiagonal;


            FieldModel() {} // empty constructor

            arma::vec::fixed<size> timeUpdate(const arma::vec::fixed<size>& state, double deltaT);

            arma::vec predictedObservation(const arma::vec::fixed<size>& state
                , const std::vector<std::tuple<message::vision::Goal::Team, message::vision::Goal::Side, message::vision::Goal::MeasurementType>>& measurements
                , const message::support::FieldDescription& field
                , const message::input::Sensors& sensors
                , const MeasurementType::GOAL&);

            arma::vec observationDifference(const arma::vec& a, const arma::vec& b);

            arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);

            arma::mat::fixed<size, size> processNoise();
        };

    }
}
#endif  // MODULE_LOCALISATION_FIELDMODEL_H
