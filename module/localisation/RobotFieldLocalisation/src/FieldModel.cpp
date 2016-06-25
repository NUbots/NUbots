/*
 * Should produce world to robot coordinate transform
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

#include "FieldModel.h"

namespace module {
    namespace platform {
        namespace darwin {

            arma::vec::fixed<FieldModel::size> FieldModel::limitState(const arma::vec::fixed<size>& state) {
                return state;
            }

            arma::vec::fixed<FieldModel::size> FieldModel::timeUpdate(const arma::vec::fixed<size>& state, double deltaT) {
                return state;
            }

            arma::vec FieldModel::predictedObservation(const arma::vec::fixed<size>& state
                , const std::vector<std::tuple<Goal::Team, Goal::Side, Goal::MeasurementType>& measurements
                , const FieldDescription& field
                , const MeasurementType::GOAL&) {

                std::vector<double> prediction;

                for(auto& type : measurements) {
                    // Switch on Team
                    switch(std::get<0>(type)) {

                        // Switch on Side
                        switch(std::get<1>(type)) {

                            // Switch on normal type
                            switch(std::get<2>) {

                            }
                        }
                    }
                }

                return prediction
            }

            arma::vec FieldModel::observationDifference(const arma::vec& a, const arma::vec& b) {
                return a - b;

            }

            arma::mat::fixed<FieldModel::size, FieldModel::size> FieldModel::processNoise() {
                return arma::diagmat(processNoiseDiagonal);
            }

        }
    }
}
