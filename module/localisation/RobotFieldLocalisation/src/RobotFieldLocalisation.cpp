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

#include "RobotFieldLocalisation.h"

#include "message/input/Sensors.h"
#include "message/vision/VisionObjects.h"
#include "message/support/Configuration.h"

namespace module {
namespace localisation {

    using message::support::Configuration;
    using message::input::Sensors;
    using message::vision::Goal;

    RobotFieldLocalisation::RobotFieldLocalisation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("RobotFieldLocalisation.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file RobotFieldLocalisation.yaml
        });


        on<Trigger<Sensors>>().then("Localisation Field Space" [this] (const Sensors& sensors) {

            // Use the current world to field state we are holding to modify sensors.world and emit that
        });

        on<Every<30, Per<std::chrono::seconds>, Sync<RobotFieldLocalisation>>().then("Robot Localisation Time Update", [this] {

            // Do a time update on the models
        });

        on<Trigger<std::vector<Goal>>, With<FieldDescription>, Sync<RobotFieldLocalisation>>().then("Localisation Goal Update", [this] (const std::vector<Goal>& goals, const FieldDescription& field) {

            // If we have two goals that are left/right
            if(goals.size() == 2) {

                // Build our measurement list
                std::vector<double> measurement;

                // Build our measurement types list
                std::vector<std::pair<Goal::Side, Goal::MeasurementType> measurementTypesOwn;
                std::vector<std::pair<Goal::Side, Goal::MeasurementType> measurementTypesOpponent;

                for (auto& goal : goals) {
                    for (auto& measurement : goal.measurement) {
                        // Insert the measurements into our vector
                        measurement.push_back(measurement.second[0]);
                        measurement.push_back(measurement.second[1]);
                        measurement.push_back(measurement.second[2]);

                        // Insert the measurement type into our measurement type vector
                        measurementTypesOwn.push_back(std::make_pair(Goal::Team::OWN, goal.side, measurement.first));
                        measurementTypesOpponent.push_back(std::make_pair(Goal::Team::OPPONENT, goal.side, measurement.first));
                    }
                }

                arma::vec armaMeas = measurement;

                // Apply our multiple measurement updates
                filter.measurementUpdate({
                      std::make_tuple(armaMeas, covariance, measurementTypesOwn)
                    , std::make_tuple(armaMeas, covariance, measurementTypesOpponent)
                });
            }

            // We have one ambigous goal
            else if(goals.size() == 1) {

                // Build our measurement list
                std::vector<double> measurement;

                // Build our measurement types list
                std::vector<std::pair<Goal::Side, Goal::MeasurementType> measurementTypes[4];

                for (auto& goal : goals) {
                    for (auto& measurement : goal.measurement) {
                        // Insert the measurements into our vector
                        measurement.push_back(measurement.second[0]);
                        measurement.push_back(measurement.second[1]);
                        measurement.push_back(measurement.second[2]);

                        // Insert the measurement type into our measurement type vector
                        measurementTypes[1].push_back(std::make_pair(Goal::Team::OWN, Goal::Side::LEFT, measurement.first));
                        measurementTypes[2].push_back(std::make_pair(Goal::Team::OWN, Goal::Side::RIGHT, measurement.first));
                        measurementTypes[3].push_back(std::make_pair(Goal::Team::OPPONENT, Goal::Side::LEFT, measurement.first));
                        measurementTypes[4].push_back(std::make_pair(Goal::Team::OPPONENT, Goal::Side::RIGHT, measurement.first));
                    }
                }

                arma::vec armaMeas = measurement;

                // Apply our multiple measurement updates
                filter.measurementUpdate({
                      std::make_tuple(armaMeas, covariance, measurementTypes[0])
                    , std::make_tuple(armaMeas, covariance, measurementTypes[1])
                    , std::make_tuple(armaMeas, covariance, measurementTypes[2])
                    , std::make_tuple(armaMeas, covariance, measurementTypes[3])
                });
            }
        });
    }
}
}
