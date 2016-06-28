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
#include "message/support/FieldDescription.h"
#include "message/localisation/FieldObject.h"
#include "utility/math/matrix/Rotation2D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/support/yaml_armadillo.h"
 #include "utility/nubugger/NUhelpers.h"

namespace module {
namespace localisation {

    using message::support::Configuration;
    using message::input::Sensors;
    using message::vision::Goal;
    using message::support::FieldDescription;
    using utility::math::filter::MMUKF;
    using utility::math::filter::UKF;
    using utility::nubugger::graph;

    RobotFieldLocalisation::RobotFieldLocalisation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("RobotFieldLocalisation.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file RobotFieldLocalisation.yaml

            if (filter.filters.empty()) {
                filter.filters.push_back(
                        MMUKF<FieldModel>::Filter{
                            1.0, 
                            UKF<FieldModel>(
                                config["initial_mean"].as<arma::vec3>()
                                , arma::diagmat(config["initial_covariance"].as<arma::vec3>())
                                )
                            }
                        );
            }

            for (auto& f : filter.filters) {
                f.filter.model.processNoiseDiagonal = config["process_noise"].as<arma::vec3>();
            }

        });


        on<Trigger<Sensors>, Sync<RobotFieldLocalisation>, Single>().then("Localisation Field Space", [this] (const Sensors& sensors) {

            // Use the current world to field state we are holding to modify sensors.world and emit that
            utility::math::matrix::Transform3D Htw = sensors.world;

            //create the world-field transform
            arma::vec3 rFWf;
            rFWf[2] = 0.0;
            rFWf.rows(0,1) = filter.get().rows(0,1);
            //XXX: check correctness
            utility::math::matrix::Transform3D Hwf = utility::math::matrix::Transform3D::createRotationZ(filter.get()[2]) 
                                                   + utility::math::matrix::Transform3D::createTranslation(rFWf);
            Hwf(3,3) = 1.0;

            //extract the 2D yaw from the field-torso transform
            utility::math::matrix::Transform3D Htf = utility::math::matrix::Transform3D(Htw * Hwf);
            double yaw = utility::math::matrix::Rotation3D(Htf.rotation()).yaw();

            //make a localisation object
            message::localisation::Self robot;

            //set position, covariance, and rotation
            robot.position = Htf.translation().rows(0,1);
            robot.robot_to_world_rotation = utility::math::matrix::Rotation2D::createRotation(yaw);
            robot.position_cov = robot.robot_to_world_rotation * filter.getCovariance().submat(0,0,1,1);
            robot.heading = robot.robot_to_world_rotation.row(0).t();
            emit(std::make_unique<std::vector<message::localisation::Self>>(std::vector<message::localisation::Self>(1,robot)));
        });

        on<Every<30, Per<std::chrono::seconds>>, Sync<RobotFieldLocalisation>>().then("Robot Localisation Time Update", [this] {

            // Do a time update on the models
            filter.timeUpdate(1.0/30.0);
        });

        on<Trigger<std::vector<Goal>>, With<FieldDescription>, Sync<RobotFieldLocalisation>>().then("Localisation Goal Update", [this] (const std::vector<Goal>& goals, const FieldDescription& field) {

            // If we have two goals that are left/right
            if(goals.size() == 2) {

                // Build our measurement list
                std::vector<double> measurement;

                // Build our measurement types list
                std::vector<std::tuple<Goal::Team, Goal::Side, Goal::MeasurementType>> measurementTypesOwn;
                std::vector<std::tuple<Goal::Team, Goal::Side, Goal::MeasurementType>> measurementTypesOpponent;

                for (auto& goal : goals) {
                    for (auto& m : goal.measurements) {
                        // Insert the measurements into our vector
                        measurement.push_back(m.second[0]);
                        measurement.push_back(m.second[1]);
                        measurement.push_back(m.second[2]);

                        // Insert the measurement type into our measurement type vector
                        measurementTypesOwn.push_back(std::make_tuple(Goal::Team::OWN, goal.side, m.first));
                        measurementTypesOpponent.push_back(std::make_tuple(Goal::Team::OPPONENT, goal.side, m.first));
                    }
                }

                arma::vec armaMeas = measurement;

                // Apply our multiple measurement updates
                filter.measurementUpdate({
                      std::make_tuple(armaMeas, arma::mat(arma::eye(armaMeas.n_elem, armaMeas.n_elem) * 1e-4), measurementTypesOwn,      field, *goals[0].sensors, FieldModel::MeasurementType::GOAL())
                    , std::make_tuple(armaMeas, arma::mat(arma::eye(armaMeas.n_elem, armaMeas.n_elem) * 1e-4), measurementTypesOpponent, field, *goals[0].sensors, FieldModel::MeasurementType::GOAL())
                });
            }

            // We have one ambigous goal
            else if(goals.size() == 1) {

                // Build our measurement list
                std::vector<double> measurement;

                // Build our measurement types list
                std::vector<std::tuple<Goal::Team, Goal::Side, Goal::MeasurementType>> measurementTypes[4];

                for (auto& goal : goals) {
                    for (auto& m : goal.measurements) {
                        // Insert the measurements into our vector
                        measurement.push_back(m.second[0]);
                        measurement.push_back(m.second[1]);
                        measurement.push_back(m.second[2]);

                        // Insert the measurement type into our measurement type vector
                        measurementTypes[0].push_back(std::make_tuple(Goal::Team::OWN, Goal::Side::LEFT, m.first));
                        measurementTypes[1].push_back(std::make_tuple(Goal::Team::OWN, Goal::Side::RIGHT, m.first));
                        measurementTypes[2].push_back(std::make_tuple(Goal::Team::OPPONENT, Goal::Side::LEFT, m.first));
                        measurementTypes[3].push_back(std::make_tuple(Goal::Team::OPPONENT, Goal::Side::RIGHT, m.first));
                    }
                }

                arma::vec armaMeas = measurement;

                // Apply our multiple measurement updates

                filter.measurementUpdate({
                      std::make_tuple(armaMeas, arma::mat(arma::eye(armaMeas.n_elem, armaMeas.n_elem) * 1e-4), measurementTypes[0], field, *goals[0].sensors, FieldModel::MeasurementType::GOAL())
                    , std::make_tuple(armaMeas, arma::mat(arma::eye(armaMeas.n_elem, armaMeas.n_elem) * 1e-4), measurementTypes[1], field, *goals[0].sensors, FieldModel::MeasurementType::GOAL())
                    , std::make_tuple(armaMeas, arma::mat(arma::eye(armaMeas.n_elem, armaMeas.n_elem) * 1e-4), measurementTypes[2], field, *goals[0].sensors, FieldModel::MeasurementType::GOAL())
                    , std::make_tuple(armaMeas, arma::mat(arma::eye(armaMeas.n_elem, armaMeas.n_elem) * 1e-4), measurementTypes[3], field, *goals[0].sensors, FieldModel::MeasurementType::GOAL())
                });
            }
        });
    }
}
}
