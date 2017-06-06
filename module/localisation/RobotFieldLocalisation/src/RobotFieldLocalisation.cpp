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

#include "extension/Configuration.h"

#include "message/input/Sensors.h"
#include "message/localisation/ResetRobotHypotheses.h"
#include "message/localisation/FieldObject.h"
#include "message/support/FieldDescription.h"
#include "message/vision/VisionObjects.h"

#include "utility/math/matrix/Rotation2D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/vision.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/eigen.h"
#include "utility/support/yaml_armadillo.h"

namespace module {
namespace localisation {

    using extension::Configuration;
    using message::input::Sensors;
    using message::vision::Goal;
    using GoalSide = message::vision::Goal::Side::Value;
    using GoalTeam = message::vision::Goal::Team::Value;
    using MeasurementType = message::vision::Goal::MeasurementType;
    using message::support::FieldDescription;
    using message::localisation::ResetRobotHypotheses;
    using utility::math::filter::MMUKF;
    using utility::math::filter::UKF;
    using utility::nubugger::graph;
    using utility::math::matrix::Rotation2D;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;

    RobotFieldLocalisation::RobotFieldLocalisation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , filter()
    , defaultMeasurementCovariance(arma::fill::ones)
    , lastUpdateTime(NUClear::clock::now()) {

        on<Configuration>("RobotFieldLocalisation.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file RobotFieldLocalisation.yaml
            defaultMeasurementCovariance = config["measurement_noise"].as<arma::vec3>();
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

            lastUpdateTime = NUClear::clock::now();

        });


        on<Trigger<ResetRobotHypotheses>
         , With<Sensors>
         , Sync<RobotFieldLocalisation>>()
         .then("Localisation ResetRobotHypotheses", [this](
                const ResetRobotHypotheses& reset
              , const Sensors& sensors
            ) {

            //Reset the filter to use the new robot hypotheses
            filter.filters.resize(0);

            for (const auto& h : reset.hypotheses) {
                arma::vec3 Tgr = arma::vec3{h.position[0], h.position[1], h.heading};

                if (!h.absoluteYaw) {
                    Tgr[2] -= Rotation3D(Transform3D(convert<double, 4, 4>(sensors.world)).rotation()).yaw();
                }

                arma::mat33 stateCov(arma::fill::eye);
                stateCov.submat(0, 0, 1, 1) = convert<double, 2, 2>(h.position_cov);
                stateCov(2, 2) = h.heading_var;
                filter.filters.emplace_back(0.0, UKF<FieldModel>(Tgr, stateCov));
            }

            filter.timeUpdate(0.0);
        });

        on<Trigger<Sensors>, Sync<RobotFieldLocalisation>, Single>().then("Localisation Field Space", [this] (const Sensors& sensors) {

            // Use the current world to field state we are holding to modify sensors.world and emit that
            utility::math::matrix::Transform3D Htg = convert<double, 4, 4>(sensors.world);

            //this actually gets Field to Torso???
            //g = odometry space
            //r = robot space (on the ground below the robot)
            //t = torso space
            Transform2D Tgr = filter.get();
            utility::math::matrix::Transform3D Hcf = utility::math::vision::getFieldToCam(
                    Tgr,
                    Htg.i()
                );

            //make a localisation object
            message::localisation::Self robot;
            Transform2D currentLocalisation = Hcf.i().projectTo2D(arma::vec3({0,0,1}),arma::vec3({1,0,0}));
            Transform2D currentOdometry = Htg.i().projectTo2D(arma::vec3({0,0,1}),arma::vec3({1,0,0}));
            // std::cerr << "Hcf : " << std::endl << Hcf << std::endl;
            // std::cerr << "currentOdometry : " << std::endl << currentOdometry << std::endl;
            //std::cerr << "internal Localisation state: " << std::endl << Tgr << std::endl;
            //std::cerr << "currentLocalisation: " << std::endl << currentLocalisation << std::endl;
            //set position, covariance, and rotation
            robot.locObject.position      = convert<double, 2>(currentLocalisation.rows(0, 1));
            robot.robot_to_world_rotation = convert<double, 2, 2>(Rotation2D::createRotation(currentLocalisation[2]));
            robot.locObject.position_cov  = robot.robot_to_world_rotation * convert<double, 2, 2>(filter.getCovariance().submat(0, 0, 1, 1));
            robot.heading                 = robot.robot_to_world_rotation.row(0).transpose();
            emit(std::make_unique<std::vector<message::localisation::Self>>(std::vector<message::localisation::Self>(1, robot)));
        });

        on<Trigger<std::vector<Goal>>, With<FieldDescription>, Sync<RobotFieldLocalisation>>().then("Localisation Goal Update", [this] (const std::vector<Goal>& goals, const FieldDescription& field) {
            auto now = NUClear::clock::now();
            float deltaT = std::chrono::duration_cast<std::chrono::nanoseconds>(now - lastUpdateTime).count() * (1 / std::nano::den);
            lastUpdateTime = now;
            filter.timeUpdate(std::fmin(deltaT,0.1));
            // If we have two goals that are left/right
            if(goals.size() == 2) {

                // Build our measurement list
                std::vector<double> measurement;

                // Build our measurement types list
                std::vector<std::tuple<GoalTeam, GoalSide, MeasurementType>> measurementTypesOwn;
                std::vector<std::tuple<GoalTeam, GoalSide, MeasurementType>> measurementTypesOpponent;

                for (auto& goal : goals) {
                    for (auto& m : goal.measurement) {
                        // Insert the measurements into our vector
                        measurement.push_back(m.position[0]);
                        measurement.push_back(m.position[1]);
                        measurement.push_back(m.position[2]);

                        // Insert the measurement type into our measurement type vector
                        measurementTypesOwn.push_back(std::make_tuple(GoalTeam::OWN, goal.side, m.type));
                        measurementTypesOpponent.push_back(std::make_tuple(GoalTeam::OPPONENT, goal.side, m.type));
                    }
                }

                arma::vec armaMeas = measurement;

                // Apply our multiple measurement updates
                arma::mat covmat = arma::diagmat(
                                    arma::repmat(
                                        arma::mat(defaultMeasurementCovariance),
                                        armaMeas.n_elem/defaultMeasurementCovariance.n_elem,
                                        1)
                                    );
                filter.measurementUpdate({
                      std::make_tuple(armaMeas, covmat, measurementTypesOwn,      field, *goals[0].visObject.sensors, FieldModel::MeasurementType::GOAL())
                    , std::make_tuple(armaMeas, covmat, measurementTypesOpponent, field, *goals[0].visObject.sensors, FieldModel::MeasurementType::GOAL())
                });
            }

            // We have one ambigous goal
            else if(goals.size() == 1) {

                // Build our measurement list
                std::vector<double> measurement;
                measurement.reserve(3*goals.size());

                // Build our measurement types list
                std::vector<std::tuple<GoalTeam, GoalSide, MeasurementType>> measurementTypes[4];

                for (auto& goal : goals) {
                    for (auto& m : goal.measurement) {
                        // Insert the measurements into our vector
                        measurement.push_back(m.position[0]);
                        measurement.push_back(m.position[1]);
                        measurement.push_back(m.position[2]);

                        // Insert the measurement type into our measurement type vector
                        measurementTypes[0].push_back(std::make_tuple(GoalTeam::OWN, GoalSide::LEFT, m.type));
                        measurementTypes[1].push_back(std::make_tuple(GoalTeam::OWN, GoalSide::RIGHT, m.type));
                        measurementTypes[2].push_back(std::make_tuple(GoalTeam::OPPONENT, GoalSide::LEFT, m.type));
                        measurementTypes[3].push_back(std::make_tuple(GoalTeam::OPPONENT, GoalSide::RIGHT, m.type));
                    }
                }

                arma::vec armaMeas = measurement;

                // Apply our multiple measurement updates
                arma::mat covmat = arma::diagmat(
                                    arma::repmat(
                                        arma::mat(defaultMeasurementCovariance),
                                        armaMeas.n_elem/defaultMeasurementCovariance.n_elem,
                                        1)
                                    );
                filter.measurementUpdate({
                      std::make_tuple(armaMeas, covmat, measurementTypes[0], field, *goals[0].visObject.sensors, FieldModel::MeasurementType::GOAL())
                    , std::make_tuple(armaMeas, covmat, measurementTypes[1], field, *goals[0].visObject.sensors, FieldModel::MeasurementType::GOAL())
                    , std::make_tuple(armaMeas, covmat, measurementTypes[2], field, *goals[0].visObject.sensors, FieldModel::MeasurementType::GOAL())
                    , std::make_tuple(armaMeas, covmat, measurementTypes[3], field, *goals[0].visObject.sensors, FieldModel::MeasurementType::GOAL())
                });
            }
        });
    }
}
}
