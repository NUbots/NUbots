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

#include "MMKFRobotLocalisation.h"

#include <nuclear>

#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/localisation/LocalisationFieldObject.h"
#include "utility/localisation/transform.h"
#include "message/vision/VisionObjects.h"
#include "message/input/Sensors.h"
#include "message/support/Configuration.h"
#include "message/support/FieldDescription.h"
#include "message/localisation/FieldObject.h"
#include "message/localisation/ResetRobotHypotheses.h"
#include "MMKFRobotLocalisationEngine.h"
#include "RobotModel.h"

using utility::math::matrix::Rotation3D;
using utility::math::angle::bearingToUnitVector;
using utility::nubugger::graph;
using utility::localisation::LocalisationFieldObject;
using message::support::Configuration;
using message::support::FieldDescription;
using message::input::Sensors;
using message::vision::Goal;
using message::localisation::Self;
using message::localisation::ResetRobotHypotheses;
using utility::nubugger::graph;

namespace module {
namespace localisation {

    void MMKFRobotLocalisation::graphMMRMHypotheses(const std::string& descr, MultiModalRobotModel& mmrm) {
        auto& hypotheses = mmrm.hypotheses();

        int model_num = 0;
        for (auto& model : hypotheses) {
            // arma::vec::fixed<localisation::robot::RobotModel::size> model_state = model->GetEstimate();
            auto model_state = model->GetEstimate();
            auto model_cov = model->GetCovariance();

            // Graph hypothesis state and covariance matrix:
            std::stringstream msg_state;
            msg_state << "MMKFRL, " << descr << ": hypothesis state " << model_num;
            emit(graph(msg_state.str(), model_state));
            std::stringstream msg_cov;
            msg_cov << "MMKFRL, " << descr << ": hypothesis cov " << model_num;
            emit(graph(msg_cov.str(), model_cov));
            model_num++;
        }
    }

    MMKFRobotLocalisation::MMKFRobotLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)),
          engine_(std::make_unique<MMKFRobotLocalisationEngine>()) {


        on<Configuration>("MultiModalRobotModel.yaml")
        .then("MultiModalRobotModelConfig Update", [this](const Configuration& config) {

            engine_->UpdateMultiModalRobotModelConfiguration(config);
            NUClear::log("Localisation config finished successfully!");
        });

        on<Configuration>("MMKFRobotLocalisationEngine.yaml")
        .then("MMKFRobotLocalisationEngineConfig Update", [this](const Configuration& config) {
            engine_->UpdateRobotLocalisationEngineConfiguration(config);
        });

        on<Startup, Optional<With<FieldDescription>>>()
        .then("FieldDescription Update", [this](std::shared_ptr<const FieldDescription> desc) {
            if (desc == nullptr) {
                throw std::runtime_error("FieldDescription Update: support::configuration::SoccerConfig module might not be installed");
            }

            auto fd = std::make_shared<FieldDescription>(*desc);
            engine_->set_field_description(fd);
        });

        on<Trigger<ResetRobotHypotheses>
         , With<Sensors>
         , Sync<MMKFRobotLocalisation>
         , Single>()
         .then("Localisation ResetRobotHypotheses", [this](const ResetRobotHypotheses& reset, const Sensors& sensors) {
            engine_->Reset(reset, sensors);

            // auto& hypotheses = engine_->robot_models_.hypotheses();
            // for (auto& model : hypotheses) {
            //     NUClear::log(__FILE__,__LINE__,"Reset",*model);
            // }
        });

        // Emit self
        emit_data_handle = on<Every<100, std::chrono::milliseconds>
         , With<Sensors>
         , Sync<MMKFRobotLocalisation>
         , Single>()
           .then("Localisation NUSight Output", [this](const Sensors& sensors) {

            auto& hypotheses = engine_->robot_models_.hypotheses();
            if (hypotheses.size() == 0) {
                NUClear::log<NUClear::ERROR>("MMKFRobotLocalisation has no robot hypotheses.");
                return;
            }

            auto robots = std::vector<Self>();

            for (auto& model : hypotheses) {
                // arma::vec::fixed<localisation::robot::RobotModel::size> model_state = model->GetEstimate();
                auto model_state = model->GetEstimate();
                auto model_cov = model->GetCovariance();

                Self robot_model;
                robot_model.position = model_state.rows(robot::kX, robot::kY);
                // Rotation3D imuRotation = Rotation3D::createRotationZ(model_state(robot::kImuOffset));
                // arma::vec3 world_heading = imuRotation * arma::mat(sensors.orientation.t()).col(0);
                // robot_model.heading = world_heading.rows(0, 1);
                robot_model.heading = utility::localisation::transform::ImuToWorldHeadingTransform(model_state(robot::kImuOffset), sensors.orientation);
                robot_model.velocity = model_state.rows(robot::kVX, robot::kVY);
                robot_model.position_cov = model_cov.submat(0,0,1,1);
                robot_model.last_measurement_time = last_measurement_time;
                robots.push_back(robot_model);
            }

            graphMMRMHypotheses("out", engine_->robot_models_);

            auto robot_msg = std::make_unique<std::vector<Self>>(robots);
            emit(std::move(robot_msg));
        });

        //Disable until first data
        emit_data_handle.disable();

        on<Trigger<Sensors>, Sync<MMKFRobotLocalisation>, Single>()
        .then("MMKFRobotLocalisation Odometry", [this](const Sensors& sensors) {
            auto curr_time = NUClear::clock::now();

            emit(graph("Odometry Measurement Update", sensors.odometry[0], sensors.odometry[1]));
            // log("Odometry Measurement Update", sensors.odometry.t());
            engine_->TimeUpdate(curr_time, sensors);
            engine_->OdometryMeasurementUpdate(sensors);
        });

        // on<Every<100, Per<std::chrono::seconds>>,
        //    With<Sensors>,
        //    Sync<MMKFRobotLocalisation>>
        //    .then("MMKFRobotLocalisation Time", [this] (const Sensors& sensors) {
        //     auto curr_time = NUClear::clock::now();
        //     engine_->TimeUpdate(curr_time, sensors);
        // });

        on<Trigger<std::vector<message::vision::Goal>>
         , With<Sensors>
         , Sync<MMKFRobotLocalisation>
         , Single>()
         .then("MMKFRobotLocalisation Measurement Step",
            [this](const std::vector<message::vision::Goal>& goals, const Sensors& sensors) {

            //Is this check necessary?
            if(!emit_data_handle.enabled()){
                //Activate when data received
                emit_data_handle.enable();
            }
            // Ignore empty vectors of goals.
            if (goals.size() == 0){
                return;
            }

            // Ignore measurements when both of the robots feet are off the ground.
            if (!goals[0].sensors->leftFootDown && !goals[0].sensors->rightFootDown) {
                return;
            }

            //DEBUG
            for (auto& goal : goals) {
                // std::cout << "  side:";
                // std::cout << ((goal.side == Goal::Side::LEFT) ? "LEFT" :
                //               (goal.side == Goal::Side::RIGHT) ? "RIGHT" : "UNKNOWN")
                //           << std::endl;

                for(uint i = 0; i < goal.measurements.size(); ++i) {
                    std::stringstream msg;
                    msg << ((goal.side == Goal::Side::LEFT) ? "L " :
                           (goal.side == Goal::Side::RIGHT) ? "R" : "U") <<
                           ((goal.team == Goal::Team::OWN) ? "OWN Goal Pos" :
                           (goal.team == Goal::Team::OPPONENT) ? "OPP Goal Pos" : "UGoal Pos") <<
                           " " << i;
                    emit(graph(msg.str(), goal.measurements[i].position[0], goal.measurements[i].position[1], goal.measurements[i].position[2]));
                    // std::cout << "  measurement: " << num++ << std::endl;
                    // std::cout << "    error:" << measurement.error << std::endl;
                }
                //std::cout << "    position:" << goal.measurements[0].position.t() << std::endl;
            }

            auto curr_time = NUClear::clock::now();
            last_measurement_time = curr_time;

            engine_->TimeUpdate(curr_time, sensors);
            engine_->ProcessObjects(goals);

            graphMMRMHypotheses("update", engine_->robot_models_);
        });
    }
}
}
