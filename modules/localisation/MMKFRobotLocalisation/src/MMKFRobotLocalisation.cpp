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
#include "utility/nubugger/NUgraph.h"
#include "utility/localisation/FieldDescription.h"
#include "utility/localisation/LocalisationFieldObject.h"
#include "messages/vision/VisionObjects.h"
#include "messages/input/Sensors.h"
#include "messages/support/Configuration.h"
#include "messages/localisation/FieldObject.h"
#include "MMKFRobotLocalisationEngine.h"
#include "RobotModel.h"

using utility::nubugger::graph;
using messages::support::Configuration;
using utility::localisation::LocalisationFieldObject;
using messages::localisation::FakeOdometry;
using modules::localisation::MultiModalRobotModelConfig;
using utility::localisation::FieldDescriptionConfig;
using messages::input::Sensors;

namespace modules {
namespace localisation {
    MMKFRobotLocalisation::MMKFRobotLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : engine_(std::make_unique<MMKFRobotLocalisationEngine>()),
          Reactor(std::move(environment)) {

        on<Trigger<Configuration<MultiModalRobotModelConfig>>>(
            "MultiModalRobotModelConfig Update",
            [this](const Configuration<MultiModalRobotModelConfig>& config) {
            engine_->UpdateConfiguration(config);
            NUClear::log("Localisation config finished successfully!");
        });

        on<Trigger<Configuration<MMKFRobotLocalisationEngineConfig>>>(
            "MMKFRobotLocalisationEngineConfig Update",
            [this](const Configuration<MMKFRobotLocalisationEngineConfig>& config) {
            engine_->UpdateConfiguration(config);
        });

        on<Trigger<Configuration<FieldDescriptionConfig>>>(
            "FieldDescriptionConfig Update",
            [this](const Configuration<FieldDescriptionConfig>& config) {
            auto fd = std::make_shared<utility::localisation::FieldDescription>(config);
            engine_->set_field_description(fd);
            NUClear::log("Localisation config finished successfully!");
        });

        // Emit to NUbugger
        on<Trigger<Every<100, std::chrono::milliseconds>>,
           Options<Sync<MMKFRobotLocalisation>>
           >("NUbugger Output", [this](const time_t&) {
            auto robot_msg = std::make_unique<std::vector<messages::localisation::Self>>();

            for (auto& model : engine_->robot_models_.hypotheses()) {
                arma::vec::fixed<localisation::robot::RobotModel::size> model_state = model->GetEstimate();
                auto model_cov = model->GetCovariance();

                messages::localisation::Self robot_model;
                robot_model.position = model_state.rows(0, 1);
                robot_model.heading = model_state.rows(2, 3);
                robot_model.sr_xx = model_cov(0, 0);
                robot_model.sr_xy = model_cov(0, 1);
                robot_model.sr_yy = model_cov(1, 1);
                robot_msg->push_back(robot_model);
            }
            emit(std::move(robot_msg));
        });

        on<Trigger<FakeOdometry>,
           Options<Sync<MMKFRobotLocalisation>>
          >("MMKFRobotLocalisation Odometry", [this](const FakeOdometry& odom) {
            auto curr_time = NUClear::clock::now();
            engine_->TimeUpdate(curr_time, odom);
        });

        on<Trigger<Sensors>,
           Options<Sync<MMKFRobotLocalisation>>
          >("MMKFRobotLocalisation Odometry", [this](const Sensors& sensors) {
            auto curr_time = NUClear::clock::now();
            engine_->TimeUpdate(curr_time, sensors.odometry);
        });

        on<Trigger<std::vector<messages::vision::Goal>>,
           Options<Sync<MMKFRobotLocalisation>>
          >("MMKFRobotLocalisation Step",
            [this](const std::vector<messages::vision::Goal>& goals) {
            auto curr_time = NUClear::clock::now();
            engine_->TimeUpdate(curr_time);
            engine_->ProcessObjects(goals);
        });
    }
}
}
