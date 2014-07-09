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
#include "utility/localisation/LocalisationFieldObject.h"
#include "messages/vision/VisionObjects.h"
#include "messages/input/Sensors.h"
#include "messages/support/Configuration.h"
#include "messages/support/FieldDescription.h"
#include "messages/localisation/FieldObject.h"
#include "MMKFRobotLocalisationEngine.h"
#include "RobotModel.h"

using utility::math::angle::bearingToUnitVector;
using utility::nubugger::graph;
using utility::localisation::LocalisationFieldObject;
using messages::support::Configuration;
using messages::support::FieldDescription;
using messages::localisation::FakeOdometry;
using messages::input::Sensors;
using modules::localisation::MultiModalRobotModelConfig;
using messages::localisation::Mock;
using messages::localisation::Self;

namespace modules {
namespace localisation {
    MMKFRobotLocalisation::MMKFRobotLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)),
          engine_(std::make_unique<MMKFRobotLocalisationEngine>()) {

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

        on<Trigger<FieldDescription>>("FieldDescription Update", [this](const FieldDescription& desc) {
            NUClear::log("FieldDescription Update");
            auto fd = std::make_shared<FieldDescription>(desc);
            engine_->set_field_description(fd);
        });

        // Emit to NUbugger
        on<Trigger<Every<100, std::chrono::milliseconds>>,
           Options<Sync<MMKFRobotLocalisation>>
           >("NUbugger Output", [this](const time_t&) {
            
            auto robots = std::vector<Self>();
            
            for (auto& model : engine_->robot_models_.hypotheses()) {
                arma::vec::fixed<localisation::robot::RobotModel::size> model_state = model->GetEstimate();
                auto model_cov = model->GetCovariance();

                Self robot_model;
                robot_model.position = model_state.rows(robot::kX, robot::kY);
                robot_model.heading = bearingToUnitVector(model_state(robot::kHeading));
                robot_model.sr_xx = model_cov(0, 0);
                robot_model.sr_xy = model_cov(0, 1);
                robot_model.sr_yy = model_cov(1, 1);
                robots.push_back(robot_model);
            }

            if (engine_->CanEmitFieldObjects()) {
                auto robot_msg = std::make_unique<std::vector<Self>>(robots);
                emit(std::move(robot_msg));
            } else {
                auto mock_robots = Mock<std::vector<Self>>(robots);
                auto mock_robot_msg = std::make_unique<Mock<std::vector<Self>>>(mock_robots);
                emit(std::move(mock_robot_msg));
            }
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
            engine_->TimeUpdate(curr_time, sensors);
        });
        on<Trigger<Every<100, Per<std::chrono::seconds>>>,
           Options<Sync<MMKFRobotLocalisation>>
          >("MMKFRobotLocalisation Time", [this](const time_t&) {
            auto curr_time = NUClear::clock::now();
            engine_->TimeUpdate(curr_time);
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
