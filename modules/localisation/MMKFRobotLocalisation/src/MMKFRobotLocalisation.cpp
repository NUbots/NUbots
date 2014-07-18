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
#include "utility/math/matrix.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/localisation/LocalisationFieldObject.h"
#include "messages/vision/VisionObjects.h"
#include "messages/input/Sensors.h"
#include "messages/support/Configuration.h"
#include "messages/support/FieldDescription.h"
#include "messages/localisation/FieldObject.h"
#include "messages/localisation/ResetRobotHypotheses.h"
#include "MMKFRobotLocalisationEngine.h"
#include "RobotModel.h"

using utility::math::angle::bearingToUnitVector;
using utility::math::matrix::zRotationMatrix;
using utility::nubugger::graph;
using utility::localisation::LocalisationFieldObject;
using modules::localisation::MultiModalRobotModelConfig;
using messages::support::Configuration;
using messages::support::FieldDescription;
using messages::input::Sensors;
using messages::vision::Goal;
using messages::localisation::Mock;
using messages::localisation::Self;
using messages::localisation::FakeOdometry;
using messages::localisation::ResetRobotHypotheses;

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

        on<Trigger<Startup>,
           With<Optional<FieldDescription>>>("FieldDescription Update",
           [this](const Startup&, const std::shared_ptr<const FieldDescription>& desc) {
            if (desc == nullptr) {
                NUClear::log(__FILE__, ", ", __LINE__, ": FieldDescription Update: support::configuration::SoccerConfig module might not be installed.");
                throw std::runtime_error("FieldDescription Update: support::configuration::SoccerConfig module might not be installed");
            }

            auto fd = std::make_shared<FieldDescription>(*desc);
            engine_->set_field_description(fd);
        });

        on<Trigger<ResetRobotHypotheses>,
           Options<Sync<MMKFRobotLocalisation>>
          >("Localisation ResetRobotHypotheses", [this](const ResetRobotHypotheses& reset) {
            engine_->Reset(reset);
        });

        // Emit to NUbugger
        on<Trigger<Every<100, std::chrono::milliseconds>>,
           With<Sensors>,
           Options<Sync<MMKFRobotLocalisation>>
           >("Localisation NUbugger Output", [this](const time_t&, const Sensors& sensors) {
            auto& hypotheses = engine_->robot_models_.hypotheses();
            if (hypotheses.size() == 0) {
                NUClear::log<NUClear::ERROR>("MMKFRobotLocalisation has no robot hypotheses.");
                return;
            }

            auto robots = std::vector<Self>();

            for (auto& model : hypotheses) {
                arma::vec::fixed<localisation::robot::RobotModel::size> model_state = model->GetEstimate();
                auto model_cov = model->GetCovariance();

                Self robot_model;
                robot_model.position = model_state.rows(robot::kX, robot::kY);
                arma::mat33 imuRotation = zRotationMatrix(model_state(robot::kImuOffset));
                arma::vec3 world_heading = imuRotation * arma::mat(sensors.orientation.t()).col(0);
                robot_model.heading = world_heading.rows(0, 1);
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

        // on<Trigger<FakeOdometry>,
        //    Options<Sync<MMKFRobotLocalisation>>
        //   >("MMKFRobotLocalisation Odometry", [this](const FakeOdometry& odom) {
        //     auto curr_time = NUClear::clock::now();
        //     engine_->TimeUpdate(curr_time, odom);
        // });
        // on<Trigger<Sensors>,
        //    Options<Sync<MMKFRobotLocalisation>>
        //   >("MMKFRobotLocalisation Sensors", [this](const Sensors& sensors) {
        //     auto curr_time = NUClear::clock::now();
        //     engine_->TimeUpdate(curr_time);
        //     // engine_->SensorsUpdate(sensors);
        // });
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

            // Ignore empty vectors of goals.
            if (goals.size() == 0)
                return;

            // // std::cout << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
            // for (auto& goal : goals) {
            //     // std::cout << __FILE__ << ", " << __LINE__ << ":" << std::endl;
            //     // std::cout << "  side:";
            //     // std::cout << ((goal.side == Goal::Side::LEFT) ? "LEFT" :
            //     //               (goal.side == Goal::Side::RIGHT) ? "RIGHT" : "UNKNOWN")
            //     //           << std::endl;

            //     for(uint i = 0; i < goal.measurements.size(); ++i) {
            //         std::stringstream msg;
            //         msg << ((goal.side == Goal::Side::LEFT) ? "LGoal Pos" :
            //                (goal.side == Goal::Side::RIGHT) ? "RGoal Pos" : "UGoal Pos") <<
            //          " " << i;
            //         emit(graph(msg.str(), goal.measurements[i].position[0], goal.measurements[i].position[1], goal.measurements[i].position[2]));
            //         // std::cout << "  measurement: " << num++ << std::endl;
            //         // std::cout << "    position:" << measurement.position.t() << std::endl;
            //         // std::cout << "    error:" << measurement.error << std::endl;
            //     }
            // }

            auto curr_time = NUClear::clock::now();
            engine_->TimeUpdate(curr_time);
            engine_->ProcessObjects(goals);
            
        });
    }
}
}
