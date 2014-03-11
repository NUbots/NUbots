/*
 * This file is part of MMKFRobotLocalisation.
 *
 * MMKFRobotLocalisation is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MMKFRobotLocalisation is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MMKFRobotLocalisation.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "MMKFRobotLocalisation.h"

#include <nuclear>

#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/NUbugger/NUgraph.h"
#include "messages/vision/VisionObjects.h"
#include "messages/support/Configuration.h"
#include "messages/localisation/FieldObject.h"
#include "RobotModel.h"
#include "FieldDescription.h"
#include "LocalisationFieldObject.h"

using utility::NUbugger::graph;
using messages::support::Configuration;
using utility::NUbugger::graph;

namespace modules {
namespace localisation {
    MMKFRobotLocalisation::MMKFRobotLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<localisation::FieldDescriptionConfig>>>(
            "Configuration Update",
            [this](const Configuration<localisation::FieldDescriptionConfig>& config) {
            auto fd = std::make_shared<localisation::FieldDescription>(config);
            engine_.set_field_description(fd);
        });

        // Emit to NUbugger
        on<Trigger<Every<250, std::chrono::milliseconds>>,
           Options<Sync<MMKFRobotLocalisation>>>("NUbugger Output", [this](const time_t&) {
            // emit(std::make_unique<messages::LMissile>());
            // std::cout << __PRETTY_FUNCTION__ << ": rand():" << rand() << std::endl;

            arma::vec::fixed<localisation::RobotModel::size> state = engine_.robot_models_.GetEstimate();
            auto cov = engine_.robot_models_.GetCovariance();

            // NUClear::log("=====================", "Covariance Matrix\n", cov);
            NUClear::log("=====================", "Number of models: ",
                         engine_.robot_models_.hypotheses().size());
            for (auto& model : engine_.robot_models_.hypotheses()) {
                NUClear::log("    ", *model);
            }

            auto robot_msg = std::make_unique<messages::localisation::FieldObject>();
            std::vector<messages::localisation::FieldObject::Model> robot_msg_models;
            
            for (auto& model : engine_.robot_models_.hypotheses()) {
                arma::vec::fixed<localisation::RobotModel::size> model_state = model->GetEstimate();

                messages::localisation::FieldObject::Model robot_model;
                robot_msg->name = "self";
                robot_model.wm_x = model_state[0];
                robot_model.wm_y = model_state[1];
                robot_model.heading = std::atan2(model_state[3], model_state[2]);
                robot_model.sd_x = 1;
                robot_model.sd_y = 0.25;
                robot_model.sr_xx = cov(0, 0);
                robot_model.sr_xy = cov(0, 1);
                robot_model.sr_yy = cov(1, 1);
                robot_model.lost = false;
                robot_msg_models.push_back(robot_model);
            }
            robot_msg->models = robot_msg_models;
            emit(std::move(robot_msg));

            auto now = NUClear::clock::now();
            auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            double t = static_cast<double>(ms_since_epoch - 1393322147502L);
            // NUClear::log("t", t, ", ms_since_epoch", ms_since_epoch);
            double secs = 5;
            marker_ = { 2 * cos(t / (1000.0 * secs)), 2 * sin(t / (1000.0 * secs)) };
            // NUClear::log("Actual robot position", marker_);

            arma::vec2 diff = { marker_[0] - state[0], marker_[1] - state[1] };
            float distance = arma::norm(diff, 2);
            emit(graph("Localisation estimate distance error", distance));
            emit(graph("Estimated robot position", state[0], state[1]));
            emit(graph("Actual robot position", marker_[0], marker_[1]));

            auto ball_msg = std::make_unique<messages::localisation::FieldObject>();
            std::vector<messages::localisation::FieldObject::Model> ball_msg_models;
            messages::localisation::FieldObject::Model ball_model;
            ball_msg->name = "ball";
            ball_model.wm_x = marker_[0];
            ball_model.wm_y = marker_[1];
            ball_model.heading = 0;
            ball_model.sd_x = 0.1;
            ball_model.sd_y = 0.1;
            ball_model.sr_xx = 0.01;
            ball_model.sr_xy = 0;
            ball_model.sr_yy = 0.01;
            ball_model.lost = false;
            ball_msg_models.push_back(ball_model);
            ball_msg->models = ball_msg_models;
            emit(std::move(ball_msg));
        });


        // Simulate Vision
        on<Trigger<Every<500, std::chrono::milliseconds>>,
           Options<Sync<MMKFRobotLocalisation>>>("Vision Simulation", [this](const time_t&) {
            auto goal1 = messages::vision::Goal();
            auto goal2 = messages::vision::Goal();


            goal1.type = messages::vision::Goal::RIGHT;
            goal2.type = messages::vision::Goal::LEFT;

            // auto camera_pos = arma::vec3 { 150.0, 100.0, 40.0 };
            // auto camera_pos = arma::vec3 { -1.50, -1.0, 0.0 };
            // auto camera_pos = arma::vec3 { -4.5, 0, 0.0 };
            auto camera_pos = arma::vec3 { marker_[0], marker_[1], 0.0 };
            double camera_heading = 3.1415926535;

            auto fd = engine_.field_description();
            auto goal_br_pos = fd->GetLFO(localisation::LFOId::kGoalBR).location();
            auto goal_bl_pos = fd->GetLFO(localisation::LFOId::kGoalBL).location();

            auto goal1_pos = arma::vec3 { goal_br_pos[0], goal_br_pos[1], 0.0 };
            auto goal2_pos = arma::vec3 { goal_bl_pos[0], goal_bl_pos[1], 0.0 };

            // NUClear::log("Goal positions\n", goal1_pos, goal2_pos);

            // (dist, bearing, declination)
            goal1.sphericalFromNeck = utility::math::coordinates::Cartesian2Spherical(goal1_pos - camera_pos);
            goal2.sphericalFromNeck = utility::math::coordinates::Cartesian2Spherical(goal2_pos - camera_pos);

            goal1.sphericalFromNeck[1] = utility::math::angle::normalizeAngle(goal1.sphericalFromNeck[1] - camera_heading);
            goal2.sphericalFromNeck[1] = utility::math::angle::normalizeAngle(goal2.sphericalFromNeck[1] - camera_heading);

            // NUClear::log("---------------------", "goal1.sphericalFromNeck\n", goal1.sphericalFromNeck);
            // NUClear::log("---------------------", "goal2.sphericalFromNeck\n", goal2.sphericalFromNeck);


            goal1.sphericalError = { 0.0001, 0.0001, 0.000001 };
            goal2.sphericalError = { 0.0001, 0.0001, 0.000001 };

            auto goals = std::make_unique<std::vector<messages::vision::Goal>>();

            goals->push_back(goal1);
            goals->push_back(goal2);

            emit(std::move(goals));
        });


        on<Trigger<Every<500, std::chrono::milliseconds>>,
           With<std::vector<messages::vision::Goal>>,
           Options<Sync<MMKFRobotLocalisation>>
          >("MMKFRobotLocalisation Step",
            [this](const time_t&,
                   const std::vector<messages::vision::Goal>& goals) {

            // NUClear::log("=====================");

            // for (auto& goal : goals) {
            //     NUClear::log(goal);
            //     NUClear::log("----------");
            // }

            // engine_.TimeUpdate(0.5);
            engine_.TimeUpdate(0.05);
            engine_.ProcessObjects(goals);
        });


        // on<Trigger<Every<100, std::chrono::milliseconds>>>([this](const time_t&) {
        //     engine_.TimeUpdate(0.05);
        // });
    }
}
}
