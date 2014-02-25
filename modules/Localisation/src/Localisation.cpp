/*
 * This file is part of Localisation.
 *
 * Localisation is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Localisation is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Localisation.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "Localisation.h"

#include <nuclear>

#include "messages/support/Configuration.h"
#include "utility/NUbugger/NUgraph.h"
#include "utility/math/coordinates.h"
#include "messages/localisation/FieldObject.h"
#include "messages/vision/VisionObjects.h"
#include "localisation/FieldDescription.h"
#include "localisation/LocalisationFieldObject.h"

using utility::NUbugger::graph;
using messages::support::Configuration;

namespace modules {
    Localisation::Localisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<localisation::FieldDescriptionConfig>>>(
            [this](const Configuration<localisation::FieldDescriptionConfig>& config) {
            // std::cout << __func__ << ": Config" << std::endl;
            
            // std::string testConfig = settings.config["testConfig"];
            // std::cout << testConfig << std::endl;

            auto fd = std::make_shared<localisation::FieldDescription>(config);

            engine_.set_field_description(fd);
        });

        // Emit to NUbugger
        on<Trigger<Every<500, std::chrono::milliseconds>>>([this](const time_t&) {
            // emit(std::make_unique<messages::LMissile>());
            // std::cout << __PRETTY_FUNCTION__ << ": rand():" << rand() << std::endl;

            auto state = engine_.robot_models_.GetEstimate();

            std::cout << state << std::endl;

            auto robot_msg = std::make_unique<messages::localisation::FieldObject>();
            robot_msg->name = "self";
            // robot_msg->wm_x = static_cast<float>(rand() % 600 - 300) * 0.01;
            // robot_msg->wm_y = static_cast<float>(rand() % 400 - 200) * 0.01;
            // robot_msg->heading = static_cast<float>(rand() % 628) * 0.01;
            robot_msg->wm_x = state[0];
            robot_msg->wm_y = state[1];
            robot_msg->heading = state[2];
            robot_msg->sd_x = 1;
            robot_msg->sd_y = 0.25;
            robot_msg->sr_xx = 0.01;
            robot_msg->sr_xy = -0.01;
            robot_msg->sr_yy = 0.10;
            robot_msg->lost = false;
            emit(std::move(robot_msg));
        });





        // Simulate Vision
        on<Trigger<Every<500, std::chrono::milliseconds>>>([this](const time_t&) {
            auto goal1 = messages::vision::Goal();
            auto goal2 = messages::vision::Goal();

            goal1.type = messages::vision::Goal::LEFT;
            goal2.type = messages::vision::Goal::RIGHT;

            // auto camera_pos = arma::vec3 { 150.0, 100.0, 40.0 };
            // auto camera_pos = arma::vec3 { 1.50, 1.0, 0.0 };
            auto camera_pos = arma::vec3 { -4.5, 0, 0.0 };

            auto fd = engine_.field_description();
            auto goal_br_pos = fd->GetLFO(localisation::LFOId::kGoalBR).location();
            auto goal_bl_pos = fd->GetLFO(localisation::LFOId::kGoalBL).location();

            auto goal1_pos = arma::vec3 { goal_br_pos[0], goal_br_pos[1], 0.0 };
            auto goal2_pos = arma::vec3 { goal_bl_pos[0], goal_bl_pos[1], 0.0 };

            NUClear::log("Goal positions", goal1_pos, goal2_pos);

            // (dist, bearing, declination)
            goal1.sphericalFromNeck = utility::math::coordinates::Cartesian2Spherical(goal1_pos - camera_pos);
            goal2.sphericalFromNeck = utility::math::coordinates::Cartesian2Spherical(goal2_pos - camera_pos);
            goal1.sphericalError = { 0.1, 0.1, 0.1 };
            goal2.sphericalError = { 0.1, 0.1, 0.1 };

            auto goals = std::make_unique<std::vector<messages::vision::Goal>>();

            goals->push_back(goal2);
            goals->push_back(goal1);

            emit(std::move(goals));
        });





        // on<Trigger<std::vector<messages::vision::Goal>>>([this](const std::vector<messages::vision::Goal>& m) {
        //     engine_.RecordMeasurement(m);
        // });

        on<Trigger<Every<500, std::chrono::milliseconds>>, 
           With<std::vector<messages::vision::Goal>>
          >(
            [this](const time_t&,
                   const std::vector<messages::vision::Goal>& goals) {
            // engine_.TimeUpdate();

            engine_.ProcessObjects(goals);
            engine_.TimeUpdate(0.1);
        });

        // emit<Scope::INITIALIZE>(std::make_unique<localisation::TimeUpdate>());
        // on<Trigger<localisation::TimeUpdate>>([this](const localisation::TimeUpdate& m) {
        //     // engine_.SwapMeasurementBuffers();

        //     // engine_.TimeUpdate();

        //     NUClear::log<NUClear::DEBUG>("Time Update", '\n');

        //     emit(std::make_unique<localisation::ObjectUpdate>());
        // });

        // on<Trigger<localisation::ObjectUpdate>>([this](const localisation::ObjectUpdate& m) {
        //     // engine_.ObjectUpdate();

        //     NUClear::log<NUClear::DEBUG>("Object Update", '\n');

        //     emit(std::make_unique<localisation::TimeUpdate>());
        // });
    }
}
