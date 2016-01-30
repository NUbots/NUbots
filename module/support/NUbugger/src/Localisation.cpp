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

#include "NUbugger.h"

#include "message/localisation/FieldObject.h"
#include "message/localisation/proto/Localisation.pb.h"

#include "utility/time/time.h"
#include "utility/localisation/transform.h"
#include "utility/nubugger/NUhelpers.h"

namespace module {
namespace support {

    using utility::nubugger::graph;
    using utility::time::getUtcTimestamp;
    using message::localisation::FieldObject;
    using message::localisation::Ball;
    using message::localisation::Self;
    using message::localisation::proto::Localisation;

    void NUbugger::provideLocalisation() {
        handles["localisation"].push_back(
        on<Every<100, std::chrono::milliseconds>,
           Optional<With<std::vector<Ball>>>,
           Optional<With<std::vector<Self>>>,
           Single,
           Priority::LOW>().then("Localisation Reaction (NUbugger.cpp)",
            [this](std::shared_ptr<const std::vector<Ball>> opt_balls,
                   std::shared_ptr<const std::vector<Self>> opt_robots) {
            auto robot_msg = std::make_unique<FieldObject>();
            auto ball_msg = std::make_unique<FieldObject>();
            bool robot_msg_set = false;
            bool ball_msg_set = false;

            if(opt_robots != nullptr && opt_robots->size() > 0) {
                const auto& robots = *opt_robots;

                // Robot message
                std::vector<FieldObject::Model> robot_msg_models;

                for (auto& model : robots) {
                    FieldObject::Model robot_model;
                    robot_msg->name = "self";
                    robot_model.wm_x = model.position[0];
                    robot_model.wm_y = model.position[1];
                    robot_model.heading = std::atan2(model.heading[1], model.heading[0]);
                    robot_model.sd_x = 1;
                    robot_model.sd_y = 0.25;
                    robot_model.sr_xx = model.position_cov(0,0);
                    robot_model.sr_xy = model.position_cov(1,0);
                    robot_model.sr_yy = model.position_cov(1,1);
                    robot_model.lost = false;
                    robot_msg_models.push_back(robot_model);

                    // break; // Only output a single model
                }
                robot_msg->models = robot_msg_models;
                robot_msg_set = true;
            }

            if(robot_msg_set && opt_balls != nullptr && opt_balls->size() > 0) {
                const auto& balls = *opt_balls;
                const auto& robots = *opt_robots;

                // Ball message
                std::vector<FieldObject::Model> ball_msg_models;

                for (auto& model : balls) {
                    arma::vec2 ball_pos = model.position;

                    if (!model.world_space) {
                        ball_pos = utility::localisation::transform::RobotToWorldTransform(
                            robots[0].position, robots[0].heading, model.position);
                    }

                    FieldObject::Model ball_model;
                    ball_msg->name = "ball";
                    ball_model.wm_x = ball_pos[0];
                    ball_model.wm_y = ball_pos[1];
                    ball_model.heading = 0;
                    ball_model.sd_x = 0.1;
                    ball_model.sd_y = 0.1;

                    //Do we need to rotate the variances?
                    ball_model.sr_xx = model.position_cov(0,0);
                    ball_model.sr_xy = model.position_cov(1,0);
                    ball_model.sr_yy = model.position_cov(1,1);
                    ball_model.lost = false;
                    ball_msg_models.push_back(ball_model);

                    emit(graph("NUbugger Localisation ball", ball_pos[0], ball_pos[1]));

                    // break; // Only output a single model
                }
                ball_msg->models = ball_msg_models;
                ball_msg_set = true;
            }

            if (robot_msg_set || ball_msg_set)
                EmitLocalisationModels(robot_msg, ball_msg);
        }));
    }

    void NUbugger::EmitLocalisationModels(const std::unique_ptr<FieldObject>& robot_model, const std::unique_ptr<FieldObject>& ball_model) {

        Localisation localisation;

        auto* api_field_object = localisation.add_field_object();
        api_field_object->set_name(robot_model->name);

        for (FieldObject::Model model : robot_model->models) {
            auto* api_model = api_field_object->add_models();

            api_model->set_wm_x(model.wm_x);
            api_model->set_wm_y(model.wm_y);
            api_model->set_heading(model.heading);
            api_model->set_sd_x(model.sd_x);
            api_model->set_sd_y(model.sd_y);
            api_model->set_sr_xx(model.sr_xx);
            api_model->set_sr_xy(model.sr_xy);
            api_model->set_sr_yy(model.sr_yy);
            api_model->set_lost(model.lost);
        }

        api_field_object = localisation.add_field_object();
        api_field_object->set_name(ball_model->name);

        for (FieldObject::Model model : ball_model->models) {
            auto* api_model = api_field_object->add_models();

            api_model->set_wm_x(model.wm_x);
            api_model->set_wm_y(model.wm_y);
            api_model->set_heading(model.heading);
            api_model->set_sd_x(model.sd_x);
            api_model->set_sd_y(model.sd_y);
            api_model->set_sr_xx(model.sr_xx);
            api_model->set_sr_xy(model.sr_xy);
            api_model->set_sr_yy(model.sr_yy);
            api_model->set_lost(model.lost);
        }

        send(localisation, 1);
    }
}
}
