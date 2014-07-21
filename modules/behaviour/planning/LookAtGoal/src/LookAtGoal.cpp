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

#include "LookAtGoal.h"

#include "messages/vision/VisionObjects.h"
#include "messages/localisation/FieldObject.h"
#include "messages/behaviour/LookStrategy.h"
#include "messages/behaviour/Look.h"
#include "messages/input/Sensors.h"
#include "messages/support/Configuration.h"
#include "utility/support/armayamlconversions.h"


namespace modules {
namespace behaviour {
namespace planning {

    using messages::vision::Ball;
    using messages::vision::Goal;
    using messages::behaviour::LookAtAngle;
    using messages::behaviour::LookAtPoint;
    using messages::behaviour::LookAtPosition;
    using messages::behaviour::Look;
    using messages::behaviour::GoalHeadBehaviourConfig;
    using messages::input::Sensors;
    using messages::support::Configuration;

    LookAtGoal::LookAtGoal(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Trigger<Configuration<GoalHeadBehaviourConfig>>>([this](const Configuration<GoalHeadBehaviourConfig>& config) {
            GOAL_SEARCH_TIMEOUT_MILLISECONDS = config["GOAL_SEARCH_TIMEOUT_MILLISECONDS"].as<float>();
            SCAN_YAW = config["SCAN_YAW"].as<arma::vec>();
            SCAN_PITCH = config["SCAN_PITCH"].as<arma::vec>();
        });

        //this reaction focuses on the ball - pan'n'scan if not visible and focus on as many objects as possible if visible
        lookAtReactionHandler = on<Trigger<std::vector<Goal>>,
            With<std::vector<Ball>>,
            With<Sensors> >([this]
            (const std::vector<Goal>& goals,
             const std::vector<Ball>& balls,
             const Sensors& sensors) {
            if (goals.size() > 0) {
                std::cout<<__FILE__<<" "<<__LINE__<<": "<<__func__<<std::endl;
                timeSinceLastSeen = sensors.timestamp;
                std::vector<LookAtAngle> angles;
                angles.reserve(4);

                for (const auto& g : goals) {
                    angles.emplace_back(LookAtAngle {g.screenAngular[0], -g.screenAngular[1]});
                }

                for (const auto& b : balls) {
                    angles.emplace_back(LookAtAngle {b.screenAngular[0], -b.screenAngular[1]});
                }

                //XXX: add looking at robots as well


                //this does a small pan around the ball to try to find other objects
                //XXX: under development
                /*if (angles.size() == 1) {
                    double theta = atan2(-angles[0].pitch,angles[0].yaw) + 0.001; //XXX: configurate the pan
                    const double distance = 0.5; //XXX: configurate this distance
                    angles.push_back(LookAtAngle {cos(theta)*distance-angles[0].pitch,-sin(theta)*distance-angles[0].yaw});
                    std::cout << theta << ", " << distance << ", " << angles[0].pitch << ", " << -angles[0].yaw << std::endl;
                    std::cout << cos(theta) << ", " << sin(theta) << ", " << cos(theta)*distance-angles[0].pitch << ", " << -sin(theta)*distance-angles[0].yaw << std::endl;
                }*/

                emit(std::make_unique<std::vector<LookAtAngle>>(angles));
            //} else if (ball != NULL) {
                //XXX: add a localisation based scan'n'pan


            } else if(std::chrono::duration<float, std::ratio<1,1000>>(sensors.timestamp - timeSinceLastSeen).count() > GOAL_SEARCH_TIMEOUT_MILLISECONDS){
                std::cout<<__FILE__<<" "<<__LINE__<<": "<<__func__<<std::endl;
                //do a blind scan'n'pan
                //XXX: this needs to be a look at sequence rather than a look at point
                std::vector<LookAtPosition> angles;

                // const size_t panPoints = 20;
                // for (size_t i = 0; i < panPoints+1; ++i) {
                //     // see http://en.wikipedia.org/wiki/Lissajous_curve
                //     //angles.emplace_back(LookAtPosition {i*scanYaw/panPoints-scanYaw/2.0,-scanPitch*(i%2)+scanPitch});
                //     double t = (i / double(panPoints + 1)) * 2 * M_PI;
                //     double a = 1;
                //     double b = 4;
                //     double A = 1;
                //     double B = 1;
                //     double d = 0;
                //     double x = A * std::sin(a * t + d);
                //     double y = B * std::sin(b * t);
                //     double xt = (x + 1) * 0.5;
                //     double yt = (y + 1) * 0.5;
                //     double yaw = xt * SCAN_YAW(1) + (1 - xt) * SCAN_YAW(0);
                //     double pitch = yt * SCAN_PITCH(1) + (1 - yt) * SCAN_PITCH(0);
                //     angles.emplace_back(LookAtPosition {yaw, pitch});
                // }

                const double scanYaw = SCAN_YAW(1);
                const double scanPitch = SCAN_PITCH(1);
                const size_t panPoints = 6;
                for (size_t i = 0; i < panPoints+1; ++i) {
                    double t = i/double(panPoints);
                    angles.emplace_back(LookAtPosition {t * scanYaw - scanYaw / 2.0, -scanPitch*(i%2)+scanPitch-0.4091});
                }
                for (size_t i = 0; i < panPoints+1; ++i) {
                    double t = i/double(panPoints);
                    angles.emplace_back(LookAtPosition {-(t * scanYaw - scanYaw / 2.0), -scanPitch*((i+1)%2)+scanPitch-0.4091});
                }

                emit(std::make_unique<std::vector<LookAtPosition>>(angles));
            }

        });

        on<Trigger<Look::PanSelection>>([this](const Look::PanSelection& panSelection) {
            lookAtReactionHandler.enable(panSelection.lookAtGoalInsteadOfBall);
        });
    }
}  // planning
}  // behaviours
}  // modules
