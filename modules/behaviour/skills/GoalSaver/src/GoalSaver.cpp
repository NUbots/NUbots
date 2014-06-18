/*
 * This file is part of NUbots Codebase.
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

#include "GoalSaver.h"

#include <armadillo>

#include "messages/motion/WalkCommand.h"
#include "messages/motion/DiveCommand.h"
#include "messages/localisation/FieldObject.h"
#include "messages/support/Configuration.h"
#include "messages/input/ServoID.h"
#include "messages/behaviour/Action.h"
#include "messages/vision/VisionObjects.h"
#include "messages/motion/Script.h"


namespace modules {
namespace behaviour {
namespace skills {

    struct ExecuteDive {};
    struct FinishDive {};


    using messages::motion::ExecuteScriptByName;

    using messages::behaviour::RegisterAction;

    using messages::behaviour::ActionPriorites;
    using messages::localisation::Ball;
    using messages::input::ServoID;
    using messages::localisation::Self;
    using messages::motion::DiveCommand;
    using messages::motion::DiveFinished;
    using messages::support::Configuration;
    using messages::motion::WalkStopCommand;
    using messages::behaviour::LimbID;

    GoalSaver::GoalSaver(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this)) {

        on<Trigger<Ball>, With<std::vector<messages::vision::Ball>>>([this] (const Ball& ball, const std::vector<messages::vision::Ball>& vision_balls) {
            if(vision_balls.size()>0 && ball.position[0] > 0){
                NUClear::log("Ball Vel:", -ball.velocity[0] , ball.position[0]);
                if(-ball.velocity[0] > SPEED_THRESHOLD && ball.position[0] < DISTANCE_THRESHOLD){
                    if(ball.position[1]>0){
                        //Dive left
                        auto x = std::make_unique<DiveCommand>();
                        x->direction[0] = 0;
                        x->direction[1] = 1;
                        emit(std::move(x));
                    } else {
                        //Dive right
                        auto x = std::make_unique<DiveCommand>();
                        x->direction[0] = 0;
                        x->direction[1] = -1;
                        emit(std::move(x));
                    }
                }
            }


        });

        // do a little configurating
        on<Trigger<Configuration<GoalSaver>>>([this] (const Configuration<GoalSaver>& config){
            DIVE_PRIORITY = config["DIVE_PRIORITY"].as<float>();
            EXECUTION_PRIORITY = config["EXECUTION_PRIORITY"].as<float>();

            SPEED_THRESHOLD = config["SPEED_THRESHOLD"].as<float>();
            DISTANCE_THRESHOLD = config["DISTANCE_THRESHOLD"].as<float>();
        });

        on<Trigger<DiveCommand>>([this] (const DiveCommand& diveCommand) {

            this->diveCommand = diveCommand;
            updatePriority(DIVE_PRIORITY);

        });

        on<Trigger<ExecuteDive>>([this] (const ExecuteDive&) {
            arma::vec direction = diveCommand.direction;

            int quadrant = getDirectionalQuadrant(direction[0], direction[1]);
            // assume valid at this point as this is checked on the walkcommand trigger
            if (quadrant == 1) {
                // side
                emit(std::make_unique<ExecuteScriptByName>(id,  std::vector<std::string>({"BlockLeft.yaml"})));
            } else if (quadrant == 3) {
                // side
                emit(std::make_unique<ExecuteScriptByName>(id, std::vector<std::string>({"BlockRight.yaml"})));
            }


            updatePriority(EXECUTION_PRIORITY);

        });

        on<Trigger<FinishDive>>([this] (const FinishDive&) {
            emit(std::move(std::make_unique<DiveFinished>()));
            updatePriority(0);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            id,
            "goal_saver",
            { std::pair<float, std::set<LimbID>>(0, { LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM }) },
            [this] (const std::set<LimbID>&) {
                emit(std::make_unique<ExecuteDive>());
            },
            [this] (const std::set<LimbID>&) {
                emit(std::make_unique<FinishDive>());
            },
            [this] (const std::set<ServoID>&) {
                emit(std::make_unique<FinishDive>());
            }
        }));
    }

    void GoalSaver::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { priority }}));
    }

    int GoalSaver::getDirectionalQuadrant(float x, float y) {

            // These represent 4 directions of looking, see https://www.desmos.com/calculator/mm8cnsnpdt for a graph of the 4 quadrants
            // Note that x is forward in relation to the robot so the forward quadrant is x >= |y|
            return x >=  std::abs(y) ? 0  // forward
                 : y >=  std::abs(x) ? 1  // left
                 : x <= -std::abs(y) ? 2  // backward
                 :                     3; // right
    }


}
}
}

