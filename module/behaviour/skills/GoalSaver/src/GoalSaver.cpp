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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "GoalSaver.h"

#include <armadillo>

#include "extension/Configuration.h"

#include "message/behaviour/ServoCommand.h"
#include "message/motion/DiveCommand.h"
#include "message/motion/WalkCommand.h"
#include "message/vision/VisionObjects.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/motion/Script.h"

namespace module {
namespace behaviour {
    namespace skills {

        struct ExecuteDive {};
        struct FinishDive {};

        using extension::Configuration;
        using extension::ExecuteScriptByName;

        using message::localisation::Ball;
        using message::localisation::Self;
        using message::motion::DiveCommand;
        using message::motion::DiveFinished;
        using message::motion::StopCommand;

        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;
        using utility::behaviour::ActionPriorites;
        using utility::behaviour::RegisterAction;

        GoalSaver::GoalSaver(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)) {

            // do a little configuration
            on<Configuration>("GoalSaver.yaml").then([this](const Configuration& config) {
                DIVE_PRIORITY      = config["DIVE_PRIORITY"].as<float>();
                EXECUTION_PRIORITY = config["EXECUTION_PRIORITY"].as<float>();
            });

            on<Trigger<DiveCommand>>().then([this](const DiveCommand& diveCommand) {
                this->diveCommand = diveCommand;
                updatePriority(DIVE_PRIORITY);
            });

            on<Trigger<ExecuteDive>>().then([this] {
                arma::vec direction = diveCommand.direction;

                int quadrant = getDirectionalQuadrant(direction[0], direction[1]);
                // assume valid at this point as this is checked on the walkcommand trigger
                if (quadrant == 1) {
                    // side
                    emit(std::make_unique<ExecuteScriptByName>(id, std::vector<std::string>({"DiveLeft.yaml"})));
                }
                else if (quadrant == 3) {
                    // side
                    emit(std::make_unique<ExecuteScriptByName>(id, std::vector<std::string>({"DiveRight.yaml"})));
                }

                updatePriority(EXECUTION_PRIORITY);
            });

            on<Trigger<FinishDive>>().then([this] {
                emit(std::move(std::make_unique<DiveFinished>()));
                updatePriority(0);
            });

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
                RegisterAction{id,
                               "Goal Saver",
                               {std::pair<float, std::set<LimbID>>(
                                   0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM})},
                               [this](const std::set<LimbID>&) { emit(std::make_unique<ExecuteDive>()); },
                               [this](const std::set<LimbID>&) { emit(std::make_unique<FinishDive>()); },
                               [this](const std::set<ServoID>&) { emit(std::make_unique<FinishDive>()); }}));
        }

        void GoalSaver::updatePriority(const float& priority) {
            emit(std::make_unique<ActionPriorites>(ActionPriorites{id, {priority}}));
        }

        int GoalSaver::getDirectionalQuadrant(float x, float y) {

            // These represent 4 directions of looking, see https://www.desmos.com/calculator/mm8cnsnpdt for a graph of
            // the 4 quadrants
            // Note that x is forward in relation to the robot so the forward quadrant is x >= |y|
            return x >= std::abs(y) ? 0                                          // forward
                                    : y >= std::abs(x) ? 1                       // left
                                                       : x <= -std::abs(y) ? 2   // backward
                                                                           : 3;  // right
        }
    }  // namespace skills
}  // namespace behaviour
}  // namespace module
