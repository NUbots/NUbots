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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#include "WalkPathFollower.h"

#include "messages/support/Configuration.h"
#include "messages/localisation/FieldObject.h"
#include "messages/motion/WalkCommand.h"
#include "messages/motion/KickCommand.h"
#include "messages/behaviour/KickPlan.h"
#include "messages/behaviour/WalkPath.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/math/geometry/RotatedRectangle.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/angle.h"


namespace modules {
namespace behaviour {
namespace planning {

    using messages::support::Configuration;
    using Self = messages::localisation::Self;

    using messages::behaviour::WalkPath;

    using messages::motion::WalkCommand;
    using messages::motion::KickFinished;
    using messages::motion::WalkStartCommand;
    using messages::motion::WalkStopCommand;
    
    using utility::math::geometry::RotatedRectangle;
    using utility::math::matrix::Transform2D;
    using utility::math::angle::vectorToBearing;


    WalkPathFollower::WalkPathFollower(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Trigger<Configuration<WalkPathFollower>>>([this] (const Configuration<WalkPathFollower>& config) {
            // Use configuration here from file WalkPathFollower.yaml
        });

        on<Trigger<KickFinished>>([this] (const KickFinished&) {
            emit(std::move(std::make_unique<WalkStartCommand>()));
        });

        on<Trigger<Every<20, Per<std::chrono::seconds>>>,
            With<std::vector<Self>>,
            With<WalkPath>,
            Options<Single>
           >("Follow current path plan", [this] (
             const NUClear::clock::time_point&/* current_time*/,
             const std::vector<Self>& selfs,
             const WalkPath& walkPath
             ) {
            if (selfs.empty()) {
                return;
            }
            auto self = selfs.front();

            // Get the robot's current state as an OMPL SE2 state:
            Transform2D currTrans = {self.position(0), self.position(1), vectorToBearing(self.heading)};

            emit(utility::nubugger::drawRectangle("WPF_RobotFootprint", RotatedRectangle(currTrans, {0.12, 0.17})));

            // Find the closest state on the path to the robot's current state:
            // int index = pathGeom->getClosestIndex(currentState);
            // // std::vector<ob::State*>& states = pathGeom->getStates();

            // if (index < 0 || numStates <= index + 1) {
            //     return;
            // }

            // // Make the robot head towards the next state on the path:

            // // ob::State* currState = pathGeom->getState(index);
            // // ob::State* nextState = pathGeom->getState(index + 1);
            // const ob::SE2StateSpace::StateType* nextState =
            //     pathGeom->getState(index + 1)->as<ob::SE2StateSpace::StateType>();

            // // Transform2D currTrans = { currState->getX(), currState->getY(), currState->getYaw() };
            // Transform2D nextTrans = { nextState->getX(), nextState->getY(), nextState->getYaw() };

            // emit(utility::nubugger::drawRectangle("WPF_TargetState", RotatedRectangle(nextTrans, {0.12, 0.17}), {1, 0, 0}));


            // std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>();
            // command->command = nextTrans - currTrans;
            // command->command[2] = utility::math::angle::difference(nextTrans.angle(), currTrans.angle());

            // emit(std::move(std::make_unique<WalkStartCommand>()));
            // emit(std::move(command));
        });
    }
}
}
}
