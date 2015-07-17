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

#include "SideChecker.h"

#include "messages/support/Configuration.h"

#include "messages/support/Configuration.h"
#include "messages/localisation/FieldObject.h"
#include "messages/behaviour/MotionCommand.h"
#include "messages/behaviour/WalkPath.h"
#include "messages/behaviour/Action.h"
#include "messages/motion/WalkCommand.h"
#include "messages/motion/KickCommand.h"
#include "messages/behaviour/KickPlan.h"
#include "messages/input/LimbID.h"
#include "messages/vision/VisionObjects.h"
#include "messages/input/ServoID.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/math/geometry/RotatedRectangle.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/angle.h"
#include "utility/math/coordinates.h"

namespace modules {
namespace localisation {

    using messages::support::Configuration;

    using messages::support::Configuration;
    using Self = messages::localisation::Self;

    using messages::behaviour::MotionCommand;
    using messages::behaviour::WalkPath;
    using messages::behaviour::RegisterAction;
    using messages::behaviour::ActionPriorites;

    using messages::motion::KickFinished;
    using messages::motion::WalkCommand;
    using messages::motion::WalkStartCommand;
    using messages::motion::WalkStopCommand;
    using messages::motion::EnableWalkEngineCommand;
    using messages::motion::DisableWalkEngineCommand;

	using messages::vision::Goal;

    using messages::input::LimbID;
    using messages::input::ServoID;

    using utility::math::geometry::RotatedRectangle;
    using utility::math::matrix::Transform2D;
    using utility::math::angle::vectorToBearing;


    SideChecker::SideChecker(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) , subsumptionId(size_t(this) * size_t(this) - size_t(this)){

        on<Trigger<Configuration<SideChecker>>>([this] (const Configuration<SideChecker>& config) {
            // Use configuration here from file SideChecker.yaml
            cfg_.priority = config["priority"];
        });

           // Register the path follower with the subsumption system:
        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            subsumptionId,
            "SideChecker",
            {
                // Limb sets required by the sidechecker:
                std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD}),
            },
            [this] (const std::set<LimbID>& givenLimbs) {

            	

            	// We want to find out if we have been placed on the left or
            	// right side of our end of the field.

            	// Look for left goal (For N measurements):

            	//  - Send a head command to search for the left goal
            	//  - 

            	
            },
            [this] (const std::set<LimbID>& takenLimbs) {

            },
            [this] (const std::set<ServoID>&) {
                // nothing
            }
        }));


        on<Trigger<SelfUnpenalisation>, With<FieldDescription>> ([this](const SelfUnpenalisation&, const FieldDescription& fieldDescription) {
            
            emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { cfg_.priority,cfg_.priority,cfg_.priority}}));
        });

        on<Trigger<std::vector<Goal>>>("Side Checker State Check", [this](const std::vector<Goal>& goals){
        	// Look for right goal (...):

        	// Calculate average distances to each goal:


            if (currentState == State::Calculate) {
                // Determine side of field based on goal distances:
                bool side = calculateSide(leftGoals, rightGoals);

                // Emit the appropriate localisation reset:

                // Relinquish control of the robot:
                emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { 0, 0, 0 }}));
            }
        });
    }

    arma::vec3 avgMeasurement(std::vector<VisionObject::Meaurement> measurements) {
        arma::vec3 avg = {0, 0, 0};
        for (auto& m : measurements) {
            avg += m.position;
        }
        avg /= measurements.size();
        return avg;
    }

    std::pair<arma::vec3, arma::vec3> avgGoalPair(std::vector<std::pair<Goal, Goal>> goalPairs) {
        std::pair<arma::vec, arma::vec> avgGoals = {{0, 0, 0}, {0, 0, 0}};
        for (auto& pair : leftGoalPairs) {

            arma::vec3 leftPos  = avgMeasurement(pair.first.measurements);
            arma::vec3 rightPos = avgMeasurement(pair.second.measurements);;

            avgGoals.left  += leftPos;
            avgGoals.right += rightPos;
        }

        avgGoals.left  /= goalPairs.size();
        avgGoals.right /= goalPairs.size();

        return avgGoals;
    }

    bool SideChecker::calculateSide(std::vector<std::pair<Goal, Goal>> leftGoalPairs, std::vector<std::pair<Goal, Goal>> rightGoalPairs) {
        // Get average spherical coordinates of goals:
        auto avgLeftGoals = avgGoalPairs(leftGoalPairs);
        auto avgRightGoals = avgGoalPairs(rightGoalPairs);

        // Get cartesian goal coordinates:
        arma::vec3 leftL  = math::coordinates::sphericalToCartesian(avgLeftGoals.first);
        arma::vec3 leftR  = math::coordinates::sphericalToCartesian(avgLeftGoals.second);
        arma::vec3 rightL = math::coordinates::sphericalToCartesian(avgLeftGoals.first);
        arma::vec3 rightR = math::coordinates::sphericalToCartesian(avgLeftGoals.second);

        if (leftL(1) < rightR(1) && leftR(1) < rightL(1)) {
            // The left goal is closer (i.e. our own goal), so we are on the right side.
            return true;
        } else if (leftL(1) > rightR(1) && leftR(1) > rightL(1)) {
            // The right goal is closer (i.e. our own goal), so we are on the left side.
            return false;
        }

        // Testing the goalposts individually was inconclusive, so we'll test the goal centres.
        arma::vec3 leftAvg = (leftL + leftR) * 0.5;
        arma::vec3 rightAvg = (rightL + rightR) * 0.5;
        return leftAvg(1) < rightAvg(1);
    }
}
}
