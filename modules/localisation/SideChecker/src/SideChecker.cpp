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
#include "messages/behaviour/SoccerObjectPriority.h"
#include "messages/behaviour/WalkPath.h"
#include "messages/behaviour/Action.h"
#include "messages/behaviour/SoccerObjectPriority.h"
#include "messages/motion/WalkCommand.h"
#include "messages/motion/KickCommand.h"
#include "messages/behaviour/KickPlan.h"
#include "messages/input/LimbID.h"
#include "messages/input/gameevents/GameEvents.h"
#include "messages/input/ServoID.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/math/geometry/RotatedRectangle.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/angle.h"

namespace modules {
namespace localisation {

    using messages::support::Configuration;

    using messages::support::Configuration;
    using Self = messages::localisation::Self;

    using messages::behaviour::MotionCommand;
    using messages::behaviour::WalkPath;
    using messages::behaviour::RegisterAction;
    using messages::behaviour::ActionPriorites;
    using messages::behaviour::SoccerObjectPriority;
    using messages::behaviour::SearchType;

    using messages::motion::KickFinished;
    using messages::motion::WalkCommand;
    using messages::motion::WalkStartCommand;
    using messages::motion::WalkStopCommand;
    using messages::motion::EnableWalkEngineCommand;
    using messages::motion::DisableWalkEngineCommand;

	using messages::vision::Goal;

    using messages::input::LimbID;
    using messages::input::ServoID;
    using SelfPenalisation = messages::input::gameevents::Penalisation<messages::input::gameevents::SELF>;

    using SelfUnpenalisation = messages::input::gameevents::Unpenalisation<messages::input::gameevents::SELF>;

    using utility::math::geometry::RotatedRectangle;
    using utility::math::matrix::Transform2D;
    using utility::math::angle::vectorToBearing;


    SideChecker::SideChecker(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) , subsumptionId(size_t(this) * size_t(this) - size_t(this)){

        on<Trigger<Configuration<SideChecker>>>([this] (const Configuration<SideChecker>& config) {
            // Use configuration here from file SideChecker.yaml
            cfg_.priority = config["priority"].as<float>();
            cfg_.number_of_samples = config["number_of_samples"].as<int>();
        });

           // Register the path follower with the subsumption system:
        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            subsumptionId,
            "SideChecker",
            {
                // Limb sets required by the sidechecker:
                std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
            },
            [this] (const std::set<LimbID>& givenLimbs) {

        		auto headCommand = std::make_unique<SoccerObjectPriority>();
        		headCommand->goal = 1;
        		headCommand->searchType = SearchType::GOAL_LEFT;
        		currentState = State::SearchLeft;
        		emit(std::move(headCommand));
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


        on<Trigger<SelfUnpenalisation>> ([this](const SelfUnpenalisation&) {
            
            emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { cfg_.priority,cfg_.priority,cfg_.priority}}));
        });

        on<Trigger<std::vector<Goal>>>("Side Checker State Check", [this](const std::vector<Goal>& goals){
        	//record goals:
        	if(goals.size() == 2){
        		bool leftPresent = false;
        		Goal left;
        		bool rightPresent = false;
        		Goal right;
        		for (auto& g : goals){
        			if(g.side == Goal::Side::LEFT){
        				leftPresent = true;
        				left = g;
        			}
        			if(g.side == Goal::Side::RIGHT){
        				rightPresent = true;
        				right = g;
        			}
        		}
        		if (rightPresent && leftPresent){
        			addGoals(left,right);
        		}
        	}

        	//Check state
        	bool leftMeasured = leftGoals.size() >= cfg_.number_of_samples;
        	bool rightMeasured = rightGoals.size() >= cfg_.number_of_samples;

        	if(leftMeasured && currentState == State::SearchLeft){
        		auto headCommand = std::make_unique<SoccerObjectPriority>();
        		headCommand->goal = 1;
        		headCommand->searchType = SearchType::GOAL_RIGHT;
        		currentState = State::SearchRight;
        		emit(std::move(headCommand));
        	} 
        	
        	if(rightMeasured && currentState == State::SearchRight){
        		currentState = State::Calculate;
        	}
        	
        	if(currentState == State::Calculate){
        		//calculateSide();
        		//emit
        	}

        });

		
    }

    void SideChecker::addGoals(Goal left, Goal right){
    	bool leftGoalToLeftOfRobot = left.measurements.front().position[1] > 0;
    	bool rightGoalToLeftOfRobot = right.measurements.front().position[1] > 0;
    	if(leftGoalToLeftOfRobot && rightGoalToLeftOfRobot){
    		leftGoals.push_back(std::pair<Goal,Goal>(left,right));
    	} else if (!(leftGoalToLeftOfRobot && rightGoalToLeftOfRobot)){
    		rightGoals.push_back(std::pair<Goal,Goal>(left,right));
    	}
    }
}
}
