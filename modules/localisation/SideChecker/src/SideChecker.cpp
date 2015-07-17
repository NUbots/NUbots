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
#include "messages/localisation/ResetRobotHypotheses.h"
#include "messages/localisation/SideChecker.h"
#include "utility/math/angle.h"
#include "utility/math/coordinates.h"

namespace modules {
namespace localisation {

    using messages::support::Configuration;

    using messages::support::Configuration;
    using Self = messages::localisation::Self;
    using messages::localisation::SideCheckingComplete;

    using messages::behaviour::MotionCommand;
    using messages::support::FieldDescription;
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
	using messages::vision::VisionObject;
    
    using messages::localisation::ResetRobotHypotheses;

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
				// We want to find out if we have been placed on the left or
            	// right side of our end of the field.

        		auto headCommand = std::make_unique<SoccerObjectPriority>();
        		headCommand->goal = 1;
        		headCommand->searchType = SearchType::GOAL_LEFT;
        		emit(std::move(headCommand));

        		// Reset state:
        		leftGoals.clear();
        		rightGoals.clear();
        		currentState = State::SearchLeft;
            },
            [this] (const std::set<LimbID>& takenLimbs) {

            },
            [this] (const std::set<ServoID>&) {
                // nothing
            }
        }));

        on<Trigger<SelfUnpenalisation>> ([this](const SelfUnpenalisation&) {
            emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { cfg_.priority }}));
        });

        on<Trigger<std::vector<Goal>>,
           With<FieldDescription>>("Side Checker State Check", [this](
           	const std::vector<Goal>& goals,
           	const FieldDescription& fieldDescr){
        	//record goals:
        	if (goals.size() == 2) {
        		bool leftPresent = false;
        		Goal left;
        		bool rightPresent = false;
        		Goal right;
        		for (auto& g : goals) {
        			if (g.side == Goal::Side::LEFT) {
        				leftPresent = true;
        				left = g;
        			} else if (g.side == Goal::Side::RIGHT) {
        				rightPresent = true;
        				right = g;
        			}
        		}
        		if (rightPresent && leftPresent){
        			addGoals(left, right);
        		}
        	}

        	//Check state
        	bool leftMeasured = leftGoals.size() >= cfg_.number_of_samples;
        	bool rightMeasured = rightGoals.size() >= cfg_.number_of_samples;

        	if (leftMeasured && currentState == State::SearchLeft) {
        		auto headCommand = std::make_unique<SoccerObjectPriority>();
        		headCommand->goal = 1;
        		headCommand->searchType = SearchType::GOAL_RIGHT;
        		emit(std::move(headCommand));
        		currentState = State::SearchRight;
        	} 
        	
        	else if (rightMeasured && currentState == State::SearchRight) {
        		currentState = State::Calculate;
        	}
        	
        	else if (currentState == State::Calculate) {

                ResetType type = calculateSide(leftGoals, rightGoals);

                // Emit the appropriate localisation reset:
                unpenalisedLocalisationReset(fieldDescr, type);
                // Relinquish control of the robot:
                emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { 0 }}));

                // Let behaviour know that we're done:
                emit(std::make_unique<SideCheckingComplete>());
            }
        });
    }

    arma::vec3 avgMeasurement(std::vector<VisionObject::Measurement> measurements) {
        arma::vec3 avg = {0, 0, 0};
        for (auto& m : measurements) {
            avg += m.position;
        }
        avg /= measurements.size();
        return avg;
    }
		
    void SideChecker::addGoals(Goal left, Goal right){
    	bool leftGoalToLeftOfRobot = left.measurements.front().position[1] > 0;
    	bool rightGoalToLeftOfRobot = right.measurements.front().position[1] > 0;
    	
    	std::pair<Goal,Goal> goalPair = std::make_pair(left, right);

    	if(leftGoalToLeftOfRobot && rightGoalToLeftOfRobot){
    		leftGoals.push_back(goalPair);
    	} else if (!(leftGoalToLeftOfRobot && rightGoalToLeftOfRobot)){
    		rightGoals.push_back(goalPair);
   		}
   	}

    std::pair<arma::vec3, arma::vec3> avgGoalPairs(std::vector<std::pair<Goal, Goal>> goalPairs) {
        std::pair<arma::vec, arma::vec> avgGoals = {{0, 0, 0}, {0, 0, 0}};
        for (auto& pair : goalPairs) {

            arma::vec3 leftPos  = avgMeasurement(pair.first.measurements);
            arma::vec3 rightPos = avgMeasurement(pair.second.measurements);;

            avgGoals.first  += leftPos;
            avgGoals.second += rightPos;
        }

        avgGoals.first  /= goalPairs.size();
        avgGoals.second /= goalPairs.size();

        return avgGoals;
    }

    SideChecker::ResetType SideChecker::calculateSide(std::vector<std::pair<Goal, Goal>> leftGoalPairs, std::vector<std::pair<Goal, Goal>> rightGoalPairs) {
        // Get average spherical coordinates of goals:
        auto avgLeftGoals = avgGoalPairs(leftGoalPairs);
        auto avgRightGoals = avgGoalPairs(rightGoalPairs);

        // Get cartesian goal coordinates:
        arma::vec3 leftL  = utility::math::coordinates::sphericalToCartesian(avgLeftGoals.first);
        arma::vec3 leftR  = utility::math::coordinates::sphericalToCartesian(avgLeftGoals.second);
        arma::vec3 rightL = utility::math::coordinates::sphericalToCartesian(avgLeftGoals.first);
        arma::vec3 rightR = utility::math::coordinates::sphericalToCartesian(avgLeftGoals.second);

        if (leftL(1) < rightR(1) && leftR(1) < rightL(1)) {
            // The left goal is closer (i.e. our own goal), so we are on the right side.
            return ResetType::ON_RIGHT_SIDE;
        } else if (leftL(1) > rightR(1) && leftR(1) > rightL(1)) {
            // The right goal is closer (i.e. our own goal), so we are on the left side.
            return ResetType::ON_LEFT_SIDE;
        }

        return ResetType::AMBIGUOUS;
    }


    void SideChecker::unpenalisedLocalisationReset(const FieldDescription& fieldDescription, ResetType resetType) {

        auto reset = std::make_unique<ResetRobotHypotheses>();
        if(resetType == ResetType::ON_LEFT_SIDE){ //On left side of field so home goal is to right
	        ResetRobotHypotheses::Self selfSideLeft;
	        selfSideLeft.position = arma::vec2({-fieldDescription.penalty_robot_start, fieldDescription.dimensions.field_width * 0.5});
	        selfSideLeft.position_cov = arma::eye(2, 2) * 0.1;
	        selfSideLeft.heading = -M_PI_2;
	        selfSideLeft.heading_var = 0.05;
	        reset->hypotheses.push_back(selfSideLeft);

	    } else if(resetType == ResetType::ON_RIGHT_SIDE){ //On right side of field so home goal is to left
	        ResetRobotHypotheses::Self selfSideRight;
	        selfSideRight.position = arma::vec2({-fieldDescription.penalty_robot_start, -fieldDescription.dimensions.field_width * 0.5});
	        selfSideRight.position_cov = arma::eye(2, 2) * 0.1;
	        selfSideRight.heading = M_PI_2;
	        selfSideRight.heading_var = 0.05;
	        reset->hypotheses.push_back(selfSideRight);

	    } else if(resetType == ResetType::AT_OWN_GOAL){ //On right side of field so home goal is to left
	        ResetRobotHypotheses::Self selfSideBaseLine;
	        selfSideBaseLine.position = arma::vec2({-fieldDescription.dimensions.field_length * 0.5 + fieldDescription.dimensions.goal_area_length, 0});
	        selfSideBaseLine.position_cov = arma::eye(2, 2) * 0.1;
	        selfSideBaseLine.heading = 0;
	        selfSideBaseLine.heading_var = 0.05;
	        reset->hypotheses.push_back(selfSideBaseLine);

	    } else {
	    	ResetRobotHypotheses::Self selfSideLeft;
	        selfSideLeft.position = arma::vec2({-fieldDescription.penalty_robot_start, fieldDescription.dimensions.field_width * 0.5});
	        selfSideLeft.position_cov = arma::eye(2, 2) * 0.1;
	        selfSideLeft.heading = -M_PI_2;
	        selfSideLeft.heading_var = 0.05;
	        reset->hypotheses.push_back(selfSideLeft);

	        ResetRobotHypotheses::Self selfSideRight;
	        selfSideRight.position = arma::vec2({-fieldDescription.penalty_robot_start, -fieldDescription.dimensions.field_width * 0.5});
	        selfSideRight.position_cov = arma::eye(2, 2) * 0.1;
	        selfSideRight.heading = M_PI_2;
	        selfSideRight.heading_var = 0.05;
	        reset->hypotheses.push_back(selfSideRight);

	        ResetRobotHypotheses::Self selfSideBaseLine;
	        selfSideBaseLine.position = arma::vec2({-fieldDescription.dimensions.field_length * 0.5 + fieldDescription.dimensions.goal_area_length, 0});
	        selfSideBaseLine.position_cov = arma::eye(2, 2) * 0.1;
	        selfSideBaseLine.heading = 0;
	        selfSideBaseLine.heading_var = 0.05;
	        reset->hypotheses.push_back(selfSideBaseLine);
	    }
        emit(std::move(reset));

    }
}
}
