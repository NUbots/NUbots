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

#include "GoalieWalkPlanner.h"

#include "messages/support/Configuration.h"
#include "messages/behaviour/MotionCommand.h"
#include "messages/motion/WalkCommand.h"
#include "messages/localisation/FieldObject.h"
#include "messages/behaviour/Action.h"
#include "messages/input/LimbID.h"
#include "messages/input/ServoID.h"

#include "utility/localisation/transform.h"
#include "utility/math/matrix/Transform2D.h"

namespace modules {
namespace behaviour {
namespace planning {

    using messages::input::LimbID;
    using messages::input::ServoID;

    using messages::support::Configuration;
    using messages::localisation::Self;
    using messages::localisation::Ball;

    using messages::motion::WalkCommand;
    using messages::motion::WalkStartCommand;
    using messages::motion::WalkStopCommand;
    using messages::motion::EnableWalkEngineCommand;
    using messages::motion::DisableWalkEngineCommand;

    using messages::behaviour::ActionPriorites;
    using messages::behaviour::RegisterAction;

    using utility::localisation::transform::RobotToWorldTransform;
    using utility::math::matrix::Transform2D;

    GoalieWalkPlanner::GoalieWalkPlanner(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)),  subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

        on<Trigger<Configuration<GoalieWalkPlanner>>>([this] (const Configuration<GoalieWalkPlanner>& config) {
            // Use configuration here from file GoalieWalkPlanner.yaml
        	command_timeout = config["command_timeout"].as<float>();
			rotation_speed_factor = config["rotation_speed_factor"].as<float>();
			max_rotation_speed = config["max_rotation_speed"].as<float>();
			translation_speed_factor = config["translation_speed_factor"].as<float>();
			max_translation_speed = config["max_translation_speed"].as<float>();
			updatePriority(20);
        });

        updateHandle = on<Trigger<std::vector<Ball>>,
        				  With<std::vector<Self>>
        				  >([this](const std::vector<Ball>& balls,
        				  		   const std::vector<Self>& selfs){

        	if(!(balls.empty() || selfs.empty())){
        		float timeSinceBallSeen = std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now() - balls[0].last_measurement_time).count() * 1e-6;
        		//TODO: config
                emit(std::move(std::make_unique<WalkStartCommand>(subsumptionId)));
        		if(timeSinceBallSeen < command_timeout){
	        		auto& ball = balls[0];
	        		auto& self = selfs[0];

	        		float selfBearing = std::atan2(self.heading[1], self.heading[0]);
	        		float rotationSpeed = - std::fmin(rotation_speed_factor * selfBearing, max_rotation_speed);

	        		float translationSpeed = - std::fmin(translation_speed_factor * ball.position[1], max_translation_speed);

	        		emit(std::make_unique<WalkCommand>(subsumptionId, Transform2D({0,translationSpeed,rotationSpeed})));
	        	} else {
	        		emit(std::make_unique<WalkCommand>(subsumptionId, Transform2D({0,0,0})));
	        	}
        	}

        });

        // Register the path follower with the subsumption system:
        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            subsumptionId,
            "GoalieWalkPlanner",
            {
                // Limb sets required by the walk engine:
                std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG}),
                std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
            },
            [this] (const std::set<LimbID>& givenLimbs) {
                if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                    // Enable the walk engine.
                    emit<Scope::DIRECT>(std::move(std::make_unique<EnableWalkEngineCommand>(subsumptionId)));
                    updateHandle.enable();
                }
            },
            [this] (const std::set<LimbID>& takenLimbs) {
                if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                    // Shut down the walk engine, since we don't need it right now.
                    emit<Scope::DIRECT>(std::move(std::make_unique<DisableWalkEngineCommand>(subsumptionId)));
                    updateHandle.disable();
                }
            },
            [this] (const std::set<ServoID>&) {
                // nothing
            }
        }));
	}

    void GoalieWalkPlanner::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { priority }}));
    }
}
}
}
