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
#include "messages/behaviour/MotionCommand.h"
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

    using messages::behaviour::MotionCommand;

    using utility::localisation::transform::RobotToWorldTransform;
    using utility::math::matrix::Transform2D;

    GoalieWalkPlanner::GoalieWalkPlanner(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)){

        on<Trigger<Configuration<GoalieWalkPlanner>>>([this] (const Configuration<GoalieWalkPlanner>& config) {
            // Use configuration here from file GoalieWalkPlanner.yaml
        	command_timeout = config["command_timeout"].as<float>();
			rotation_speed_factor = config["rotation_speed_factor"].as<float>();
			max_rotation_speed = config["max_rotation_speed"].as<float>();
			translation_speed_factor = config["translation_speed_factor"].as<float>();
			max_translation_speed = config["max_translation_speed"].as<float>();
        });

        on<Trigger<std::vector<Ball>>,
        				  With<std::vector<Self>>
        				  >([this](const std::vector<Ball>& balls,
        				  		   const std::vector<Self>& selfs){

        	if (!(balls.empty() || selfs.empty())) {
                std::unique_ptr<MotionCommand> motionCommand;
                // motionCommand->type = MotionCommand::Type::DirectCommand;

                float timeSinceBallSeen = std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now() - balls[0].last_measurement_time).count() * 1e-6;
                if(timeSinceBallSeen < command_timeout){
                    auto& ball = balls[0];
                    auto& self = selfs[0];

                    float selfBearing = std::atan2(self.heading[1], self.heading[0]);
                    float rotationSpeed = - std::fmin(rotation_speed_factor * selfBearing, max_rotation_speed);

                    float translationSpeed = - std::fmin(translation_speed_factor * ball.position[1], max_translation_speed);

                    // motionCommand->walkCommand = Transform2D({0,translationSpeed,rotationSpeed});
                    motionCommand = std::make_unique<MotionCommand>(MotionCommand::DirectCommand({0, translationSpeed, rotationSpeed}));
                } else {
                    motionCommand = std::make_unique<MotionCommand>(MotionCommand::DirectCommand({0, 0, 0}));
                    // motionCommand = std::make_unique<MotionCommand>(MotionCommand::StandStill());
                    // motionCommand->walkCommand = Transform2D({0,0,0});
                }
                emit(std::move(motionCommand));
        	}

        });

	}

}
}
}
