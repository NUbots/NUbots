
/*
 * This file is part of NUbugger.
 *
 * NUbugger is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUbugger is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUbugger.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "InverseKinematics.h"

#include "messages/support/Configuration.h"
#include "messages/input/ServoID.h"
#include "messages/motion/ServoWaypoint.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/math/matrix.h"

namespace modules {
namespace debug {
    using messages::support::Configuration;
    using messages::motion::ServoWaypoint;
    using messages::input::ServoID;
    using utility::motion::kinematics::calculateLegJoints;
    using utility::math::matrix::xRotationMatrix;
    using utility::math::matrix::yRotationMatrix;
    using utility::math::matrix::zRotationMatrix;

    InverseKinematics::InverseKinematics(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
			on<
				Trigger<Configuration<InverseKinematics> >
			>([this](const Configuration<InverseKinematics>& inverseKinematicsConfig) {
				// walk!

				arma::mat44 target = yRotationMatrix(inverseKinematicsConfig.config["zAngle"], 4);
				target *= xRotationMatrix(inverseKinematicsConfig.config["yAngle"], 4);
				target *= zRotationMatrix(inverseKinematicsConfig.config["xAngle"], 4);
				
				// translation
				target(0,3) = inverseKinematicsConfig.config["x"]; // down/up
				target(1,3) = inverseKinematicsConfig.config["y"]; // left/right
				target(2,3) = inverseKinematicsConfig.config["z"]; // front/back

                bool left = inverseKinematicsConfig.config["left"];
                bool right = inverseKinematicsConfig.config["right"];

                auto waypoints = std::make_unique<std::vector<ServoWaypoint> >();

                if (left) {
                    std::vector<std::pair<ServoID, float> > legJoints = calculateLegJoints(target, true);
                    for (auto& legJoint : legJoints) {
                        ServoWaypoint waypoint;

                        ServoID servoID;
                        float position;

                        std::tie(servoID, position) = legJoint;

                        waypoint.time = NUClear::clock::now() + std::chrono::seconds(2);
                        waypoint.id = servoID;
                        waypoint.position = position;
                        waypoint.gain = 20;

                        waypoints->push_back(waypoint);
                    }
                }

                if (right) {
                    std::vector<std::pair<ServoID, float> > legJoints = calculateLegJoints(target, false);
                    for (auto& legJoint : legJoints) {
                        ServoWaypoint waypoint;

                        ServoID servoID;
                        float position;

                        std::tie(servoID, position) = legJoint;

                        waypoint.time = NUClear::clock::now() + std::chrono::seconds(2);
                        waypoint.id = servoID;
                        waypoint.position = position;
                        waypoint.gain = 20;

                        waypoints->push_back(waypoint);
                    }
                }


                emit(std::move(waypoints));
			});
    }

} // debug
} // modules
