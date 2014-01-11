/*
 * This file is part of WalkEngine.
 *
 * WalkEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * WalkEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WalkEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "WalkEngine.h"

#include <armadillo>

#include "messages/motion/ServoWaypoint.h"
#include "messages/support/Configuration.h"
#include "utility/motion/InverseKinematics.h"

using messages::support::Configuration;
using messages::motion::ServoWaypoint;
using utility::motion::kinematics::calculateLegJoints;

namespace modules {
    namespace motion {
        
        WalkEngine::WalkEngine(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

			// Considerations
			//	Configuration
			//		Step height
			//		Motion speed (fps)
			//		Max speed
			//	Distortion
			//		Command
			//			Walk rotation (make left or right step longer/shorter to turn, also rotate hip motors to move with it)
			//			Walk speed (should probably be between 0 and 1 unless a real value can be calculated)
			//		Balance
			//			For a particular speed, the centre of mass should be kept at a fixed distance in front
			//			Left/Right
			//				Correct using grounded foot ankle to add force left or right?
			//				Correct by adjusting mass position using air foot?
			//			Forward Backward
			//				Fixed by changing step length so foot landing matches expected centre of mass (Kinematics/IMU)
			//			
			/*
			airfoot
			execute script frame
			groundfoot
			execute balance maintainnece operations (including arms)
			
			std::function<int (int int)> thing = [this, thing1, thing2](int a, int b) {
				
			};

			std::vector<std::function<int (int int)>> things

			frame = frames[i](1,2);
			*/

			struct WalkCommand {
				float forwardSpeed; // percentage of max speed
				float rotationSpeed; // radians/s, positive = left rotation (right hand rule)
			};

			on<Trigger<Configuration<WalkEngine>>>([this](const Configuration<WalkEngine>& config) {
				// TODO: step height, motion speed, max speed
				//make teh lambads

				std::vector<std::function<std::vector<ServoWaypoint> (const WalkCommand&)>> leftCycle;
				std::vector<std::function<std::vector<ServoWaypoint> (const WalkCommand&)>> rightCycle;
				
				/*for (int i = 0; i < cycleTime / framerate; ++i) {
					// make frames
				}*/
			});
			
			on<
				Trigger<Every<5, std::chrono::seconds>>,
				With<WalkCommand>
			>([this](const time_t&, const WalkCommand& walkCommand) {
				// walk!

				NUClear::log<NUClear::DEBUG>("I must walk!");
				/*arma::mat target = {0, 0, 0, 0,
				                    0, 0, 0, 0,
				                    0, 0, 0, 0,
				                    0, 0, 0, 0};
				target.reshape(4,4);*/
				arma::mat44 target;
				calculateLegJoints(target, true);
				
			});

			// temp test

			emit(std::move(std::make_unique<WalkCommand>(WalkCommand{1, 0})));

			
        }
        
    }  // motion
}  // modules
