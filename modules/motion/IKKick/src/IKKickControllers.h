
/*
 * This file is part of the NUbots Codebase.
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

#ifndef MODULES_MOTION_IKKICKCONTROLLERS_H
#define MODULES_MOTION_IKKICKCONTROLLERS_H

#include "utility/math/matrix/Transform3D.h"
#include "messages/input/Sensors.h"
#include "messages/input/LimbID.h"
#include "messages/input/ServoID.h"
#include <armadillo>

namespace modules{
namespace motion{

		enum MotionStage {
			READY = 0,
			RUNNING = 1,
			STOPPING = 2,
			FINISHED = 3
		};

		class SixDOFMotionController{
			private:
				MotionStage stage;
				bool stable;
			public:
				
				void start(){
					if(stage != MotionStage::FINISHED){
						stage = MotionStage::RUNNING;
					}
				}

				void stop()			{stage = MotionStage::STOPPING;}
				bool isRunning()	{return stage == MotionStage::RUNNING;}
				bool isStable()		{return stable;}
				bool isFinished()   {return stage == MotionStage::FINISHED;}
				void reset()		{stage = MotionStage::READY;}

				virtual utility::math::matrix::Transform3D getFootPose(const messages::input::Sensors& sensors, float deltaT) = 0;
		};

		class KickBalancer : public SixDOFMotionController{
				public:
					virtual utility::math::matrix::Transform3D getFootPose(const messages::input::Sensors& sensors, float deltaT);
		};

		class FootLifter : public SixDOFMotionController{
				public:
					virtual utility::math::matrix::Transform3D getFootPose(const messages::input::Sensors& sensors, float deltaT);
		};

		class Kicker : public SixDOFMotionController{
				public:
					virtual utility::math::matrix::Transform3D getFootPose(const messages::input::Sensors& sensors, float deltaT);	
		};

	}
}

#endif