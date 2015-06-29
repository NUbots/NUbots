
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
#include "messages/support/Configuration.h"
#include "messages/input/ServoID.h"
#include <armadillo>
#include <nuclear>

namespace modules{
namespace motion{

		enum MotionStage {
			READY = 0,
			RUNNING = 1,
			STOPPING = 2,
			FINISHED = 3
		};


        struct IKKickConfig{
            static constexpr const char* CONFIGURATION_PATH = "IKKick.yaml";
        };


		class SixDOFMotionController{
			protected:
				MotionStage stage;
				bool stable = false;

				float motion_gain = 10;
				messages::input::LimbID supportFoot;
				arma::vec3 ballPosition;
				arma::vec3 goalPosition;
				NUClear::clock::time_point motionStartTime;

			public:
				virtual void computeMotion(const messages::input::Sensors& sensors) = 0;
				void start(const messages::input::Sensors& sensors){
					if(stage != MotionStage::FINISHED){
						stage = MotionStage::RUNNING;
						computeMotion(sensors);
					}
				}

				void stop()			{stage = MotionStage::STOPPING;}
				bool isRunning()	{return stage == MotionStage::RUNNING;}
				bool isStable()		{return stable;}
				bool isFinished()   {return stage == MotionStage::FINISHED;}
				void reset()		{stage = MotionStage::READY;}


				void setKickParameters(messages::input::LimbID supportFoot_, arma::vec3 ballPosition_, arma::vec3 goalPosition_){
					supportFoot = supportFoot_;
					ballPosition = ballPosition_;
					goalPosition = goalPosition_;
				}

				utility::math::matrix::Transform3D getTorsoPose(const messages::input::Sensors& sensors){
			        // Get our foot positions
			        auto leftFoot = sensors.forwardKinematics.find(messages::input::ServoID::L_ANKLE_ROLL)->second;
			        auto rightFoot = sensors.forwardKinematics.find(messages::input::ServoID::R_ANKLE_ROLL)->second;

			        // Find position vector from support foot to torso in support foot coordinates.
		        	return supportFoot == messages::input::LimbID::LEFT_LEG ? leftFoot.i() : rightFoot.i();
		        }

				virtual utility::math::matrix::Transform3D getFootPose(const messages::input::Sensors& sensors, float deltaT) = 0;

				virtual void configure(const messages::support::Configuration<IKKickConfig>& config) = 0;
		};

		class KickBalancer : public SixDOFMotionController{
		private:
			float stand_height = 0.18;
			float tolerance = 0.01;
		public:
			virtual utility::math::matrix::Transform3D getFootPose(const messages::input::Sensors& sensors, float deltaT);
			virtual void configure(const messages::support::Configuration<IKKickConfig>& config);
			virtual void computeMotion(const messages::input::Sensors& sensors);
		};

		class FootLifter : public SixDOFMotionController{
		private:
			//Config variables
			float velocity = 0.1;
			float lift_foot_height = 0.05;
			float lift_foot_back = 0.01;

			//State variables
			utility::math::matrix::Transform3D startPose;
			utility::math::matrix::Transform3D finishPose;
			float distance;
		public: 
			virtual utility::math::matrix::Transform3D getFootPose(const messages::input::Sensors& sensors, float deltaT);
			virtual void configure(const messages::support::Configuration<IKKickConfig>& config);
			virtual void computeMotion(const messages::input::Sensors& sensors);
		};

		class Kicker : public SixDOFMotionController{
		// private:
		public:
			virtual utility::math::matrix::Transform3D getFootPose(const messages::input::Sensors& sensors, float deltaT);	
			virtual void configure(const messages::support::Configuration<IKKickConfig>& config);
			virtual void computeMotion(const messages::input::Sensors& sensors);
		};

	}
}

#endif