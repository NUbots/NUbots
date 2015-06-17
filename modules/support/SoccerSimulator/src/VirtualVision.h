/*
 * This file is part of the NUbots Codebase.
 * @author Jake Fountain
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
 #include <armadillo>

#include "messages/vision/VisionObjects.h"
#include "messages/input/CameraParameters.h"
#include "messages/input/Sensors.h"
#include "utility/math/coordinates.h"
#include "utility/localisation/transform.h"


namespace modules {
namespace support {

    using messages::input::Sensors;
    using utility::math::matrix::Transform2D;

	inline static std::vector<messages::vision::VisionObject::Measurement> computeVisible(arma::vec3 objPosition, CameraParameters cam_params, Transform 2D robotPose, std::shared_ptr<Sensors> sensors){
		//Assumes we need to see the bottom or...
		messages::vision::VisionObject::Measurement measurement;
        measurement.position = SphericalRobotObservation(robotPose.xy(), robotPose.angle(), position);
        measurement.error = arma::eye(3, 3) * 0.1;			

        arma::vec4 cam_space = sensors->camToGround.i() * sphericalToCartesian4(measurement1.position);
        arma::vec2 screen_angular = cartesianToSpherical(cam_space.rows(0,2)).rows(1,2);

		std::vector<messages::vision::VisionObject::Measurement> measurements;
        if(std::fabs(screen_angular[0]) < cam_params.FOV[0] / 2 && std::fabs(screen_angular[1]) < cam_params.FOV[1] / 2){
        	measurements.push_back(measurement);
        	measurements.push_back(measurement);
        }
        return measurements;
    }

	class VirtualGoalPost {
	public:
		VirtualGoalPost(arma::vec3 position_, float height_){
			position = position_;
			height = height_;
		}
		~VirtualGoalPost(){}

		arma::vec3 position;
		float height;

		Goal detect(CameraParameters cam_params, Transform 2D robotPose, std::shared_ptr<Sensors> sensors){
			Goal result;

			//Assumes we need to see the bottom or...
			auto bottom_measurements = computeVisible(position,cam_params,robotPose,sensors);

	  //       //...the top, with one measurement each
			// std::vector<messages::vision::VisionObject::Measurement> top_measurements = computeVisible(position+arma::vec3({0,0,height}),cam_params,robotPose,sensors);
			
			// //collect
			// bottom_measurements.insert(bottom_measurements.end()
			// 						   top_measurements.begin(),
			// 						   top_measurements.end());

			for (auto& m : bottom_measurements){
				result.second.measurements.push_back(m);
			}
			//If no measurements are in the goal, then there it was not observed
			return result;
		}
	};

	class VirtualBall {
		VirtualBall(arma::vec2 position_, float diameter_){
			position = position_;
			diameter = diameter_;
		}
		~VirtualBall(){}

		arma::vec2 position;
		float diameter;

		std::pair<bool,Ball> detect(cam_params, robot_pose){
			std::pair<bool,Ball> result;

			return obj;
		}

		Ball detect(CameraParameters cam_params, Transform 2D robotPose, std::shared_ptr<Sensors> sensors){
			Ball result;

			auto bottom_measurements = computeVisible(position,cam_params,robotPose,sensors);

			for (auto& m : bottom_measurements){
				result.second.measurements.push_back(m);
			}
			//If no measurements are in the Ball, then there it was not observed
			return result;
		}
	};

}
}