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

#ifndef MODULES_VIRTUALVISION
#define MODULES_VIRTUALVISION

#include <armadillo>

#include "messages/vision/VisionObjects.h"
#include "messages/input/CameraParameters.h"
#include "messages/input/Sensors.h"
#include "utility/math/coordinates.h"
#include "utility/math/geometry/Quad.h"
#include "utility/localisation/transform.h"


namespace modules {
namespace support {

    using messages::vision::Goal;
    using messages::vision::Ball;
    using messages::input::Sensors;
    using utility::math::matrix::Transform2D;
    using utility::localisation::transform::SphericalRobotObservation;
    using messages::input::CameraParameters;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::math::coordinates::sphericalToCartesian4;
    using utility::math::geometry::Quad;

    struct VisibleMeasurement {
		std::vector<messages::vision::VisionObject::Measurement> measurements;
    	arma::vec2 screenAngular;
    };

	inline static VisibleMeasurement computeVisible(arma::vec3 objPosition, const CameraParameters& camParams, Transform2D robotPose, std::shared_ptr<Sensors> sensors){
		//Assumes we need to see the bottom or...
		messages::vision::VisionObject::Measurement measurement;
        measurement.position = SphericalRobotObservation(robotPose.xy(), robotPose.angle(), objPosition);
        measurement.error = arma::eye(3, 3) * 0.1;			

        arma::vec4 cam_space = sensors->kinematicsCamToGround.i() * sphericalToCartesian4(measurement.position);
        arma::vec2 screenAngular = cartesianToSpherical(cam_space.rows(0,2)).rows(1,2);

		std::vector<messages::vision::VisionObject::Measurement> measurements;
        if(std::fabs(screenAngular[0]) < camParams.FOV[0] / 2 && std::fabs(screenAngular[1]) < camParams.FOV[1] / 2){
        	measurements.push_back(measurement);
        	measurements.push_back(measurement); // TODO: Fix the need for double measurements.
        }

        VisibleMeasurement visibleMeas = {
        	measurements,
        	screenAngular
        };

        return visibleMeas;
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

		Goal detect(const CameraParameters& camParams, Transform2D robotPose, std::shared_ptr<Sensors> sensors){
			Goal result;

			auto visibleMeasurements = computeVisible(position,camParams,robotPose,sensors);

			for (auto & m : visibleMeasurements.measurements){
				result.measurements.push_back(m);
			}
			
			result.screenAngular = visibleMeasurements.screenAngular;
			result.sensors = sensors;
			result.timestamp = sensors->timestamp; // TODO: Eventually allow this to be different to sensors.
			//Singularity goals
			result.angularSize = arma::vec2({0, 0});
			result.quad = Quad(result.screenAngular,result.screenAngular,result.screenAngular,result.screenAngular);
			result.side = messages::vision::Goal::Side::UNKNOWN;
			//If no measurements are in the goal, then it was not observed
			return result;
		}
	};

	class VirtualBall {
	public: 
		VirtualBall() {
			position = arma::vec3({0,0,0});
			diameter = 0.1;
		}

		VirtualBall(arma::vec2 position_, float diameter_) {
			position = position_;
			diameter = diameter_;
		}
		~VirtualBall(){}

		// utility::math::matrix::Transform2D ballPose;
		arma::vec3 position;
        arma::vec3 velocity;  

		// arma::vec2 position;
		float diameter;

		Ball detect(const CameraParameters& camParams, Transform2D robotPose, std::shared_ptr<Sensors> sensors){
			Ball result;

			auto visibleMeasurements = computeVisible(position, camParams, robotPose, sensors);

			// TODO: set timestamp, sensors, classifiedImage?
			for (auto& m : visibleMeasurements.measurements){
				result.measurements.push_back(m);
			}


			result.screenAngular = visibleMeasurements.screenAngular;
			result.angularSize = arma::vec2({0, 0});
			result.sensors = sensors;
			result.timestamp = sensors->timestamp; // TODO: Eventually allow this to be different to sensors.


			//If no measurements are in the Ball, then there it was not observed
			return result;
		}
	};

}
}

#endif
