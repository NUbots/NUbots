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

#include "message/vision/VisionObjects.h"
#include "message/input/CameraParameters.h"
#include "message/input/Sensors.h"
#include "utility/math/coordinates.h"
#include "utility/math/geometry/Quad.h"
#include "utility/localisation/transform.h"

namespace module {
namespace support {

    using message::vision::Goal;
    using message::vision::Ball;
    using message::input::Sensors;
    using utility::math::matrix::Transform2D;
    using utility::localisation::transform::SphericalRobotObservation;
    using message::input::CameraParameters;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::math::coordinates::sphericalToCartesian4;
    using utility::math::geometry::Quad;

    struct VisibleMeasurement {
		std::vector<message::vision::VisionObject::Measurement> measurements;
    	arma::vec2 screenAngular;
    };

	inline static VisibleMeasurement computeVisible(arma::vec3 objPosition, const CameraParameters& camParams, Transform2D robotPose, std::shared_ptr<const Sensors> sensors, arma::vec4 error){
		//Assumes we need to see the bottom or...
		message::vision::VisionObject::Measurement measurement;
        measurement.position = SphericalRobotObservation(robotPose.xy(), robotPose.angle(), objPosition);
        measurement.error = arma::eye(3, 3);

        measurement.error[0] = std::fmax(error(0) * std::fabs(measurement.position(0)), error(1));
        measurement.error.diag()[1] = error[1];
        measurement.error.diag()[2] = error[2];

        arma::vec4 cam_space = sensors->kinematicsCamToGround.i() * sphericalToCartesian4(measurement.position);
        arma::vec2 screenAngular = cartesianToSpherical(cam_space.rows(0,2)).rows(1,2);

		std::vector<message::vision::VisionObject::Measurement> measurements;
        if(std::fabs(screenAngular[0]) < camParams.FOV[0] / 2 && std::fabs(screenAngular[1]) < camParams.FOV[1] / 2){
        	measurements.push_back(measurement);
        	//measurements.push_back(measurement); // TODO: Fix the need for double measurements.
        }

        VisibleMeasurement visibleMeas = {
        	measurements,
        	screenAngular
        };

        return visibleMeas;
    }

	class VirtualGoalPost {
	public:
		VirtualGoalPost(arma::vec3 position_, float height_, Goal::Side side_, Goal::Team team_){
			position = position_;
			height = height_;
			side = side_;
			team = team_;
		}
		~VirtualGoalPost(){}

		arma::vec3 position = {0, 0, 0};
		float height = 1.1;
		Goal::Side side = Goal::Side::UNKNOWN; // LEFT, RIGHT, or UNKNOWN
		Goal::Team team = Goal::Team::UNKNOWN; // OWN, OPPONENT, or UNKNOWN

		Goal detect(const CameraParameters& camParams, Transform2D robotPose, std::shared_ptr<const Sensors> sensors, arma::vec4 error){
			Goal result;

			auto visibleMeasurements = computeVisible(position,camParams,robotPose,sensors,error);

			for (auto & m : visibleMeasurements.measurements){
				result.measurements.push_back(m);
			}

			result.screenAngular = visibleMeasurements.screenAngular;
			result.sensors = sensors;
			result.timestamp = sensors->timestamp; // TODO: Eventually allow this to be different to sensors.
			//Singularity goals
			result.angularSize = arma::vec2({0, 0});
			result.quad = Quad(result.screenAngular,result.screenAngular,result.screenAngular,result.screenAngular);
			result.side = side;
			result.team = team;
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

		Ball detect(const CameraParameters& camParams, Transform2D robotPose, std::shared_ptr<const Sensors> sensors, arma::vec4 error){
			Ball result;

			auto visibleMeasurements = computeVisible(position,camParams,robotPose,sensors,error);

			// TODO: set timestamp, sensors, classifiedImage?
			for (auto& m : visibleMeasurements.measurements){
				m.velocity.rows(0,1) = robotPose.rotation().i() * velocity.rows(0,1);
				m.velCov = 0.1 * arma::eye(3,3);
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
