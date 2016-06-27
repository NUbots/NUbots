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
#include "utility/math/vision.h"
#include "message/support/FieldDescription.h"

namespace module {
namespace support {

    using message::vision::Goal;
    using message::vision::Ball;
    using utility::math::vision::cameraSpaceGoalProjection;
    using message::input::Sensors;
    using utility::math::matrix::Transform2D;
    using utility::localisation::transform::SphericalRobotObservation;
    using message::input::CameraParameters;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::math::coordinates::sphericalToCartesian4;
    using utility::math::geometry::Quad;
    using message::support::FieldDescription;


class VirtualGoalPost {
	private:
		arma::vec2 getCamRay(const arma::vec3& norm1, const arma::vec3& norm2, double focalLength, arma::uvec2 imSize) {
			//Solve the vector intersection between two planes to get the camera ray of the quad corner
			arma::vec3 result;
			const double zdiff = norm2[2]*norm1[1] - norm1[2] * norm2[1];
			const double ydiff = norm2[1]*norm1[2] - norm1[1] * norm2[2];
			if (std::abs(zdiff) > std::numeric_limits<double>::epsilon()) {
				result[0] = 1.0;
				result[2] = (norm1[0] * norm2[1] - norm1[1] * norm2[0]) / zdiff;
				result[1] = (-norm1[0] - norm1[2] * result[2]) / norm1[1];

			} else if (std::abs(ydiff) > std::numeric_limits<double>::epsilon()) {
				result[0] = 1.0;
				result[1] = (norm1[0] * norm2[2] - norm1[2] * norm2[0]) / ydiff;
				result[2] = (-norm1[0] - norm1[1] * result[1]) / norm1[2];

			} else {
				result[2] = 1.0;
				const double ndiff = norm1[0] * norm2[1] - norm1[1] * norm2[0];
				result[2] = 1.0;
				result[1] = (norm1[2] * norm2[0] - norm1[0] * norm2[2]) / ndiff;
				result[0] = (-norm1[2] - norm1[1] * result[1]) / norm1[0];
				if (result[0] < 0.0) {
					result *= -1.0;
				}
			}

			return arma::conv_to<arma::vec>::from(
						utility::math::vision::screenToImage(
							utility::math::vision::projectCamSpaceToScreen(result, focalLength), 
							imSize
						)
					);
		}

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

		Goal detect(const CameraParameters& camParams, 
					Transform2D& robotPose, 
					std::shared_ptr<const Sensors> sensors, 
					arma::vec4& error,
					const FieldDescription& field){
			Goal result;

			//make a non-homogeneous robot pose
			arma::vec3 robotPose3;
			robotPose3[2] = std::atan2(robotPose[1], robotPose[0]);
			robotPose3.rows(0,1) = robotPose.submat(0,2,1,2);

			//push the new measurement types
			//TODO: simulate not having values if off screen
			arma::mat::fixed<3,4> goalNormals = cameraSpaceGoalProjection(robotPose3, this->position, field, *sensors);
			result.measurements.push_back(
				std::make_pair<Goal::MeasurementType, arma::vec3>(
							Goal::MeasurementType::LEFT_NORMAL,
							goalNormals.col(0)
						)
				);
			result.measurements.push_back(
				std::make_pair<Goal::MeasurementType, arma::vec3>(
							Goal::MeasurementType::RIGHT_NORMAL,
							goalNormals.col(1)
						)
				);
			result.measurements.push_back(
				std::make_pair<Goal::MeasurementType, arma::vec3>(
							Goal::MeasurementType::TOP_NORMAL,
							goalNormals.col(2)
						)
				);
			result.measurements.push_back(
				std::make_pair<Goal::MeasurementType, arma::vec3>(
							Goal::MeasurementType::BASE_NORMAL,
							goalNormals.col(3)
						)
				);

			//build the predicted quad
            utility::math::geometry::Quad quad(
            		getCamRay(goalNormals.col(0), goalNormals.col(3), camParams.focalLengthPixels, camParams.imageSizePixels),
            		getCamRay(goalNormals.col(0), goalNormals.col(2), camParams.focalLengthPixels, camParams.imageSizePixels),
            		getCamRay(goalNormals.col(1), goalNormals.col(2), camParams.focalLengthPixels, camParams.imageSizePixels),
            		getCamRay(goalNormals.col(1), goalNormals.col(3), camParams.focalLengthPixels, camParams.imageSizePixels)
            	);

			result.sensors = sensors;
			result.timestamp = sensors->timestamp; // TODO: Eventually allow this to be different to sensors.
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

			//auto visibleMeasurements = computeVisible(position,camParams,robotPose,sensors,error);

			// TODO: set timestamp, sensors, classifiedImage?
			/*for (auto& m : visibleMeasurements.measurements){
				m.velocity.rows(0,1) = robotPose.rotation().i() * velocity.rows(0,1);
				m.velCov = 0.1 * arma::eye(3,3);
				result.measurements.push_back(m);
			}*/


			//result.screenAngular = visibleMeasurements.screenAngular;
			//result.angularSize = arma::vec2({0, 0});
			result.sensors = sensors;
			result.timestamp = sensors->timestamp; // TODO: Eventually allow this to be different to sensors.


			//If no measurements are in the Ball, then there it was not observed
			return result;
		}
	};

}
}

#endif
