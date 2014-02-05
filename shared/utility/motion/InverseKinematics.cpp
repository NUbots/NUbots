/*
 * This file is part of InverseKinematics.
 *
 * InverseKinematics is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * InverseKinematics is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with InverseKinematics.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include <nuclear_bits/LogLevel.h>
#include <cmath>
#include <nuclear>

#include "InverseKinematics.h"

#include "utility/math/matrix.h"

namespace utility {
namespace motion {
namespace kinematics {

	using messages::input::ServoID;
	using utility::math::matrix::xRotationMatrix;
	using utility::math::matrix::yRotationMatrix;
	
	std::vector<std::pair<ServoID, float> > calculateLegJoints(arma::mat44 target, bool isLeft) {

		static const float LENGTH_BETWEEN_LEGS = 74.0f;
		static const float UPPER_LEG_LENGTH = 93.0f;
		static const float LOWER_LEG_LENGTH = 93.0f;
		static const double PI = arma::math::pi();

		const int sign(isLeft ? -1 : 1);
		std::vector<std::pair<ServoID, float> > positions;

		// add a translation in the y axis - shifts to absolute offset?
        target(1,3) += sign * LENGTH_BETWEEN_LEGS / 2;
        //target = TransformMatrices::RotX(sign * pi_4) * target;
        target = arma::inv(target);

        // Matrix access format is matrix[row][collumn]

		// total distance of translation
        float length = sqrt(target(0,3) * target(0,3) + target(1,3) * target(1,3) + target(2,3) * target(2,3));
        float sqrLength = length * length;
        float sqrUpperLeg = UPPER_LEG_LENGTH * UPPER_LEG_LENGTH;
        float sqrLowerLeg = LOWER_LEG_LENGTH * LOWER_LEG_LENGTH;

		// cosine rule (a^2 = b^2 + c^2 - 2*b*c*cosA) rearranged for cosA
		// angle between ankle and angle-hip vector
        float cosLowerLeg = (sqrLowerLeg + sqrLength - sqrUpperLeg) / (2 * LOWER_LEG_LENGTH * length);
		// angle between upper and lower leg
        float cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);

        const float min = -1.0f;
        const float max = 1.0f;

        //!< TODO: Check if this cosLowerLeg shoud condition should be inverted.
        /*if (!isInside(cosKnee, min, max) || isInside(cosLowerLeg, min, max)) {
			cosKnee = limit(cosKnee, min, max);
			cosLowerLeg = limit(cosLowerLeg, min, max);
			throw std::runtime_error("Target angle unreachable");
        }*/

        float joint3 = PI - acos(cosKnee);
        float joint4 = -acos(cosLowerLeg);
        double yzabs = sqrt(target(1,3)*target(1,3) + target(2,3)*target(2,3));
        joint4 -= atan2(target(0,3), yzabs);
        float joint5 = atan2(target(1,3), target(2,3)) * sign;

        // calculate rotation matrix before hip joints
		arma::mat33 hipFromFoot(arma::fill::eye);

        hipFromFoot = hipFromFoot * xRotationMatrix(-sign * joint5, 3) * yRotationMatrix(-joint4 - joint3, 3);

        // compute rotation matrix for hip from rotation before hip and desired rotation
        arma::mat33 hip = arma::inv(hipFromFoot) * target.submat(0,0,2,2);

        // compute joints from rotation matrix using theorem of euler angles
        // see http://www.geometrictools.com/Documentation/EulerAngles.pdf
        // this is possible because of the known order of joints (z, x, y seen from body resp. y, x, z seen from foot)
        float joint1 = asin(-hip(1,2)) * -sign;
        float joint2 = -atan2(hip(0,2), hip(2,2));
        float joint0 = -atan2(hip(1,0), hip(1,1));

        // Since the joint ordering is different simplest way is to directly index.
		if (isLeft) {
            positions.push_back(std::make_pair(ServoID::L_HIP_YAW, joint0));
            positions.push_back(std::make_pair(ServoID::L_HIP_ROLL, -joint1));
            positions.push_back(std::make_pair(ServoID::L_HIP_PITCH, joint2));
            positions.push_back(std::make_pair(ServoID::L_KNEE, joint3));
            positions.push_back(std::make_pair(ServoID::L_ANKLE_PITCH, joint4));
            positions.push_back(std::make_pair(ServoID::L_ANKLE_ROLL, -joint5));
		} else {
            positions.push_back(std::make_pair(ServoID::R_HIP_YAW, joint0));
            positions.push_back(std::make_pair(ServoID::R_HIP_ROLL, joint1));
            positions.push_back(std::make_pair(ServoID::R_HIP_PITCH, joint2));
            positions.push_back(std::make_pair(ServoID::R_KNEE, joint3));
            positions.push_back(std::make_pair(ServoID::R_ANKLE_PITCH, joint4));
            positions.push_back(std::make_pair(ServoID::R_ANKLE_ROLL, joint5));
		}

		return positions;

		//return std::vector<std::pair<ServoID, float>>();
		
	}

	std::vector<std::pair<ServoID, float> > calculateLegJoints2(arma::mat44 target, bool isLeft) {
        // given in meters
		static const float LENGTH_BETWEEN_LEGS = 0.074;
		static const float UPPER_LEG_LENGTH = 0.093;
		static const float LOWER_LEG_LENGTH = 0.093;

		const int sign(isLeft ? -1 : 1);
		std::vector<std::pair<ServoID, float> > positions;

        // minus epsilon so that a straight leg will be straight!
        float dx = target(0,3) - std::numeric_limits<float>::epsilon();
        float dy = target(1,3) - std::numeric_limits<float>::epsilon();
        float dz = target(2,3) - std::numeric_limits<float>::epsilon();

        float length = sqrt(dz * dz + dy * dy);

        float hipYaw = 0;
        float hipRoll = 0;
        float hipPitch = 0;
        float knee = 0;
        float anklePitch = 0;
        float ankleRoll = 0;

        float sqrLength = length * length;
        float sqrUpperLeg = UPPER_LEG_LENGTH * UPPER_LEG_LENGTH;
        float sqrLowerLeg = LOWER_LEG_LENGTH * LOWER_LEG_LENGTH;

        float cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);
        float cosUpperLeg = (sqrUpperLeg + sqrLength - sqrLowerLeg) / (2 * UPPER_LEG_LENGTH * length);

        // TODO: check if cosKnee is between 1 and -1

        knee = M_PI - acos(cosKnee);
        hipPitch = -(acos(cosUpperLeg) + atan(dy / dz));

        if (isLeft) {
            positions.push_back(std::make_pair(ServoID::L_HIP_YAW, hipYaw));
            positions.push_back(std::make_pair(ServoID::L_HIP_ROLL, -hipRoll));
            positions.push_back(std::make_pair(ServoID::L_HIP_PITCH, hipPitch));
            positions.push_back(std::make_pair(ServoID::L_KNEE, knee));
            positions.push_back(std::make_pair(ServoID::L_ANKLE_PITCH, anklePitch));
            positions.push_back(std::make_pair(ServoID::L_ANKLE_ROLL, -ankleRoll));
        }

        return positions;

    }

	std::vector<std::pair<ServoID, float> > calculateLegJoints3(arma::mat44 target, bool isLeft) {
		static const float LENGTH_BETWEEN_LEGS = 0.074;
        static const float DISTANCE_FROM_BODY_TO_HIP_JOINT = 0.034;
		static const float UPPER_LEG_LENGTH = 0.093;
		static const float LOWER_LEG_LENGTH = 0.093;

		std::vector<std::pair<ServoID, float> > positions;

        float hipYaw = 0;
        float hipRoll = 0;
        float hipPitch = 0;
        float knee = 0;
        float anklePitch = 0;
        float ankleRoll = 0;

        arma::vec3 ankleX = target.submat(0,0,2,0);
        arma::vec3 ankleY = target.submat(0,1,2,1);
        arma::vec3 ankleZ = target.submat(0,2,2,2);

        arma::vec3 anklePos = target.submat(0,3,2,3);

        arma::vec3 hipOffset = {LENGTH_BETWEEN_LEGS / 2.0, 0, DISTANCE_FROM_BODY_TO_HIP_JOINT};

        arma::vec3 targetLeg = anklePos - hipOffset;

        //NUClear::log<NUClear::DEBUG>(ankleX, "\n", ankleY, "\n", ankleZ, "\n", anklePos);
        //NUClear::log<NUClear::DEBUG>(hipOffset, "\n", targetLeg);


        float length = arma::norm(targetLeg, 2);
        if(length > UPPER_LEG_LENGTH+LOWER_LEG_LENGTH){
            NUClear::log<NUClear::DEBUG>("InverseKinematics::calculateLegJoints3 : Requested position beyond leg reach.");
            return positions;
        }
        //NUClear::log<NUClear::DEBUG>("Length: ", length);
        float sqrLength = length * length;
        float sqrUpperLeg = UPPER_LEG_LENGTH * UPPER_LEG_LENGTH;
        float sqrLowerLeg = LOWER_LEG_LENGTH * LOWER_LEG_LENGTH;

        float cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);
       // NUClear::log<NUClear::DEBUG>("Cos Knee: ", cosKnee);
        // TODO: check if cosKnee is between 1 and -1
        knee = acos(cosKnee);

        float cosLowerLeg = (sqrLowerLeg + sqrLength - sqrUpperLeg) / (2 * LOWER_LEG_LENGTH * length);
        // TODO: check if cosLowerLeg is between 1 and -1
        float lowerLeg = acos(cosLowerLeg);

        float phi2 = acos(arma::dot(targetLeg, ankleY)/length);

        anklePitch = lowerLeg + phi2 - M_PI_2;

        arma::vec3 unitTargetLeg = targetLeg / length;

        arma::vec3 hipX = arma::cross(ankleY, unitTargetLeg);
        float hipXLength = arma::norm(hipX,2);
        if(hipXLength>0){
            hipX /= hipXLength;
        } else {
            NUClear::log<NUClear::DEBUG>("InverseKinematics::calculateLegJoints3 : targetLeg and ankleY parrallel. This is unhandled at the moment.");
            return positions;
        }
        arma::vec3 legPlaneTangent = arma::cross(ankleY, hipX); //Will be unit as ankleY and hipX are normal and unit

        ankleRoll = atan2(arma::dot(ankleX, legPlaneTangent),arma::dot(ankleX, hipX));

        arma::vec3 globalX = {1,0,0};
        arma::vec3 globalY = {0,1,0};
        arma::vec3 globalZ = {0,0,1};

        bool isAnkleAboveWaist = arma::dot(unitTargetLeg,globalZ)<0;
        
        float cosZandHipX = arma::dot(globalZ, hipX);
        bool hipRollPositive = cosZandHipX <= 0;
        arma::vec3 legPlaneGlobalZ = (isAnkleAboveWaist ? -1 : 1 ) * (globalZ - ( cosZandHipX * hipX));
        float legPlaneGlobalZLength = arma::norm(legPlaneGlobalZ, 2);
        if(legPlaneGlobalZLength>0){
           legPlaneGlobalZ /= legPlaneGlobalZLength;
        }

        float cosHipRoll = arma::dot(legPlaneGlobalZ, globalZ);
        // TODO: check if cosHipRoll is between 1 and -1
        hipRoll = (hipRollPositive ? 1 : -1) * acos(cosHipRoll);


        float phi4 = M_PI - knee - lowerLeg;
        //Superposition values:
        float sinPIminusPhi2 = std::sin(M_PI - phi2);
        arma::vec3 unitUpperLeg = unitTargetLeg * (std::sin(phi2 - phi4) / sinPIminusPhi2) + ankleY * (std::sin(phi4) / sinPIminusPhi2);
        bool isHipPitchPositive = dot(hipX,cross(unitUpperLeg, legPlaneGlobalZ))>=0;    

        hipPitch = (isHipPitchPositive ? 1 : -1) * acos(arma::dot(legPlaneGlobalZ, unitUpperLeg));
        
        arma::vec3 hipXProjected = (isAnkleAboveWaist ? -1 : 1) * hipX;  //If leg is above waist then hipX is pointing in the wrong direction in the xy plane 
        hipXProjected[2] = 0;
        hipXProjected /= arma::norm(hipXProjected, 2);
        bool isHipYawPositive = arma::dot(hipXProjected,globalY)>=0;

       
        hipYaw = (isHipYawPositive ? 1 : -1) * acos(arma::dot( hipXProjected,globalX));

        if (isLeft) {
            positions.push_back(std::make_pair(ServoID::L_HIP_YAW, -hipYaw));
            positions.push_back(std::make_pair(ServoID::L_HIP_ROLL, hipRoll));
            positions.push_back(std::make_pair(ServoID::L_HIP_PITCH, -hipPitch));
            positions.push_back(std::make_pair(ServoID::L_KNEE, M_PI - knee));
            positions.push_back(std::make_pair(ServoID::L_ANKLE_PITCH, -anklePitch));
            positions.push_back(std::make_pair(ServoID::L_ANKLE_ROLL, ankleRoll));
        } else {
            positions.push_back(std::make_pair(ServoID::R_HIP_YAW, -hipYaw));
            positions.push_back(std::make_pair(ServoID::R_HIP_ROLL, -hipRoll));
            positions.push_back(std::make_pair(ServoID::R_HIP_PITCH, -hipPitch));
            positions.push_back(std::make_pair(ServoID::R_KNEE, M_PI - knee));
            positions.push_back(std::make_pair(ServoID::R_ANKLE_PITCH, -anklePitch));
            positions.push_back(std::make_pair(ServoID::R_ANKLE_ROLL, -ankleRoll));
        }

        return positions;
    }

} // kinematics
} // motion
} // utility