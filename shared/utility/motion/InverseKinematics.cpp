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

#include "InverseKinematics.h"

#include "utility/math/matrix.h"

namespace utility {
namespace motion {
namespace kinematics {

	using messages::input::ServoID;
	using utility::math::matrix::xRotationMatrix;
	using utility::math::matrix::yRotationMatrix;
	
	std::vector<std::pair<ServoID, float>> calculateLegJoints(arma::mat44 target, bool isLeft) {

		static const float LENGTH_BETWEEN_LEGS = 74.0f;
		static const float UPPER_LEG_LENGTH = 93.0f;
		static const float LOWER_LEG_LENGTH = 93.0f;
		static const double PI = arma::math::pi();

		const int sign(isLeft ? -1 : 1);
		std::vector<std::pair<ServoID, float>> positions;

		// add a translation in the y axis
        target(1,3) += sign * LENGTH_BETWEEN_LEGS / 2; // Shift to leg offset.
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
        if (!isInside(cosKnee, min, max) || isInside(cosLowerLeg, min, max)) {
			cosKnee = limit(cosKnee, min, max);
			cosLowerLeg = limit(cosLowerLeg, min, max);
			throw "Target angle unreachable";
        }

        float joint3 = PI - acos(cosKnee);
        float joint4 = -acos(cosLowerLeg);
        double yzabs = sqrt(target(1,3)*target(1,3) + target(2,3)*target(2,3));
        joint4 -= atan2(target(0,3), yzabs);
        float joint5 = atan2(target(1,3), target(2,3)) * sign;

        // calulate rotation matrix before hip joints
		arma::mat44 hipFromFoot;
		hipFromFoot.eye();

        //hipFromFoot = TransformMatrices::RotY(-joint4-joint3) * TransformMatrices::RotX(-sign*joint5) * hipFromFoot;
        hipFromFoot = hipFromFoot * xRotationMatrix(-sign * joint5, 4) * yRotationMatrix(-joint4 - joint3, 4);

        // compute rotation matrix for hip from rotation before hip and desired rotation
        arma::mat44 hip = arma::inv(hipFromFoot) * target;

        // compute joints from rotation matrix using theorem of euler angles
        // see http://www.geometrictools.com/Documentation/EulerAngles.pdf
        // this is possible because of the known order of joints (z, x, y seen from body resp. y, x, z seen from foot)
        float joint1 = asin(-hip(1,2)) * -sign;
        float joint2 = -atan2(hip(0,2), hip(2,2));
        float joint0 = -atan2(hip(1,0), hip(1,1));

        // Since the joint ordering is different simplest way is to directly index.
        /*if(isLeft) {
            // set computed joints in jointData
            jointPositions[DarwinJoint::LHipYaw] = joint0;
            jointPositions[DarwinJoint::LHipRoll] = -joint1;
            jointPositions[DarwinJoint::LHipPitch] = joint2;
            jointPositions[DarwinJoint::LKneePitch] = joint3;
            jointPositions[DarwinJoint::LAnklePitch] = joint4;
            jointPositions[DarwinJoint::LAnkleRoll] = -joint5;
        } else {
            // set computed joints in jointData
            jointPositions[DarwinJoint::RHipYaw] = joint0;
            jointPositions[DarwinJoint::RHipRoll] = joint1;
            jointPositions[DarwinJoint::RHipPitch] = joint2;
            jointPositions[DarwinJoint::RKneePitch] = joint3;
            jointPositions[DarwinJoint::RAnklePitch] = joint4;
            jointPositions[DarwinJoint::RAnkleRoll] = joint5;
        }*/
		
		//std::cout << positions << std::endl;
		std::cout << "Joint 0: " << joint0 << std::endl;
		std::cout << "Joint 1: " << joint1 << std::endl;
		std::cout << "Joint 2: " << joint2 << std::endl;
		std::cout << "Joint 3: " << joint3 << std::endl;
		std::cout << "Joint 4: " << joint4 << std::endl;
		std::cout << "Joint 5: " << joint5 << std::endl;

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

} // kinematics
}  // motion
}  // utility