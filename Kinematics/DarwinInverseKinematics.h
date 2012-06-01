#ifndef H_DARWININVERSEKINEMATICS_H_DEFINED
#define H_DARWININVERSEKINEMATICS_H_DEFINED

/**
 * @file DarwinInverseKinematic.h
 * @author Steven Nicklin
 */

/*!
    @file DarwinInverseKinematics.h
    @author Steven Nicklin
 
    @brief Declaration of NAOInverseKinematics class
    Heavily modified version of the B-Human code release 2011 inverse kinematics, modified to be used within the NUbots
    robot software framework.
    @class NAOInverseKinematics
    @brief Implementation of inverse kinematics calculations for the NAO robot.
 */

#include "Tools/Math/Matrix.h"
#include "NUInverseKinematics.h"
#include "Kinematics.h"
#include "Tools/Math/TransformMatrices.h"
#include "Tools/Math/General.h"
#include "Tools/Math/StlVector.h"
#include <iostream>

// NUbot order: 0.HeadPitch, 1.HeadYaw, 2.LShoulderRoll, 3.LShoulderPitch, 4.LElbowRoll, 5.LElbowYaw, 6.RShoulderRoll, 7.RShoulderPitch, 8.RElbowRoll, 9.RElbowYaw, 10.LHipRoll, 11.LHipPitch, 12.LHipYawPitch, 13.LKneePitch, 14.LAnkleRoll, 15.LAnklePitch, 16.RHipRoll, 17.RHipPitch, 18.RHipYawPitch, 19.RKneePitch, 20.RAnkleRoll, 21.RAnklePitch
// B-Human order: 0.HeadYaw, 1.HeadPitch, 2.LShoulderPitch, 3.LShoulderRoll, 4.LElbowYaw, 5.LElbowRoll, 6.RShoulderPitch, 7.RShoulderRoll, 8.RElbowYaw, 9.RElbowRoll, 10.LHipYawPitch, 11.LHipRoll, 12.LHipPitch, 13.LKneePitch, 14.LAnklePitch, 15.LAnkleRoll, 16.RHipYawPitch, 17.RHipRoll, 18.RHipPitch, 19.RKneePitch, 20.RAnklePitch, 21.RAnkleRoll

namespace DarwinJoint
{
    enum
    {
        HeadPitch,
        HeadYaw,
        LShoulderRoll,
        LShoulderPitch,
        LElbowRoll,
        RShoulderRoll,
        RShoulderPitch,
        RElbowRoll,
        LHipRoll,
        LHipPitch,
        LHipYaw,
        LKneePitch,
        LAnkleRoll,
        LAnklePitch,
        RHipRoll,
        RHipPitch,
        RHipYaw,
        RKneePitch,
        RAnkleRoll,
        RAnklePitch
    };
}

class DarwinInverseKinematics: public NUInverseKinematics
{
public:

    void test()
    {
        std::vector<float> joints(20,0.0f);
        DarwinInverseKinematics kin;
        Matrix leftPosition(4,4,true);
        leftPosition[0][3] = 0;
        leftPosition[1][3] = 37;
        leftPosition[2][3] = -186;

        Matrix rightPosition(4,4,true);
        rightPosition[0][3] = 0;
        rightPosition[1][3] = -37;
        rightPosition[2][3] = -186;
        
        bool ret = kin.calculateLegJoints(leftPosition, rightPosition, joints);
        std::cout << "Position: Straight" << std::endl;
        std::cout << "Left Position: " << std::endl << leftPosition << std::endl;
        std::cout << "Right Position: " << std::endl << rightPosition << std::endl;
        std::cout << "Joint Positions: " << std::endl << joints << std::endl << std::endl;

        leftPosition[0][3] = 100;
        leftPosition[1][3] = 37;
        leftPosition[2][3] = -102.9;

        rightPosition[0][3] = 100;
        rightPosition[1][3] = -37;
        rightPosition[2][3] = -102.9;

        ret = kin.calculateLegJoints(leftPosition, rightPosition, joints);

        std::cout << "Position: Forward" << std::endl;
        std::cout << "Left Position: " << std::endl << leftPosition << std::endl;
        std::cout << "Right Position: " << std::endl << rightPosition << std::endl;
        std::cout << "Joint Positions: " << std::endl << joints << std::endl << std::endl;

        leftPosition[0][3] = 0;
        leftPosition[1][3] = 37;
        leftPosition[2][3] = -150;

        rightPosition[0][3] = 0;
        rightPosition[1][3] = -37;
        rightPosition[2][3] = -150;

        ret = kin.calculateLegJoints(leftPosition, rightPosition, joints);

        std::cout << "Position: Sit" << std::endl;
        std::cout << "Left Position: " << std::endl << leftPosition << std::endl;
        std::cout << "Right Position: " << std::endl << rightPosition << std::endl;
        std::cout << "Joint Positions: " << std::endl << joints << std::endl << std::endl;

        const float rot_angle = 1.5;
        leftPosition = TransformMatrices::RotZ(rot_angle);
        leftPosition[0][3] = 0;
        leftPosition[1][3] = 37;
        leftPosition[2][3] = -186;

        rightPosition = TransformMatrices::RotZ(-rot_angle);
        rightPosition[0][3] = 0;
        rightPosition[1][3] = -37;
        rightPosition[2][3] = -186;

        ret = kin.calculateLegJoints(leftPosition, rightPosition, joints);

        std::cout << "Position: Feet rotated out" << std::endl;
        std::cout << "Left Position: " << std::endl << leftPosition << std::endl;
        std::cout << "Right Position: " << std::endl << rightPosition << std::endl;
        std::cout << "Joint Positions: " << std::endl << joints << std::endl << std::endl;
    }
    bool calculateLegJoints(const Matrix& leftPosition, const Matrix& rightPosition, std::vector<float>& jointPositions)
    {
        bool targetReachable = true;

        if(!calculateLeg(leftPosition, jointPositions, true))
          targetReachable = false;
        if(!calculateLeg(rightPosition, jointPositions, false))
          targetReachable = false;
        return targetReachable;
    }

    bool calculateArmJoints(const Matrix& leftPosition, const Matrix& rightPosition, std::vector<float>& jointPositions)
    {
        bool targetReachable = false;
        return targetReachable;
    }
private:
    bool isInside(float t, float min, float max) const
    {return min <= max ? t >= min && t <= max : t >= min || t <= max;}
    float limit(float t, float min, float max) const {return t < min ? min : t > max ? max : t;}


    bool calculateLeg(const Matrix& position, std::vector<float>& jointPositions, bool isLeft)
    {
        const float lengthBetweenLegs = 74.0f;
        Matrix target(position);
        const double pi_4 = mathGeneral::PI / 4.0;

        const int sign(isLeft ? -1 : 1);

        target[1][3] += sign * lengthBetweenLegs * 0.5; // Shift to leg offset.
        //target = TransformMatrices::RotX(sign * pi_4) * target;
        target = InverseMatrix(target);

        // Matrix access format is matrix[row][collumn]

        float length = sqrt(target[0][3]*target[0][3] + target[1][3]*target[1][3] + target[2][3]*target[2][3]);
        float sqrLength = length * length;
        float upperLeg = 93.0f;
        float sqrUpperLeg = upperLeg*upperLeg;
        float lowerLeg = 93.0f;
        float sqrLowerLeg = lowerLeg * lowerLeg;
        float cosLowerLeg = (sqrLowerLeg + sqrLength - sqrUpperLeg) / (2 * lowerLeg * length);
        float cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * upperLeg * lowerLeg);

        bool targetReachable = true;

        const float min = -1.0f;
        const float max = 1.0f;

        //!< TODO: Check if this cosLowerLeg shoud condition should be inverted.
        if(!isInside(cosKnee,min,max) || isInside(cosLowerLeg, min, max))
        {
          cosKnee = limit(cosKnee, min, max);
          cosLowerLeg = limit(cosLowerLeg, min, max);
          targetReachable = false;
        }

        float joint3 = mathGeneral::PI - acos(cosKnee);
        float joint4 = -acos(cosLowerLeg);
        double yzabs = sqrt(target[1][3]*target[1][3] + target[2][3]*target[2][3]);
        joint4 -= atan2(target[0][3], yzabs);
        float joint5 = atan2(target[1][3], target[2][3]) * sign;

        // calulate rotation matrix before hip joints
        Matrix hipFromFoot(4,4,true);
        //hipFromFoot = TransformMatrices::RotY(-joint4-joint3) * TransformMatrices::RotX(-sign*joint5) * hipFromFoot;
        hipFromFoot = hipFromFoot * TransformMatrices::RotX(-sign*joint5) * TransformMatrices::RotY(-joint4-joint3);

        // compute rotation matrix for hip from rotation before hip and desired rotation
        Matrix hip = InverseMatrix(hipFromFoot) * target;

        // compute joints from rotation matrix using theorem of euler angles
        // see http://www.geometrictools.com/Documentation/EulerAngles.pdf
        // this is possible because of the known order of joints (z, x, y seen from body resp. y, x, z seen from foot)
        float joint1 = asin(-hip[1][2]) * -sign;
        //joint1 -= pi_4; // because of the 45∞-rotational construction of the Nao legs
        float joint2 = -atan2(hip[0][2], hip[2][2]);
        float joint0 = -atan2(hip[1][0], hip[1][1]);

        // Since the joint ordering is different simplest way is to directly index.
        if(isLeft)
        {
            // set computed joints in jointData
            jointPositions[DarwinJoint::LHipYaw] = joint0;
            jointPositions[DarwinJoint::LHipRoll] = -joint1;
            jointPositions[DarwinJoint::LHipPitch] = joint2;
            jointPositions[DarwinJoint::LKneePitch] = joint3;
            jointPositions[DarwinJoint::LAnklePitch] = joint4;
            jointPositions[DarwinJoint::LAnkleRoll] = -joint5;
        }
        else
        {
            // set computed joints in jointData
            jointPositions[DarwinJoint::RHipYaw] = joint0;
            jointPositions[DarwinJoint::RHipRoll] = joint1;
            jointPositions[DarwinJoint::RHipPitch] = joint2;
            jointPositions[DarwinJoint::RKneePitch] = joint3;
            jointPositions[DarwinJoint::RAnklePitch] = joint4;
            jointPositions[DarwinJoint::RAnkleRoll] = joint5;
        }

        return targetReachable;
    }
};

#endif
