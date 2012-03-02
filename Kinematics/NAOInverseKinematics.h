#ifndef H_NAOINVERSEKINEMATICS_H_DEFINED
#define H_NAOINVERSEKINEMATICS_H_DEFINED

/**
 * @file InverseKinematic.h
 * @author Alexander H‰rtl
 * @author jeff
 */

/*!
    @file NAOInverseKinematics.h
    @author Alexander H‰rtl
    @author jeff
    @author Steven Nicklin
 
    @brief Declaration of NAOInverseKinematics class
    Heavily modified version of the B-Human code release 2011 inverse kinematics, modified to be used within the NUbots
    robot software framework.
    @class NAOInverseKinematics
    @brief Implementation of inverse kinematics calculations for the NAO robot.
 */

#include "Tools/Math/Matrix.h"
#include "NUInverseKinematics.h"
#include "Tools/Math/TransformMatrices.h"
#include "Tools/Math/General.h"
#include "Tools/Math/StlVector.h"
#include <iostream>

// NUbot order: 0.HeadPitch, 1.HeadYaw, 2.LShoulderRoll, 3.LShoulderPitch, 4.LElbowRoll, 5.LElbowYaw, 6.RShoulderRoll, 7.RShoulderPitch, 8.RElbowRoll, 9.RElbowYaw, 10.LHipRoll, 11.LHipPitch, 12.LHipYawPitch, 13.LKneePitch, 14.LAnkleRoll, 15.LAnklePitch, 16.RHipRoll, 17.RHipPitch, 18.RHipYawPitch, 19.RKneePitch, 20.RAnkleRoll, 21.RAnklePitch
// B-Human order: 0.HeadYaw, 1.HeadPitch, 2.LShoulderPitch, 3.LShoulderRoll, 4.LElbowYaw, 5.LElbowRoll, 6.RShoulderPitch, 7.RShoulderRoll, 8.RElbowYaw, 9.RElbowRoll, 10.LHipYawPitch, 11.LHipRoll, 12.LHipPitch, 13.LKneePitch, 14.LAnklePitch, 15.LAnkleRoll, 16.RHipYawPitch, 17.RHipRoll, 18.RHipPitch, 19.RKneePitch, 20.RAnklePitch, 21.RAnkleRoll

namespace Joint
{
    enum
    {
        HeadPitch,
        HeadYaw,
        LShoulderRoll,
        LShoulderPitch,
        LElbowRoll,
        LElbowYaw,
        RShoulderRoll,
        RShoulderPitch,
        RElbowRoll,
        RElbowYaw,
        LHipRoll,
        LHipPitch,
        LHipYawPitch,
        LKneePitch,
        LAnkleRoll,
        LAnklePitch,
        RHipRoll,
        RHipPitch,
        RHipYawPitch,
        RKneePitch,
        RAnkleRoll,
        RAnklePitch
    };
}

class NAOInverseKinematics: public NUInverseKinematics
{
public:

    void test()
    {
        std::vector<float> joints(22,0.0f);
        NAOInverseKinematics kin;
        Matrix leftPosition(4,4,true);
        leftPosition[0][3] = 0;
        leftPosition[1][3] = 50;
        leftPosition[2][3] = -220;

        Matrix rightPosition(4,4,true);
        rightPosition[0][3] = 0;
        rightPosition[1][3] = -50;
        rightPosition[2][3] = -220;
        
        bool ret = kin.calculateLegJoints(leftPosition, rightPosition, joints);
        std::cout << "Position 1: " << std::endl;
        std::cout << "Left Position: " << std::endl << leftPosition << std::endl;
        std::cout << "Right Position: " << std::endl << rightPosition << std::endl;
        
        std::cout << "Joint Positions: " << std::endl << joints << std::endl;

        leftPosition[0][3] = 100;
        leftPosition[1][3] = 50;
        leftPosition[2][3] = -102.9;

        rightPosition[0][3] = 100;
        rightPosition[1][3] = -50;
        rightPosition[2][3] = -102.9;

        ret = kin.calculateLegJoints(leftPosition, rightPosition, joints);

        std::cout << "Position 2: " << std::endl;
        std::cout << "Left Position: " << std::endl << leftPosition << std::endl;
        std::cout << "Right Position: " << std::endl << rightPosition << std::endl;

        std::cout << "Joint Positions: " << std::endl << joints << std::endl;

        leftPosition[0][3] = 0;
        leftPosition[1][3] = 50;
        leftPosition[2][3] = -150;

        rightPosition[0][3] = 0;
        rightPosition[1][3] = -50;
        rightPosition[2][3] = -150;

        ret = kin.calculateLegJoints(leftPosition, rightPosition, joints);

        std::cout << "Position 3: " << std::endl;
        std::cout << "Left Position: " << std::endl << leftPosition << std::endl;
        std::cout << "Right Position: " << std::endl << rightPosition << std::endl;

        std::cout << "Joint Positions: " << std::endl << joints << std::endl;
        
    }
    bool calculateLegJoints(const Matrix& leftPosition, const Matrix& rightPosition, std::vector<float>& jointPositions)
    {
        bool targetReachable = true;

        if(!calculateLeg(leftPosition, jointPositions, true))
          targetReachable = false;
        if(!calculateLeg(rightPosition, jointPositions, false))
          targetReachable = false;

         // the hip joints of both legs must be equal, so it is computed as weighted mean and the foot positions are
         // recomputed with fixed joint0 and left open foot rotation (as possible failure)
        float joint0 = (jointPositions[Joint::LHipYawPitch] + jointPositions[Joint::RHipYawPitch]) * 0.5;
        if(!calculateLegFixedHip(leftPosition, jointPositions, joint0, true))
          targetReachable = false;
        if(!calculateLegFixedHip(rightPosition, jointPositions, joint0, false))
          targetReachable = false;

        return targetReachable;
    }

    bool calculateArmJoints(const Matrix& leftPosition, const Matrix& rightPosition, std::vector<float>& jointPositions)
    {
        bool targetReachable = true;
        return targetReachable;
    }
private:
    bool isInside(float t, float min, float max) const
    {return min <= max ? t >= min && t <= max : t >= min || t <= max;}
    float limit(float t, float min, float max) const {return t < min ? min : t > max ? max : t;}


    bool calculateLeg(const Matrix& position, std::vector<float>& jointPositions, bool isLeft)
    {
        const float lengthBetweenLegs = 100.0f;
        Matrix target(position);

        const int sign(isLeft ? -1 : 1);

        target[1][3] += sign * lengthBetweenLegs * 0.5;
        target = TransformMatrices::RotX(sign * mathGeneral::PI / 4.0f) * target;
        target = InverseMatrix(target);

        // Matrix access format is matrix[row][collumn]

        float length = sqrt(target[0][3]*target[0][3] + target[1][3]*target[1][3] + target[2][3]*target[2][3]);
        float sqrLength = length * length;
        float upperLeg = 100.0f;
        float sqrUpperLeg = upperLeg*upperLeg;
        float lowerLeg = 102.9f;
        float sqrLowerLeg = lowerLeg * lowerLeg;
        float cosLowerLeg = (sqrLowerLeg + sqrLength - sqrUpperLeg) / (2 * lowerLeg * length);
        float cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * upperLeg * lowerLeg);

        bool targetReachable = true;

        const float min = -1.0f;
        const float max = 1.0f;

        //!< TODO: Check if this cosLowerLeg shoud condition should be inverted.
        if(!isInside(cosKnee,min,max) || isInside(cosLowerLeg, min, max))
        {
//            std::cout << "calculateLeg: Not reachable - " << cosKnee << " - " << cosLowerLeg << std::endl;
//            std::cout << "cosKnee in range: " << (isInside(cosKnee,min,max)?"True":"False") << std::endl;
//            std::cout << "cosLowerLeg in range: " << (isInside(cosLowerLeg,min,max)?"True":"False") << std::endl;

          cosKnee = limit(cosKnee, min, max);
          cosLowerLeg = limit(cosLowerLeg, min, max);

//          std::cout << "linited values: " << cosKnee << " - " << cosLowerLeg << std::endl;

          targetReachable = false;
        }

        float joint3 = mathGeneral::PI - acos(cosKnee);
        float joint4 = -acos(cosLowerLeg);
        double yzabs = sqrt(target[1][3]*target[1][3] + target[2][3]*target[2][3]);
        joint4 -= atan2(target[0][3], yzabs);
        float joint5 = atan2(target[1][3], target[2][3]) * sign;

        // calulate rotation matrix before hip joints
        Matrix hipFromFoot(4,4,true);
        hipFromFoot = TransformMatrices::RotY(-joint4-joint3) * TransformMatrices::RotX(-sign*joint5) * hipFromFoot;

        // compute rotation matrix for hip from rotation before hip and desired rotation
        Matrix hip = InverseMatrix(hipFromFoot) * target;

        // compute joints from rotation matrix using theorem of euler angles
        // see http://www.geometrictools.com/Documentation/EulerAngles.pdf
        // this is possible because of the known order of joints (z, x, y seen from body resp. y, x, z seen from foot)
        float joint1 = asin(-hip[1][2]) * -sign;
        joint1 -= mathGeneral::PI * 0.25; // because of the 45∞-rotational construction of the Nao legs
        float joint2 = -atan2(hip[0][2], hip[2][2]);
        float joint0 = atan2(hip[1][0], hip[1][1]) * -sign;

        // Since the joint ordering is different simplest way is to directly index.
        if(isLeft)
        {
            // set computed joints in jointData
            jointPositions[Joint::LHipYawPitch] = joint0;
            jointPositions[Joint::LHipRoll] = -joint1;
            jointPositions[Joint::LHipPitch] = joint2;
            jointPositions[Joint::LKneePitch] = joint3;
            jointPositions[Joint::LAnklePitch] = joint4;
            jointPositions[Joint::LAnkleRoll] = -joint5;
        }
        else
        {
            // set computed joints in jointData
            jointPositions[Joint::RHipYawPitch] = joint0;
            jointPositions[Joint::RHipRoll] = joint1;
            jointPositions[Joint::RHipPitch] = joint2;
            jointPositions[Joint::RKneePitch] = joint3;
            jointPositions[Joint::RAnklePitch] = joint4;
            jointPositions[Joint::RAnkleRoll] = joint5;
        }

        return targetReachable;
    }

    bool calculateLegFixedHip(const Matrix& position, std::vector<float>& jointPositions, float hipPosition, bool isLeft)
    {
        Matrix target(position);
        const int sign(isLeft ? -1 : 1);
        const float lengthBetweenLegs = 100.0f;
        const float joint0 = hipPosition;

        target[1][3] += sign * lengthBetweenLegs * 0.5;
        target = TransformMatrices::RotZ(-sign*joint0) * TransformMatrices::RotX(sign*mathGeneral::PI*0.25) * target; // compute residual transformation with fixed joint0

        float length = sqrt(target[0][3]*target[0][3] + target[1][3]*target[1][3] + target[2][3]*target[2][3]);
        float sqrLength = length * length;
        float upperLeg = 100.0f;
        float sqrUpperLeg = upperLeg * upperLeg;
        float lowerLeg = 102.9f;
        float sqrLowerLeg = lowerLeg * lowerLeg;
        float cosUpperLeg = (sqrUpperLeg + sqrLength - sqrLowerLeg) / (2 * upperLeg * length);
        float cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * upperLeg * lowerLeg);

        bool targetReachable = true;

        const float min = -1.0f;
        const float max = 1.0f;

        //!< TODO: Check if this cosLowerLeg shoud condition should be inverted.
        if(!isInside(cosKnee,min,max) || isInside(cosUpperLeg, min, max))
        {
//            std::cout << "Length: " << length << std::endl;
//            std::cout << "calculateLegFixedHip: Not reachable - " << cosKnee << " - " << cosUpperLeg << std::endl;
//            std::cout << "cosKnee in range: " << (isInside(cosKnee,min,max)?"True":"False") << std::endl;
//            std::cout << "cosUpperLeg in range: " << (isInside(cosUpperLeg,min,max)?"True":"False") << std::endl;

          cosKnee = limit(cosKnee, min, max);
          cosUpperLeg = limit(cosUpperLeg, min, max);

//          std::cout << "linited values: " << cosKnee << " - " << cosUpperLeg << std::endl;

          targetReachable = false;
        }

        float joint1 = target[2][3] == 0.0f ? 0.0f : atan(target[1][3] / -target[2][3]) * sign;
        float joint2 = -acos(cosUpperLeg);
        double yzabs = sqrt(target[1][3]*target[1][3] + target[2][3]*target[2][3]);
        joint2 -= atan2(target[0][3], yzabs * -mathGeneral::sign(target[2][3]));
        float joint3 = mathGeneral::PI - acos(cosKnee);

        //Matrix beforeFoot = TransformMatrices::RotY(joint2 + joint3) * TransformMatrices::RotX(joint1*sign);
        Matrix beforeFoot = TransformMatrices::RotX(joint1*sign) * TransformMatrices::RotY(joint2 + joint3);
        joint1 -= mathGeneral::PI * 0.25; // because of the strange hip of Nao

        // compute joints from rotation matrix using theorem of euler angles
        // see http://www.geometrictools.com/Documentation/EulerAngles.pdf
        // this is possible because of the known order of joints (y, x, z) where z is left open and is seen as failure
        Matrix foot = InverseMatrix(beforeFoot) * target;

        float joint5 = asin(-foot[1][2]) * sign;
        float joint4 = -atan2(foot[0][2], foot[2][2]) * -1;

        // Since the joint ordering is different simplest way is to directly index.
        if(isLeft)
        {
            // set computed joints in jointData
            jointPositions[Joint::LHipYawPitch] = joint0;
            jointPositions[Joint::LHipRoll] = -joint1;
            jointPositions[Joint::LHipPitch] = joint2;
            jointPositions[Joint::LKneePitch] = joint3;
            jointPositions[Joint::LAnklePitch] = joint4;
            jointPositions[Joint::LAnkleRoll] = -joint5;
        }
        else
        {
            // set computed joints in jointData
            jointPositions[Joint::RHipYawPitch] = joint0;
            jointPositions[Joint::RHipRoll] = joint1;
            jointPositions[Joint::RHipPitch] = joint2;
            jointPositions[Joint::RKneePitch] = joint3;
            jointPositions[Joint::RAnklePitch] = joint4;
            jointPositions[Joint::RAnkleRoll] = joint5;
        }

        return targetReachable;
    }
};

#endif
