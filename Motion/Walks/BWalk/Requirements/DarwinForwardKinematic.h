/**
 * @file ForwardKinematic.h
 * @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include "Pose3D.h"
#include "MassCalibration.h"
#include "Tools/Math/General.h"

//float lengthBetweenLegs = 100;
//float upperLegLength = 100;
//float lowerLegLength = 102.9; // 102.75 / 102.74
//float heightLeg5Joint = 45.19;
//float zLegJoint1ToHeadPan = 211.5;
//float xHeadTiltToCamera = 48.8;
//float zHeadTiltToCamera = 23.81;
//float headTiltToCameraTilt = 0.698132;
//float xHeadTiltToUpperCamera = 53.9;
//float zHeadTiltToUpperCamera = 67.9;
//float headTiltToUpperCameraTilt = 0;
//Vector3<> armOffset(0, 98,185);
//float yElbowShoulder = 15;
//float upperArmLength = 105;
//float lowerArmLength = 130; // estimated
//float imageRecordingTime = 0.034;
//float imageRecordingDelay = -0.003;

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
        LHipYaw,
        LKneePitch,
        LAnkleRoll,
        LAnklePitch,
        RHipRoll,
        RHipPitch,
        RHipYaw,
        RKneePitch,
        RAnkleRoll,
        RAnklePitch,
        total_joints
    };
}

class ForwardKinematic
{
public:
  static void calculateArmChain(bool left, const std::vector<float>& joints, const MassCalibration& massCalibration, Pose3D limbs[MassCalibration::numOfLimbs])
  {
    Vector3<> armOffset(0, 98,185);
    float yElbowShoulder = 15;
    float upperArmLength = 105;

    int sign = left ? -1 : 1;
    float joint0, joint1, joint2, joint3;
    // Since the joint ordering is different simplest way is to directly index.
    if(left)
    {
        // set computed joints in jointData
        joint0 = joints[Joint::LShoulderPitch];
        joint1 = joints[Joint::LShoulderRoll];
        joint2 = joints[Joint::LElbowYaw];
        joint3 = joints[Joint::LElbowRoll];
    }
    else
    {
        joint0 = joints[Joint::RShoulderPitch];
        joint1 = joints[Joint::RShoulderRoll];
        joint2 = joints[Joint::RElbowYaw];
        joint3 = joints[Joint::RElbowRoll];
    }

    MassCalibration::Limb shoulder = left ? MassCalibration::shoulderLeft : MassCalibration::shoulderRight;

    limbs[shoulder + 0] = Pose3D( armOffset.x, armOffset.y * -sign, armOffset.z)
                          .rotateY(joint0);
    limbs[shoulder + 1] = Pose3D(limbs[shoulder + 0])
                          .rotateZ(joint1);
    limbs[shoulder + 2] = Pose3D(limbs[shoulder + 1])
                          .translate(upperArmLength, yElbowShoulder * -sign, 0)
                          .rotateX(joint2);
    limbs[shoulder + 3] = Pose3D(limbs[shoulder + 2])
                          .rotateZ(joint3);
  }

  static void calculateLegChain(bool left, const std::vector<float>& joints, const MassCalibration& massCalibration, Pose3D limbs[MassCalibration::numOfLimbs])
  {
    float lengthBetweenLegs = 100;
    float upperLegLength = 100;
    float lowerLegLength = 102.9; // 102.75 / 102.74

    int sign = left ? -1 : 1;
    float joint0, joint1, joint2, joint3, joint4, joint5;

    // Since the joint ordering is different simplest way is to directly index.
    if(left)
    {
        // set computed joints in jointData
        joint0 = joints[Joint::LHipYawPitch];
        joint1 = -joints[Joint::LHipRoll];
        joint2 = joints[Joint::LHipPitch];
        joint3 = joints[Joint::LKneePitch];
        joint4 = joints[Joint::LAnklePitch];
        joint5 = -joints[Joint::LAnkleRoll];
    }
    else
    {
        joint0 = joints[Joint::RHipYawPitch];
        joint1 = joints[Joint::RHipRoll];
        joint2 = joints[Joint::RHipPitch];
        joint3 = joints[Joint::RKneePitch];
        joint4 = joints[Joint::RAnklePitch];
        joint5 = joints[Joint::RAnkleRoll];
    }

    MassCalibration::Limb pelvis = left ? MassCalibration::pelvisLeft : MassCalibration::pelvisRight;

    limbs[pelvis + 0] =  Pose3D(0, lengthBetweenLegs / 2.0f * -sign, 0)
                         .rotateX(-mathGeneral::PI/4.0f * sign)
                         .rotateZ(joint0 * sign)
                         .rotateX(mathGeneral::PI/4.0f * sign);
    limbs[pelvis + 1] = Pose3D(limbs[pelvis + 0])
                        .rotateX(joint1*sign);
    limbs[pelvis + 2] = Pose3D(limbs[pelvis + 1])
                        .rotateY(joint2);
    limbs[pelvis + 3] = Pose3D(limbs[pelvis + 2])
                        .translate(0, 0, -upperLegLength)
                        .rotateY(joint3);
    limbs[pelvis + 4] = Pose3D(limbs[pelvis + 3])
                        .translate(0, 0, -lowerLegLength)
                        .rotateY(joint4);
    limbs[pelvis + 5] = Pose3D(limbs[pelvis + 4])
                        .rotateX(joint5*sign);
  }

  static void calculateHeadChain(const std::vector<float>& joints, const MassCalibration& massCalibration, Pose3D limbs[MassCalibration::numOfLimbs])
  {
    float zLegJoint1ToHeadPan = 211.5;
    limbs[MassCalibration::neck] = Pose3D(0, 0, zLegJoint1ToHeadPan)
                                   .rotateZ(joints[Joint::HeadYaw]);
    limbs[MassCalibration::head] = Pose3D(limbs[MassCalibration::neck])
                                   .rotateY(joints[Joint::HeadPitch]);
  }
};
