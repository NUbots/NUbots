// Gait utility class that provides joint pose representation functionality
// File: gait_joint_pose.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include "gait_joint_pose.h"

#include "gait_abstract_pose.h"
#include "gait_inverse_pose.h"
#include "utility/input/ServoID.h"

namespace gait {
namespace pose {

    using utility::input::ServoID;

    //
    // JointLegPose
    //

    // Set the joint leg pose to a given inverse leg pose
    void JointLegPose::setFromInversePose(const InverseLegPose& pose) {
        cld = pose.cld;
        pose.getJointAngles(hipYaw, hipRoll, hipPitch, kneePitch, anklePitch, ankleRoll);
    }

    // Set the joint leg pose to a given abstract leg pose
    void JointLegPose::setFromAbstractPose(const AbstractLegPose& pose) {
        cld = pose.cld;
        pose.getJointAngles(hipYaw, hipRoll, hipPitch, kneePitch, anklePitch, ankleRoll);
    }

    // Blend this joint pose towards another one by a given blending factor
    void JointLegPose::blendTowards(const JointLegPose& other, double b)  // b: 0 => No change, 1 => Replace by other
    {
        double invb          = 1.0 - b;
        hipYaw               = invb * hipYaw + b * other.hipYaw;
        hipRoll              = invb * hipRoll + b * other.hipRoll;
        hipPitch             = invb * hipPitch + b * other.hipPitch;
        kneePitch            = invb * kneePitch + b * other.kneePitch;
        anklePitch           = invb * anklePitch + b * other.anklePitch;
        ankleRoll            = invb * ankleRoll + b * other.ankleRoll;
        cld.effortHipYaw     = invb * cld.effortHipYaw + b * other.cld.effortHipYaw;
        cld.effortHipRoll    = invb * cld.effortHipRoll + b * other.cld.effortHipRoll;
        cld.effortHipPitch   = invb * cld.effortHipPitch + b * other.cld.effortHipPitch;
        cld.effortKneePitch  = invb * cld.effortKneePitch + b * other.cld.effortKneePitch;
        cld.effortAnklePitch = invb * cld.effortAnklePitch + b * other.cld.effortAnklePitch;
        cld.effortAnkleRoll  = invb * cld.effortAnkleRoll + b * other.cld.effortAnkleRoll;
        cld.supportCoeff     = invb * cld.supportCoeff + b * other.cld.supportCoeff;
    }

    //
    // JointArmPose
    //

    // Set the joint arm pose to a given inverse arm pose
    void JointArmPose::setFromInversePose(const InverseArmPose& pose) {
        cad = pose.cad;
        pose.getJointAngles(shoulderPitch, shoulderRoll, elbowPitch);
    }

    // Set the joint arm pose to a given abstract arm pose
    void JointArmPose::setFromAbstractPose(const AbstractArmPose& pose) {
        cad = pose.cad;
        pose.getJointAngles(shoulderPitch, shoulderRoll, elbowPitch);
    }

    // Blend this joint pose towards another one by a given blending factor
    void JointArmPose::blendTowards(const JointArmPose& other, double b)  // b: 0 => No change, 1 => Replace by other
    {
        double invb             = 1.0 - b;
        shoulderPitch           = invb * shoulderPitch + b * other.shoulderPitch;
        shoulderRoll            = invb * shoulderRoll + b * other.shoulderRoll;
        elbowPitch              = invb * elbowPitch + b * other.elbowPitch;
        cad.effortShoulderPitch = invb * cad.effortShoulderPitch + b * other.cad.effortShoulderPitch;
        cad.effortShoulderRoll  = invb * cad.effortShoulderRoll + b * other.cad.effortShoulderRoll;
        cad.effortElbowPitch    = invb * cad.effortElbowPitch + b * other.cad.effortElbowPitch;
    }

    //
    // JointPose
    //

    // Set the joint pose to a given inverse pose
    void JointPose::setFromInversePose(const InversePose& pose) {
        leftLeg.setFromInversePose(pose.leftLeg);
        rightLeg.setFromInversePose(pose.rightLeg);
        leftArm.setFromInversePose(pose.leftArm);
        rightArm.setFromInversePose(pose.rightArm);
    }

    // Set the joint pose to a given abstract pose
    void JointPose::setFromAbstractPose(const AbstractPose& pose) {
        leftLeg.setFromAbstractPose(pose.leftLeg);
        rightLeg.setFromAbstractPose(pose.rightLeg);
        leftArm.setFromAbstractPose(pose.leftArm);
        rightArm.setFromAbstractPose(pose.rightArm);
    }

    // Set the joint pose to the values specified by a given joint position array
    void JointPose::readJointPosArray(const std::array<double, ServoID::NUMBER_OF_SERVOS>& pos) {
        // Transcribe values for the left leg
        leftLeg.hipYaw     = pos[ServoID::L_HIP_YAW];
        leftLeg.hipRoll    = pos[ServoID::L_HIP_ROLL];
        leftLeg.hipPitch   = pos[ServoID::L_HIP_PITCH];
        leftLeg.kneePitch  = pos[ServoID::L_KNEE];
        leftLeg.anklePitch = pos[ServoID::L_ANKLE_PITCH];
        leftLeg.ankleRoll  = pos[ServoID::L_ANKLE_ROLL];

        // Transcribe values for the right leg
        rightLeg.hipYaw     = pos[ServoID::R_HIP_YAW];
        rightLeg.hipRoll    = pos[ServoID::R_HIP_ROLL];
        rightLeg.hipPitch   = pos[ServoID::R_HIP_PITCH];
        rightLeg.kneePitch  = pos[ServoID::R_KNEE];
        rightLeg.anklePitch = pos[ServoID::R_ANKLE_PITCH];
        rightLeg.ankleRoll  = pos[ServoID::R_ANKLE_ROLL];

        // Transcribe values for the left arm
        leftArm.shoulderPitch = pos[ServoID::L_SHOULDER_PITCH];
        leftArm.shoulderRoll  = pos[ServoID::L_SHOULDER_ROLL];
        leftArm.elbowPitch    = pos[ServoID::L_ELBOW];

        // Transcribe values for the right arm
        rightArm.shoulderPitch = pos[ServoID::R_SHOULDER_PITCH];
        rightArm.shoulderRoll  = pos[ServoID::R_SHOULDER_ROLL];
        rightArm.elbowPitch    = pos[ServoID::R_ELBOW];
    }

    // Set the joint efforts to the values specified by a given joint effort array
    void JointPose::readJointEffortArray(const std::array<double, utility::input::ServoID::NUMBER_OF_SERVOS>& effort) {
        // Transcribe values for the left leg
        leftLeg.cld.effortHipYaw     = effort[ServoID::L_HIP_YAW];
        leftLeg.cld.effortHipRoll    = effort[ServoID::L_HIP_ROLL];
        leftLeg.cld.effortHipPitch   = effort[ServoID::L_HIP_PITCH];
        leftLeg.cld.effortKneePitch  = effort[ServoID::L_KNEE];
        leftLeg.cld.effortAnklePitch = effort[ServoID::L_ANKLE_PITCH];
        leftLeg.cld.effortAnkleRoll  = effort[ServoID::L_ANKLE_ROLL];

        // Transcribe values for the right leg
        rightLeg.cld.effortHipYaw     = effort[ServoID::R_HIP_YAW];
        rightLeg.cld.effortHipRoll    = effort[ServoID::R_HIP_ROLL];
        rightLeg.cld.effortHipPitch   = effort[ServoID::R_HIP_PITCH];
        rightLeg.cld.effortKneePitch  = effort[ServoID::R_KNEE];
        rightLeg.cld.effortAnklePitch = effort[ServoID::R_ANKLE_PITCH];
        rightLeg.cld.effortAnkleRoll  = effort[ServoID::R_ANKLE_ROLL];

        // Transcribe values for the left arm
        leftArm.cad.effortShoulderPitch = effort[ServoID::L_SHOULDER_PITCH];
        leftArm.cad.effortShoulderRoll  = effort[ServoID::L_SHOULDER_ROLL];
        leftArm.cad.effortElbowPitch    = effort[ServoID::L_ELBOW];

        // Transcribe values for the right arm
        rightArm.cad.effortShoulderPitch = effort[ServoID::R_SHOULDER_PITCH];
        rightArm.cad.effortShoulderRoll  = effort[ServoID::R_SHOULDER_ROLL];
        rightArm.cad.effortElbowPitch    = effort[ServoID::R_ELBOW];
    }

    // Transcribe the stored joint pose to a joint position array
    std::array<double, utility::input::ServoID::NUMBER_OF_SERVOS> JointPose::writeJointPosArray() const {
        std::array<double, utility::input::ServoID::NUMBER_OF_SERVOS> pos;

        // Transcribe values for the left leg
        pos[ServoID::L_HIP_YAW]     = leftLeg.hipYaw;
        pos[ServoID::L_HIP_ROLL]    = leftLeg.hipRoll;
        pos[ServoID::L_HIP_PITCH]   = leftLeg.hipPitch;
        pos[ServoID::L_KNEE]        = leftLeg.kneePitch;
        pos[ServoID::L_ANKLE_PITCH] = leftLeg.anklePitch;
        pos[ServoID::L_ANKLE_ROLL]  = leftLeg.ankleRoll;

        // Transcribe values for the right leg
        pos[ServoID::R_HIP_YAW]     = rightLeg.hipYaw;
        pos[ServoID::R_HIP_ROLL]    = rightLeg.hipRoll;
        pos[ServoID::R_HIP_PITCH]   = rightLeg.hipPitch;
        pos[ServoID::R_KNEE]        = rightLeg.kneePitch;
        pos[ServoID::R_ANKLE_PITCH] = rightLeg.anklePitch;
        pos[ServoID::R_ANKLE_ROLL]  = rightLeg.ankleRoll;

        // Transcribe values for the left arm
        pos[ServoID::L_SHOULDER_PITCH] = leftArm.shoulderPitch;
        pos[ServoID::L_SHOULDER_ROLL]  = leftArm.shoulderRoll;
        pos[ServoID::L_ELBOW]          = leftArm.elbowPitch;

        // Transcribe values for the right arm
        pos[ServoID::R_SHOULDER_PITCH] = rightArm.shoulderPitch;
        pos[ServoID::R_SHOULDER_ROLL]  = rightArm.shoulderRoll;
        pos[ServoID::R_ELBOW]          = rightArm.elbowPitch;

        return pos;
    }

    // Transcribe the stored joint efforts to a joint effort array
    std::array<double, utility::input::ServoID::NUMBER_OF_SERVOS> JointPose::writeJointEffortArray() const {

        std::array<double, utility::input::ServoID::NUMBER_OF_SERVOS> effort;

        // Transcribe values for the left leg
        effort[ServoID::L_HIP_YAW]     = leftLeg.cld.effortHipYaw;
        effort[ServoID::L_HIP_ROLL]    = leftLeg.cld.effortHipRoll;
        effort[ServoID::L_HIP_PITCH]   = leftLeg.cld.effortHipPitch;
        effort[ServoID::L_KNEE]        = leftLeg.cld.effortKneePitch;
        effort[ServoID::L_ANKLE_PITCH] = leftLeg.cld.effortAnklePitch;
        effort[ServoID::L_ANKLE_ROLL]  = leftLeg.cld.effortAnkleRoll;

        // Transcribe values for the right leg
        effort[ServoID::R_HIP_YAW]     = rightLeg.cld.effortHipYaw;
        effort[ServoID::R_HIP_ROLL]    = rightLeg.cld.effortHipRoll;
        effort[ServoID::R_HIP_PITCH]   = rightLeg.cld.effortHipPitch;
        effort[ServoID::R_KNEE]        = rightLeg.cld.effortKneePitch;
        effort[ServoID::R_ANKLE_PITCH] = rightLeg.cld.effortAnklePitch;
        effort[ServoID::R_ANKLE_ROLL]  = rightLeg.cld.effortAnkleRoll;

        // Transcribe values for the left arm
        effort[ServoID::L_SHOULDER_PITCH] = leftArm.cad.effortShoulderPitch;
        effort[ServoID::L_SHOULDER_ROLL]  = leftArm.cad.effortShoulderRoll;
        effort[ServoID::L_ELBOW]          = leftArm.cad.effortElbowPitch;

        // Transcribe values for the right arm
        effort[ServoID::R_SHOULDER_PITCH] = rightArm.cad.effortShoulderPitch;
        effort[ServoID::R_SHOULDER_ROLL]  = rightArm.cad.effortShoulderRoll;
        effort[ServoID::R_ELBOW]          = rightArm.cad.effortElbowPitch;

        return effort;
    }

    // Blend this joint pose towards another one by a given blending factor
    void JointPose::blendTowards(const JointPose& other, double b)  // b: 0 => No change, 1 => Replace by other
    {
        leftLeg.blendTowards(other.leftLeg, b);
        rightLeg.blendTowards(other.rightLeg, b);
        leftArm.blendTowards(other.leftArm, b);
        rightArm.blendTowards(other.rightArm, b);
    }

}  // namespace pose
}  // namespace gait
