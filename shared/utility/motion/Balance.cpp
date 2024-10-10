/*
 * MIT License
 *
 * Copyright (c) 2014 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "Balance.hpp"

#include "message/actuation/KinematicsModel.hpp"

namespace utility::motion {

    using LimbID = utility::input::LimbID;
    // using ServoID = utility::input::ServoID;
    using message::actuation::KinematicsModel;
    using message::input::Sensors;

    void Balancer::configure(const YAML::Node& config) {
        rotationPGain = config["angle_gain"]["p"].as<float>();
        rotationIGain = config["angle_gain"]["i"].as<float>();
        rotationDGain = config["angle_gain"]["d"].as<float>();

        hipRotationScale   = config["hip_rotation_scale"].as<float>();
        ankleRotationScale = config["ankle_rotation_scale"].as<float>();

        translationPGainX = config["translation_gain"]["X"]["p"].as<float>();
        translationDGainX = config["translation_gain"]["X"]["d"].as<float>();

        translationPGainY = config["translation_gain"]["Y"]["p"].as<float>();
        translationDGainY = config["translation_gain"]["Y"]["d"].as<float>();

        translationPGainZ = config["translation_gain"]["Z"]["p"].as<float>();
        translationDGainZ = config["translation_gain"]["Z"]["d"].as<float>();

        lastBalanceTime = NUClear::clock::now();
    }


    void Balancer::balance(const KinematicsModel& model,
                           Eigen::Isometry3f& footToTorso,
                           const LimbID& leg,
                           const Sensors& sensors) {

        // Goal is based on the support foot rotation.
        Eigen::AngleAxisf goalTorsoOrientation = Eigen::AngleAxisf(footToTorso.rotation().inverse());

        //------------------------------------
        // Rotation
        //------------------------------------

        // Robot coords in world (:Robot -> World)
        Eigen::Isometry3f Htw         = sensors.Htw.cast<float>();
        Eigen::AngleAxisf orientation = Eigen::AngleAxisf(Htw.rotation().inverse());

        // .eulerAngles(0, 1, 2) returns {roll, pitch, yaw}
        float orientationYaw = orientation.toRotationMatrix().eulerAngles(0, 1, 2)[2];

        // The nested AngleAxis creates a -orientationYaw radians rotation about the Z axis
        // The outside AngleAxis constructs an AngleAxis from the returned Quaternion type of the multiplication
        Eigen::AngleAxisf yawlessOrientation =
            Eigen::AngleAxisf(Eigen::AngleAxisf(-orientationYaw, Eigen::Vector3f::UnitZ()) * orientation);

        // Removes any yaw component
        float goalTorsoOrientationYaw = goalTorsoOrientation.toRotationMatrix().eulerAngles(0, 1, 2)[2];

        // Again the nested AngleAxis is a rotation -goalTorsoOrientation radians about the Z axis and the outer one
        // converts from Quaternion type to AngleAxis
        Eigen::AngleAxisf yawlessGoalOrientation = Eigen::AngleAxisf(
            Eigen::AngleAxisf(-goalTorsoOrientationYaw, Eigen::Vector3f::UnitZ()) * goalTorsoOrientation);

        // Error orientation maps: Goal -> Current
        Eigen::AngleAxisf errorOrientation = Eigen::AngleAxisf(yawlessOrientation * yawlessGoalOrientation.inverse());

        // Our goal position as a quaternions
        Eigen::Quaternion<float> errorQuaternion(errorOrientation);

        // Calculate our D error and I error
        Eigen::Quaternion<float> differential = lastErrorQuaternion.inverse() * errorQuaternion;

        // TODO(MotionTeam): LEARN HOW TO COMPUTE THE INTEGRAL TERM CORRECTLY
        // footGoalErrorSum = footGoalErrorSum.slerp(goalQuaternion * footGoalErrorSum, 1.0/90.0);

        // Apply the PID gains
        Eigen::AngleAxisf ankleRotation =
            Eigen::AngleAxisf(Eigen::Quaternion<float>::Identity().slerp(rotationPGain, errorQuaternion)
                              // * UnitQuaternion().slerp(footGoalErrorSum, rotationIGain)
                              * Eigen::Quaternion<float>::Identity().slerp(rotationDGain, differential));

        // Apply our rotation by scaling the thetas by the rotationScale parameters
        auto hipRotation = ankleRotation;
        ankleRotation    = Eigen::AngleAxisf(ankleRotationScale * ankleRotation.angle(), ankleRotation.axis());
        hipRotation      = Eigen::AngleAxisf(hipRotationScale * hipRotation.angle(), hipRotation.axis());


        // Apply this rotation goal to our position
        footToTorso.linear() = ankleRotation.toRotationMatrix() * footToTorso.rotation();

        // Get the position of our hip to rotate around
        Eigen::Isometry3f hip = Eigen::Isometry3f::Identity();
        hip.translation()     = Eigen::Vector3f(model.leg.HIP_OFFSET_X,
                                            model.leg.HIP_OFFSET_Y * (leg == LimbID::RIGHT_LEG ? -1 : 1),
                                            -model.leg.HIP_OFFSET_Z);

        // Rotate around our hip to apply a balance
        footToTorso = utility::math::transform::rotateLocal(footToTorso, hipRotation, hip);  // Lean against the motion

        // Store our current footToTorso for D calculations
        lastErrorQuaternion = errorQuaternion;

        //------------------------------------
        // Translation
        //------------------------------------
        // Get servo load signals
        // ServoID balanceServo = ServoID::L_ANKLE_PITCH;
        // if(leg == LimbID::RIGHT_LEG){
        //     balanceServo = ServoID::R_ANKLE_PITCH;
        // }
        // // float anklePitchTorque = sensors.servos[int(balanceServo)].load;

        // Get error signal
        //.eulerAngles(0, 1, 2) == {roll, pitch, yaw}
        double pitch_gyro = errorQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[1];
        double pitch      = pitch_gyro;  // anklePitchTorque;
        double roll       = errorQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[0];
        double total      = std::fabs(pitch_gyro) + std::fabs(roll);

        // Differentiate error signal
        auto now = NUClear::clock::now();
        double timeSinceLastMeasurement =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now - lastBalanceTime).count() * 1e-9;
        double newdPitch = timeSinceLastMeasurement != 0 ? (pitch - lastPitch) / timeSinceLastMeasurement
                                                         : 0;  // note that this is not a great computation of the diff
        double newdRoll  = timeSinceLastMeasurement != 0 ? (roll - lastRoll) / timeSinceLastMeasurement : 0;

        // Exponential filter for velocity
        dPitch = newdPitch * 0.1 + dPitch * 0.9;
        dRoll  = newdRoll * 0.1 + dRoll * 0.9;

        double dTotal = std::fabs(dPitch) + std::fabs(dRoll);

        lastPitch       = pitch;
        lastRoll        = roll;
        lastBalanceTime = now;

        // //Debug result
        // emit(graph("pitch error", pitch, dPitch));
        // emit(graph("pd translation", translationPGainX * sensors.bodyCentreHeight * pitch, translationDGainX *
        // sensors.bodyCentreHeight * dPitch));

        // Compute torso position adjustment
        Eigen::Vector3f torsoAdjustment_world =
            Eigen::Vector3f(
                -translationPGainX * sensors.Htw(2, 3) * pitch - translationDGainX * sensors.Htw(2, 3) * dPitch,
                translationPGainY * sensors.Htw(2, 3) * roll + translationDGainY * sensors.Htw(2, 3) * dRoll,
                -translationPGainZ * total - translationDGainY * dTotal)
                .cast<float>();

        // Apply opposite translation to the foot position
        footToTorso = footToTorso.translate(-torsoAdjustment_world);
    }
}  // namespace utility::motion
