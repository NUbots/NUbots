/*
 * This file is part of WalkEngine.
 *
 * WalkEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * WalkEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WalkEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */
#include "Balance.hpp"

#include "message/actuation/KinematicsModel.hpp"

namespace utility::motion {

    using LimbID = utility::input::LimbID;
    // using ServoID = utility::input::ServoID;
    using message::actuation::KinematicsModel;
    using message::input::Sensors;

    void Balancer::configure(const YAML::Node& config) {
        rotationPGain = config["angle_gain"]["p"].as<double>();
        rotationIGain = config["angle_gain"]["i"].as<double>();
        rotationDGain = config["angle_gain"]["d"].as<double>();

        hipRotationScale   = config["hip_rotation_scale"].as<double>();
        ankleRotationScale = config["ankle_rotation_scale"].as<double>();

        translationPGainX = config["translation_gain"]["X"]["p"].as<double>();
        translationDGainX = config["translation_gain"]["X"]["d"].as<double>();

        translationPGainY = config["translation_gain"]["Y"]["p"].as<double>();
        translationDGainY = config["translation_gain"]["Y"]["d"].as<double>();

        translationPGainZ = config["translation_gain"]["Z"]["p"].as<double>();
        translationDGainZ = config["translation_gain"]["Z"]["d"].as<double>();

        lastBalanceTime = NUClear::clock::now();
    }


    void Balancer::balance(const KinematicsModel& model,
                           Eigen::Isometry3d& footToTorso,
                           const LimbID& leg,
                           const Sensors& sensors) {

        // Goal is based on the support foot rotation.
        Eigen::AngleAxisd goalTorsoOrientation = Eigen::AngleAxisd(footToTorso.rotation().inverse());

        //------------------------------------
        // Rotation
        //------------------------------------

        // Robot coords in world (:Robot -> World)
        Eigen::Isometry3d Htw         = Eigen::Isometry3d(sensors.Htw);
        Eigen::AngleAxisd orientation = Eigen::AngleAxisd(Htw.rotation().inverse());

        // .eulerAngles(0, 1, 2) returns {roll, pitch, yaw}
        double orientationYaw = orientation.toRotationMatrix().eulerAngles(0, 1, 2)[2];

        // The nested AngleAxis creates a -orientationYaw radians rotation about the Z axis
        // The outside AngleAxis constructs an AngleAxis from the returned Quaternion type of the multiplication
        Eigen::AngleAxisd yawlessOrientation =
            Eigen::AngleAxisd(Eigen::AngleAxisd(-orientationYaw, Eigen::Vector3d::UnitZ()) * orientation);

        // Removes any yaw component
        double goalTorsoOrientationYaw = goalTorsoOrientation.toRotationMatrix().eulerAngles(0, 1, 2)[2];

        // Again the nested AngleAxis is a rotation -goalTorsoOrientation radians about the Z axis and the outer one
        // converts from Quaternion type to AngleAxis
        Eigen::AngleAxisd yawlessGoalOrientation = Eigen::AngleAxisd(
            Eigen::AngleAxisd(-goalTorsoOrientationYaw, Eigen::Vector3d::UnitZ()) * goalTorsoOrientation);

        // Error orientation maps: Goal -> Current
        Eigen::AngleAxisd errorOrientation = Eigen::AngleAxisd(yawlessOrientation * yawlessGoalOrientation.inverse());

        // Our goal position as a quaternions
        Eigen::Quaternion<double> errorQuaternion(errorOrientation);

        // Calculate our D error and I error
        Eigen::Quaternion<double> differential = lastErrorQuaternion.inverse() * errorQuaternion;

        // TODO(MotionTeam): LEARN HOW TO COMPUTE THE INTEGRAL TERM CORRECTLY
        // footGoalErrorSum = footGoalErrorSum.slerp(goalQuaternion * footGoalErrorSum, 1.0/90.0);

        // Apply the PID gains
        Eigen::AngleAxisd ankleRotation =
            Eigen::AngleAxisd(Eigen::Quaternion<double>::Identity().slerp(rotationPGain, errorQuaternion)
                              // * UnitQuaternion().slerp(footGoalErrorSum, rotationIGain)
                              * Eigen::Quaternion<double>::Identity().slerp(rotationDGain, differential));

        // Apply our rotation by scaling the thetas by the rotationScale parameters
        auto hipRotation = ankleRotation;
        ankleRotation    = Eigen::AngleAxisd(ankleRotationScale * ankleRotation.angle(), ankleRotation.axis());
        hipRotation      = Eigen::AngleAxisd(hipRotationScale * hipRotation.angle(), hipRotation.axis());


        // Apply this rotation goal to our position
        footToTorso.linear() = ankleRotation.toRotationMatrix() * footToTorso.rotation();

        // Get the position of our hip to rotate around
        Eigen::Isometry3d hip = Eigen::Isometry3d::Identity();
        hip.translation()     = Eigen::Vector3d(model.leg.HIP_OFFSET_X,
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
        // // double anklePitchTorque = sensors.servos[int(balanceServo)].load;

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
        Eigen::Vector3d torsoAdjustment_world = Eigen::Vector3d(
            -translationPGainX * sensors.Htw(2, 3) * pitch - translationDGainX * sensors.Htw(2, 3) * dPitch,
            translationPGainY * sensors.Htw(2, 3) * roll + translationDGainY * sensors.Htw(2, 3) * dRoll,
            -translationPGainZ * total - translationDGainY * dTotal);

        // Apply opposite translation to the foot position
        footToTorso = footToTorso.translate(-torsoAdjustment_world);
    }
}  // namespace utility::motion
