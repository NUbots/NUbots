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
#include "Balance.h"

#include "message/motion/KinematicsModel.h"
#include "utility/support/eigen_armadillo.h"

namespace utility {
namespace motion {

    using LimbID = utility::input::LimbID;
    // using ServoID = utility::input::ServoID;
    using message::input::Sensors;
    using message::motion::KinematicsModel;
    using utility::math::geometry::UnitQuaternion;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform3D;

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
                           Transform3D& footToTorso,
                           const LimbID& leg,
                           const Sensors& sensors) {

        // Goal is based on the support foot rotation.
        Rotation3D goalTorsoOrientation = footToTorso.rotation().i();

        //------------------------------------
        // Rotation
        //------------------------------------

        // Robot coords in world (:Robot -> World)
        Rotation3D orientation        = Transform3D(convert(sensors.Htw)).rotation().i();
        Rotation3D yawlessOrientation = Rotation3D::createRotationZ(-orientation.yaw()) * orientation;

        // Removes any yaw component
        Rotation3D yawlessGoalOrientation =
            Rotation3D::createRotationZ(-goalTorsoOrientation.yaw()) * goalTorsoOrientation;

        // Error orientation maps: Goal -> Current
        Rotation3D errorOrientation = yawlessOrientation * yawlessGoalOrientation.i();

        // Our goal position as a quaternions
        UnitQuaternion errorQuaternion(errorOrientation);

        // Calculate our D error and I error
        UnitQuaternion differential = lastErrorQuaternion.i() * errorQuaternion;

        // TODO: LEARN HOW TO COMPUTE THE INTEGRAL TERM CORRECTLY
        // footGoalErrorSum = footGoalErrorSum.slerp(goalQuaternion * footGoalErrorSum, 1.0/90.0);

        // emit(graph("pid", Rotation3D(goalQuaternion).pitch(), Rotation3D(footGoalErrorSum).pitch(),
        // Rotation3D(error).pitch()));

        // Apply the PID gains
        UnitQuaternion ankleRotation = UnitQuaternion().slerp(errorQuaternion, rotationPGain)
                                       // * UnitQuaternion().slerp(footGoalErrorSum, rotationIGain)
                                       * UnitQuaternion().slerp(differential, rotationDGain);

        // Apply our rotation
        auto hipRotation = ankleRotation;
        ankleRotation.scaleAngle(ankleRotationScale);
        hipRotation.scaleAngle(hipRotationScale);

        // Apply this rotation goal to our position
        footToTorso.rotation() = Rotation3D(ankleRotation) * footToTorso.rotation();

        // Get the position of our hip to rotate around

        Transform3D hip = Transform3D(arma::vec3({model.leg.HIP_OFFSET_X,
                                                  model.leg.HIP_OFFSET_Y * (leg == LimbID::RIGHT_LEG ? -1 : 1),
                                                  -model.leg.HIP_OFFSET_Z}));

        // Rotate around our hip to apply a balance
        footToTorso = footToTorso.rotateLocal(Rotation3D(hipRotation), hip);  // Lean against the motion

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
        double pitch_gyro = Rotation3D(errorQuaternion).pitch();
        double pitch      = pitch_gyro;  // anklePitchTorque;
        double roll       = Rotation3D(errorQuaternion).roll();
        double total      = std::fabs(pitch_gyro) + std::fabs(roll);

        // Differentiate error signal
        auto now = NUClear::clock::now();
        double timeSinceLastMeasurement =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now - lastBalanceTime).count() * 1e-9;
        double newdPitch = timeSinceLastMeasurement != 0 ? (pitch - lastPitch) / timeSinceLastMeasurement
                                                         : 0;  // note that this is not a great computation of the diff
        double newdRoll = timeSinceLastMeasurement != 0 ? (roll - lastRoll) / timeSinceLastMeasurement : 0;

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
        arma::vec3 torsoAdjustment_world = arma::vec3({-translationPGainX * sensors.body_centre_height * pitch
                                                           - translationDGainX * sensors.body_centre_height * dPitch,
                                                       translationPGainY * sensors.body_centre_height * roll
                                                           + translationDGainY * sensors.body_centre_height * dRoll,
                                                       -translationPGainZ * total - translationDGainY * dTotal});

        // //Rotate from world space to torso space
        // Rotation3D yawLessOrientation =
        // Rotation3D::createRotationZ(-Transform3D(convert(sensors.Htw)).rotation()).yaw()) *
        // Transform3D(convert(sensors.Htw)).rotation();

        arma::vec3 torsoAdjustment_torso = torsoAdjustment_world;

        // Apply opposite translation to the foot position
        footToTorso = footToTorso.translate(-torsoAdjustment_torso);
    }
}  // namespace motion
}  // namespace utility
