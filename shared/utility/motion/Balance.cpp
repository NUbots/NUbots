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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */
#include "Balance.h"

namespace utility {
namespace motion {

    using message::input::LimbID;
    using message::input::Sensors;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform3D;
    using utility::math::geometry::UnitQuaternion;
    using utility::motion::kinematics::DarwinModel;

    void Balancer::configure(const YAML::Node& config) {
        rotationPGain = config["angle_gain"]["p"].as<float>();
        rotationIGain = config["angle_gain"]["i"].as<float>();
        rotationDGain = config["angle_gain"]["d"].as<float>();

        hipRotationScale = config["hip_rotation_scale"].as<float>();
        ankleRotationScale = config["ankle_rotation_scale"].as<float>();

        translationPGainX = config["translation_gain"]["X"]["p"].as<float>();
        translationDGainX = config["translation_gain"]["X"]["d"].as<float>();

        translationPGainY = config["translation_gain"]["Y"]["p"].as<float>();
        translationDGainY = config["translation_gain"]["Y"]["d"].as<float>();

        translationPGainZ = config["translation_gain"]["Z"]["p"].as<float>();
        translationDGainZ = config["translation_gain"]["Z"]["d"].as<float>();

        lastBalanceTime = NUClear::clock::now();
    }

    void Balancer::balance(Transform3D& footToTorso, const LimbID& leg, const Sensors& sensors) {

        //Goal is based on the support foot rotation.
        Rotation3D goalTorsoOrientation = footToTorso.rotation().i();

        //------------------------------------
        // Rotation
        //------------------------------------

        //Robot coords in world (:Robot -> World)
        Rotation3D orientation = sensors.orientation.i();
        Rotation3D yawlessOrientation = Rotation3D::createRotationZ(-orientation.yaw()) * orientation;

        // Removes any yaw component
        Rotation3D yawlessGoalOrientation = Rotation3D::createRotationZ(-goalTorsoOrientation.yaw()) * goalTorsoOrientation;

        //Error orientation maps: Goal -> Current
        Rotation3D errorOrientation = yawlessOrientation * yawlessGoalOrientation.i();

        // Our goal position as a quaternions
        UnitQuaternion errorQuaternion(errorOrientation);

        // Calculate our D error and I error
        UnitQuaternion differential = lastErrorQuaternion.i() * errorQuaternion;

        //TODO: LEARN HOW TO COMPUTE THE INTEGRAL TERM CORRECTLY
        // footGoalErrorSum = footGoalErrorSum.slerp(goalQuaternion * footGoalErrorSum, 1.0/90.0);

        // emit(graph("pid", Rotation3D(goalQuaternion).pitch(), Rotation3D(footGoalErrorSum).pitch(), Rotation3D(error).pitch()));

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
        //TODO: template with model
        Transform3D hip = Transform3D(arma::vec3({
            DarwinModel::Leg::HIP_OFFSET_X,
            DarwinModel::Leg::HIP_OFFSET_Y * (leg == LimbID::RIGHT_LEG ? -1 : 1),
            -DarwinModel::Leg::HIP_OFFSET_Z
        }));

        // Rotate around our hip to apply a balance
        footToTorso = footToTorso.rotateLocal(Rotation3D(hipRotation), hip); // Lean against the motion

        // Store our current footToTorso for D calculations
        lastErrorQuaternion = errorQuaternion;

        //------------------------------------
        // Translation
        //------------------------------------
        //Get error signal
        double pitch = Rotation3D(errorQuaternion).pitch();
        double roll = Rotation3D(errorQuaternion).roll();
        double total = std::fabs(pitch) + std::fabs(roll);

        //Differentiate error signal
        auto now = NUClear::clock::now();
        double timeSinceLastMeasurement = std::chrono::duration_cast<std::chrono::nanoseconds>(now - lastBalanceTime).count() * 1e-9;
        double newdPitch = timeSinceLastMeasurement != 0 ? (pitch - lastPitch) / timeSinceLastMeasurement : 0; //note that this is not a great computation of the diff
        double newdRoll = timeSinceLastMeasurement != 0 ? (roll - lastRoll) / timeSinceLastMeasurement : 0;

        //Exponential filter for velocity
        dPitch = newdPitch * 0.1 + dPitch * 0.9;
        dRoll = newdRoll * 0.1 + dRoll * 0.9;

        double dTotal = std::fabs(dPitch) + std::fabs(dRoll);

        lastPitch = pitch;
        lastRoll = roll;
        lastBalanceTime = now;

        // //Debug result
        // emit(graph("pitch error", pitch, dPitch));
        // emit(graph("pd translation", translationPGainX * sensors.bodyCentreHeight * pitch, translationDGainX * sensors.bodyCentreHeight * dPitch));

        //Compute torso position adjustment
        arma::vec3 torsoAdjustment_world = arma::vec3({- translationPGainX * sensors.bodyCentreHeight * pitch - translationDGainX * sensors.bodyCentreHeight * dPitch,
                                                         translationPGainY * sensors.bodyCentreHeight * roll + translationDGainY * sensors.bodyCentreHeight * dRoll,
                                                       - translationPGainZ * total - translationDGainY * dTotal});

        // //Rotate from world space to torso space
        // Rotation3D yawLessOrientation = Rotation3D::createRotationZ(-sensors.orientation.yaw()) * sensors.orientation;

        arma::vec3 torsoAdjustment_torso = torsoAdjustment_world;

        //Apply opposite translation to the foot position
        footToTorso = footToTorso.translate(-torsoAdjustment_torso);

    }
}
}





