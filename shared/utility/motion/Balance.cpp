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

    using messages::input::LimbID;
    using messages::input::Sensors;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform3D;
    using utility::math::geometry::UnitQuaternion;
    using utility::motion::kinematics::DarwinModel;

    template <class RobotModel>
    void Balancer<RobotModel>::balance(Transform3D& footToTorso, const LimbID& leg, const Sensors& sensors) {

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

        //Error orientation maps: Current -> Goal
        Rotation3D errorOrientation = yawlessGoalOrientation * yawlessOrientation.i();

        // Our goal position as a quaternions
        UnitQuaternion errorQuaternion(errorOrientation);

        // Calculate our D error and I error
        UnitQuaternion differential = lastErrorQuaternion.i() * errorQuaternion;

        //TODO: LEARN HOW TO COMPUTE THE INTEGRAL TERM CORRECTLY
        // footGoalErrorSum = footGoalErrorSum.slerp(goalQuaternion * footGoalErrorSum, 1.0/90.0);

        // emit(graph("pid", Rotation3D(goalQuaternion).pitch(), Rotation3D(footGoalErrorSum).pitch(), Rotation3D(error).pitch()));

        // Apply the PID gains
        UnitQuaternion rotation = UnitQuaternion().slerp(errorQuaternion, rotationPGain)
                                // * UnitQuaternion().slerp(footGoalErrorSum, rotationIGain)
                                * UnitQuaternion().slerp(differential, rotationDGain);

        // Halve our correction (so the other half is applied at the hip)
        rotation.scaleAngle(0.5);

        // Apply this rotation goal to our position
        footToTorso.rotation() = Rotation3D(rotation) * footToTorso.rotation();

        // Get the position of our hip to rotate around
        //TODO: template with model
        Transform3D hip = Transform3D(arma::vec3({
            RobotModel::Leg::HIP_OFFSET_X,
            RobotModel::Leg::HIP_OFFSET_Y * (leg == LimbID::RIGHT_LEG ? -1 : 1),
            -RobotModel::Leg::HIP_OFFSET_Z
        }));

        // Rotate around our hip to apply a balance
        footToTorso = footToTorso.rotateLocal(Rotation3D(rotation).i(), hip);

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

        double newdPitch = (pitch - lastPitch) / timeSinceLastMeasurement; //note that this is not a great computation of the diff
        double newdRoll = (roll - lastRoll) / timeSinceLastMeasurement;

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





