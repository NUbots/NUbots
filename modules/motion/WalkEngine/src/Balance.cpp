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

#include "WalkEngine.h"

#include "utility/math/matrix/Rotation3D.h"
#include "utility/motion/RobotModels.h"
#include "utility/nubugger/NUhelpers.h"

namespace modules {
namespace motion {

    using messages::input::LimbID;
    using messages::input::ServoID;
    using messages::input::Sensors;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform3D;
    using utility::math::matrix::AxisAngle;
    using utility::math::geometry::UnitQuaternion;
    using utility::nubugger::graph;
    using utility::motion::kinematics::DarwinModel;

    void WalkEngine::balance(Transform3D& target, const LimbID& leg, const Sensors& sensors) {

        //------------------------------------
        // Rotation
        //------------------------------------

        // Get current orientation, offset by body tilt. Maps robot to world. 
        Rotation3D tiltedOrientation = sensors.orientation.i().rotateY(-bodyTilt);
        // Removes any yaw component
        Rotation3D goalOrientation = Rotation3D::createRotationZ(-tiltedOrientation.yaw()) * tiltedOrientation;

        // Our goal position as a quaternions
        UnitQuaternion goalQuaternion(goalOrientation);

        // Calculate our D error and I error
        UnitQuaternion error = lastFootGoalRotation.i() * goalQuaternion;
        //TODO: LEARN HOW TO COMPUTE THE INTEGRAL TERM CORRECTLY
        // footGoalErrorSum = footGoalErrorSum.slerp(goalQuaternion * footGoalErrorSum, 1.0/90.0);

        // emit(graph("pid", Rotation3D(goalQuaternion).pitch(), Rotation3D(footGoalErrorSum).pitch(), Rotation3D(error).pitch()));

        // Apply the PID gains
        UnitQuaternion rotation = UnitQuaternion().slerp(goalQuaternion, balancePGain)
                                // * UnitQuaternion().slerp(footGoalErrorSum, balanceIGain)
                                * UnitQuaternion().slerp(error, balanceDGain).i();

        // Halve our correction (so the other half is applied at the hip)
        rotation.scaleAngle(0.5);

        // Apply this rotation goal to our position
        target.rotation() = Rotation3D(rotation) * target.rotation();

        // Get the position of our hip to rotate around
        Transform3D hip = Transform3D(arma::vec3({
            DarwinModel::Leg::HIP_OFFSET_X,
            DarwinModel::Leg::HIP_OFFSET_Y * (leg == LimbID::RIGHT_LEG ? -1 : 1),
            -DarwinModel::Leg::HIP_OFFSET_Z
        }));

        // Rotate around our hip to apply a balance
        target = target.rotateLocal(Rotation3D(rotation).i(), hip);

        // Store our current target for D calculations
        lastFootGoalRotation = goalQuaternion;

        //------------------------------------
        // Translation 
        //------------------------------------
        //Get error signal
        double pitch = Rotation3D(goalQuaternion).pitch();
        double roll = Rotation3D(goalQuaternion).roll();
        double total = std::fabs(pitch) + std::fabs(roll);

        //Differentiate error signal
        auto now = NUClear::clock::now();
        double timeSinceLastMeasurement = std::chrono::duration_cast<std::chrono::nanoseconds>(now - lastBalanceTime).count() * 1e-9;

        double newdPitch = (pitch - lastPitch) / timeSinceLastMeasurement; //note that this is not a great computation of the diff
        double newdRoll = (roll - lastRoll) / timeSinceLastMeasurement;

        //Exponential filter
        dPitch = newdPitch * 0.1 + dPitch * 0.9;
        dRoll = newdRoll * 0.1 + dRoll * 0.9;

        double dTotal = std::fabs(dPitch) + std::fabs(dRoll);

        lastPitch = pitch;
        lastRoll = roll;
        lastBalanceTime = now;

        //Debug result
        emit(graph("pitch error", pitch, dPitch));
        emit(graph("pd translation", balanceTransPGainX * sensors.bodyCentreHeight * std::sin(pitch), balanceTransDGainX * sensors.bodyCentreHeight * dPitch));

        //Compute torso position adjustment
        arma::vec3 torsoAdjustment_world = arma::vec3({- balanceTransPGainX * sensors.bodyCentreHeight * std::sin(pitch) - balanceTransDGainX * sensors.bodyCentreHeight * dPitch,
                                                         balanceTransPGainY * sensors.bodyCentreHeight * std::sin(roll) + balanceTransDGainY * sensors.bodyCentreHeight * dRoll,
                                                       - balanceTransPGainZ * std::sin(total) + balanceTransDGainY * dTotal});

        //Rotate from world space to torso space
        Rotation3D yawLessOrientation = Rotation3D::createRotationZ(-sensors.orientation.yaw()) * sensors.orientation;

        arma::vec3 torsoAdjustment_torso = yawLessOrientation * torsoAdjustment_world;

        //Apply opposite translation to the foot position
        target = target.translate(-torsoAdjustment_torso);


        //-------------------------------
        // Gains
        //-------------------------------

        for(auto it = servoControlPGains.cbegin(); it != servoControlPGains.cend(); ++it){
            auto id = it->first;
            float gain = it->second;
            jointGains[id] = std::fmin(gainLegs, std::fabs(total * gain));
            emit(graph(messages::input::stringFromId(id), jointGains[id]));
        }


    }
}
}





