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

#ifndef UTILITY_MOTION_BALANCE_HPP
#define UTILITY_MOTION_BALANCE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>
#include <yaml-cpp/yaml.h>

#include "message/input/Sensors.hpp"
#include "message/motion/KinematicsModel.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/math/matrix/transform.hpp"
//#include "utility/input/ServoID.hpp"

namespace utility {
namespace motion {

    template <typename Scalar>
    class Balancer {
    private:
        // Config
        Scalar rotationPGain = 0;
        Scalar rotationIGain = 0;
        Scalar rotationDGain = 0;

        Scalar translationPGainX = 0;
        Scalar translationPGainY = 0;
        Scalar translationPGainZ = 0;

        Scalar translationDGainX = 0;
        Scalar translationDGainY = 0;
        Scalar translationDGainZ = 0;

        Scalar ankleRotationScale = 0;
        Scalar hipRotationScale   = 0;

        // State
        Scalar dPitch    = 0;
        Scalar dRoll     = 0;
        Scalar lastPitch = 0;
        Scalar lastRoll  = 0;

        Eigen::Quaternion<Scalar> lastErrorQuaternion;
        NUClear::clock::time_point lastBalanceTime;

    public:
        Balancer() : lastErrorQuaternion(), lastBalanceTime() {}
        void configure(const YAML::Node& config) {
            rotationPGain = config["angle_gain"]["p"].as<Scalar>();
            rotationIGain = config["angle_gain"]["i"].as<Scalar>();
            rotationDGain = config["angle_gain"]["d"].as<Scalar>();

            hipRotationScale   = config["hip_rotation_scale"].as<Scalar>();
            ankleRotationScale = config["ankle_rotation_scale"].as<Scalar>();

            translationPGainX = config["translation_gain"]["X"]["p"].as<Scalar>();
            translationDGainX = config["translation_gain"]["X"]["d"].as<Scalar>();

            translationPGainY = config["translation_gain"]["Y"]["p"].as<Scalar>();
            translationDGainY = config["translation_gain"]["Y"]["d"].as<Scalar>();

            translationPGainZ = config["translation_gain"]["Z"]["p"].as<Scalar>();
            translationDGainZ = config["translation_gain"]["Z"]["d"].as<Scalar>();

            lastBalanceTime = NUClear::clock::now();
        }

        void balance(const message::motion::KinematicsModel& model,
                     Eigen::Transform<Scalar, 3, Eigen::Affine>& footToTorso,
                     const utility::input::LimbID& leg,
                     const message::input::Sensors& sensors) {

            // Goal is based on the support foot rotation.
            Eigen::AngleAxis<Scalar> goalTorsoOrientation = Eigen::AngleAxis<Scalar>(footToTorso.rotation().inverse());

            //------------------------------------
            // Rotation
            //------------------------------------

            // Robot coords in world (:Robot -> World)
            Eigen::Transform<Scalar, 3, Eigen::Affine> Htw(sensors.Htw);
            Eigen::AngleAxis<Scalar> orientation = Eigen::AngleAxis<Scalar>(Htw.rotation().inverse());

            // .eulerAngles(0, 1, 2) returns {roll, pitch, yaw}
            Scalar orientationYaw = orientation.toRotationMatrix().eulerAngles(0, 1, 2)[2];

            // The nested AngleAxis creates a -orientationYaw radians rotation about the Z axis
            // The outside AngleAxis constructs an AngleAxis from the returned Quaternion type of the multiplication
            Eigen::AngleAxis<Scalar> yawlessOrientation = Eigen::AngleAxis<Scalar>(
                Eigen::AngleAxis<Scalar>(-orientationYaw, Eigen::Matrix<Scalar, 3, 1>::UnitZ()) * orientation);

            // Removes any yaw component
            Scalar goalTorsoOrientationYaw = goalTorsoOrientation.toRotationMatrix().eulerAngles(0, 1, 2)[2];

            // Again the nested AngleAxis is a rotation -goalTorsoOrientation radians about the Z axis and the outer one
            // converts from Quaternion type to AngleAxis
            Eigen::AngleAxis<Scalar> yawlessGoalOrientation = Eigen::AngleAxis<Scalar>(
                Eigen::AngleAxis<Scalar>(-goalTorsoOrientationYaw, Eigen::Matrix<Scalar, 3, 1>::UnitZ())
                * goalTorsoOrientation);

            // Error orientation maps: Goal -> Current
            Eigen::AngleAxis<Scalar> errorOrientation =
                Eigen::AngleAxis<Scalar>(yawlessOrientation * yawlessGoalOrientation.inverse());

            // Our goal position as a quaternions
            Eigen::Quaternion<Scalar> errorQuaternion(errorOrientation);

            // Calculate our D error and I error
            Eigen::Quaternion<Scalar> differential = lastErrorQuaternion.inverse() * errorQuaternion;

            // TODO: LEARN HOW TO COMPUTE THE INTEGRAL TERM CORRECTLY
            // footGoalErrorSum = footGoalErrorSum.slerp(goalQuaternion * footGoalErrorSum, 1.0/90.0);

            // emit(graph("pid", Rotation3D(goalQuaternion).pitch(), Rotation3D(footGoalErrorSum).pitch(),
            // Rotation3D(error).pitch()));

            // Apply the PID gains
            Eigen::AngleAxis<Scalar> ankleRotation =
                Eigen::AngleAxis<Scalar>(Eigen::Quaternion<Scalar>::Identity().slerp(rotationPGain, errorQuaternion)
                                         // * UnitQuaternion().slerp(footGoalErrorSum, rotationIGain)
                                         * Eigen::Quaternion<Scalar>::Identity().slerp(rotationDGain, differential));

            // Apply our rotation by scaling the thetas by the rotationScale parameters
            auto hipRotation = ankleRotation;
            ankleRotation = Eigen::AngleAxis<Scalar>(ankleRotationScale * ankleRotation.angle(), ankleRotation.axis());
            hipRotation   = Eigen::AngleAxis<Scalar>(hipRotationScale * hipRotation.angle(), hipRotation.axis());


            // Apply this rotation goal to our position
            footToTorso.linear() = ankleRotation.toRotationMatrix() * footToTorso.rotation();

            // Get the position of our hip to rotate around
            Eigen::Transform<Scalar, 3, Eigen::Affine> hip = Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity();
            hip.translation()                              = Eigen::Matrix<Scalar, 3, 1>(
                model.leg.HIP_OFFSET_X,
                model.leg.HIP_OFFSET_Y * (leg == utility::input::LimbID::RIGHT_LEG ? -1 : 1),
                -model.leg.HIP_OFFSET_Z);

            // Rotate around our hip to apply a balance
            footToTorso =
                utility::math::transform::rotateLocal(footToTorso, hipRotation, hip);  // Lean against the motion

            // Store our current footToTorso for D calculations
            lastErrorQuaternion = errorQuaternion;

            //------------------------------------
            // Translation
            //------------------------------------
            // Get servo load signals
            // ServoID balanceServo = ServoID::L_ANKLE_PITCH;
            // if(leg == utility::input::LimbID::RIGHT_LEG){
            //     balanceServo = ServoID::R_ANKLE_PITCH;
            // }
            // // Scalar anklePitchTorque = sensors.servos[int(balanceServo)].load;

            // Get error signal
            //.eulerAngles(0, 1, 2) == {roll, pitch, yaw}
            Scalar pitch_gyro = errorQuaternion.toRotationMatrix().eulerAngles(0, 1, 2).y();
            Scalar pitch      = pitch_gyro;  // anklePitchTorque;
            Scalar roll       = errorQuaternion.toRotationMatrix().eulerAngles(0, 1, 2).x();
            Scalar total      = std::fabs(pitch_gyro) + std::fabs(roll);

            // Differentiate error signal
            auto now = NUClear::clock::now();
            Scalar timeSinceLastMeasurement =
                std::chrono::duration_cast<std::chrono::nanoseconds>(now - lastBalanceTime).count() * 1e-9;
            Scalar newdPitch = timeSinceLastMeasurement != 0
                                   ? (pitch - lastPitch) / timeSinceLastMeasurement
                                   : 0;  // note that this is not a great computation of the diff
            Scalar newdRoll  = timeSinceLastMeasurement != 0 ? (roll - lastRoll) / timeSinceLastMeasurement : 0;

            // Exponential filter for velocity
            dPitch = newdPitch * 0.1 + dPitch * 0.9;
            dRoll  = newdRoll * 0.1 + dRoll * 0.9;

            Scalar dTotal = std::fabs(dPitch) + std::fabs(dRoll);

            lastPitch       = pitch;
            lastRoll        = roll;
            lastBalanceTime = now;

            // //Debug result
            // emit(graph("pitch error", pitch, dPitch));
            // emit(graph("pd translation", translationPGainX * sensors.bodyCentreHeight * pitch, translationDGainX *
            // sensors.bodyCentreHeight * dPitch));

            // Compute torso position adjustment
            Eigen::Matrix<Scalar, 3, 1> torsoAdjustment_world = Eigen::Matrix<Scalar, 3, 1>(
                -translationPGainX * sensors.Htw(2, 3) * pitch - translationDGainX * sensors.Htw(2, 3) * dPitch,
                translationPGainY * sensors.Htw(2, 3) * roll + translationDGainY * sensors.Htw(2, 3) * dRoll,
                -translationPGainZ * total - translationDGainY * dTotal);

            // //Rotate from world space to torso space
            // Rotation3D yawLessOrientation =
            // Rotation3D::createRotationZ(-Transform3D(convert(sensors.Htw)).rotation()).yaw()) *
            // Transform3D(convert(sensors.Htw)).rotation();

            // Apply opposite translation to the foot position
            footToTorso = footToTorso.translate(-torsoAdjustment_world);
        }
    };
}  // namespace motion
}  // namespace utility

#endif
