/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGE_PLATFORM_DARWIN_KINEMATICS_MODEL_H
#define MESSAGE_PLATFORM_DARWIN_KINEMATICS_MODEL_H

#include <armadillo>

namespace message {
namespace platform {
namespace darwin {

    struct DarwinKinematicsModel {

        struct Leg {
            arma::vec3 hipOffset;
            float upperLegLength;
            float lowerLegLength;
            float heelLength;
            float lengthBetweenLegs;
            float footCentreToAnkleCentre;

            struct Foot {
                float width;
                float height;
                float length;
                float toeLength;
            } foot;

        } leg;

        struct Head {
            float cameraDeclinationAngleOffset;
            arma::vec3 neckToCamera;

            struct Neck {
                float length;
                arma::vec3 basePositionFromOrigin;
            } neck;

            struct HeadMovementLimits {
                arma::vec2 yaw;
                arma::vec2 pitch;
            } headMovementLimits;

        } head;

        struct Arm {
            float distanceBetweenShoulders;

            struct Shoulder {
                float length;
                float width;
                float height;
                arma::vec2 offset;
            } shoulder;

            struct UpperArm {
                float length;
                arma::vec2 offset;
            } upperArm;

            struct LowerArm {
                float length;
                arma::vec2 offset;
            } lowerArm;

        } arm;

        float teamDarwinChestToOrigin;

        struct MassModel {
            uint numberOfMasses;
            uint massRepresentationDimension;

            struct Masses {
                arma::vec4 leftShoulderRoll;
                arma::vec4 rightShoulderRoll;

                arma::vec4 leftShoulderPitch;
                arma::vec4 rightShoulderPitch;

                arma::vec4 leftElbow;
                arma::vec4 rightElbow;

                arma::vec4 leftHipRoll;
                arma::vec4 rightHipRoll;

                arma::vec4 leftHipPitch;
                arma::vec4 rightHipPitch;

                arma::vec4 leftHipYaw;
                arma::vec4 rightHipYaw;

                arma::vec4 leftKnee;
                arma::vec4 rightKnee;

                arma::vec4 leftAnkleRoll;
                arma::vec4 rightAnkleRoll;

                arma::vec4 leftAnklePitch;
                arma::vec4 rightAnklePitch;

                arma::vec4 headPitch;
                arma::vec4 headYaw;

                arma::vec4 torso;
            } masses;
        } massModel;
    };

}  // darwin
}  // platform
}  // message

#endif  // MESSAGE_PLATFORM_DARWIN_DARWINSENSORS_H
