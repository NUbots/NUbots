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

#ifndef MESSAGE_MOTION_FOOTMOTIONCOMMAND_H
#define MESSAGE_MOTION_FOOTMOTIONCOMMAND_H

#include <armadillo>
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"

namespace message 
{
namespace motion 
{
    /*
    struct [name]
    {
        [variables]
        [structure](...)
            : ...
            , ... {}
    };
    */

    using message::input::LimbID;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;

    struct FootMotionStopped {};

    struct FootMotionUpdate 
    {
        double phase;
        LimbID activeForwardLimb;
        Transform2D leftFoot2D;
        Transform2D rightFoot2D;
        Transform3D leftFoot3D;
        Transform3D rightFoot3D;
        FootMotionUpdate(
                            double phase, 
                            const LimbID& activeForwardLimb,
                            const Transform2D& leftFoot2D, 
                            const Transform2D& rightFoot2D, 
                            const Transform3D& leftFoot3D, 
                            const Transform3D& rightFoot3D
                        )
            : phase(phase)
            , activeForwardLimb(activeForwardLimb)
            , leftFoot2D(leftFoot2D)
            , rightFoot2D(rightFoot2D) 
            , leftFoot3D(leftFoot3D)
            , rightFoot3D(rightFoot3D) {}
    };

    struct NextFootTargetInfo
    {
        Transform2D leftFootSource;
        Transform2D rightFootSource;
        Transform2D supportMass;
        Transform2D leftFootDestination;
        Transform2D rightFootDestination;
        NextFootTargetInfo(
                            const Transform2D& inLeftFootSource,
                            const Transform2D& inRightFootSource,
                            const Transform2D& inSupportMass,
                            const Transform2D& inLeftFootDestination, 
                            const Transform2D& inRightFootDestination
                         )
            : leftFootSource(inLeftFootSource)
            , rightFootSource(inRightFootSource)
            , supportMass(inSupportMass)
            , leftFootDestination(inLeftFootDestination)
            , rightFootDestination(inRightFootDestination) {}
    };

    struct FootStepRequested
    {
        bool status;
        FootStepRequested(bool status)
            : status(status) {}
    };
    
    struct FootStepCompleted
    {
        bool status;
        FootStepCompleted(bool status)
            : status(status) {}
    };

    struct EnableFootMotion 
    {
        EnableFootMotion()  { }
    };

    struct DisableFootMotion
    {
        DisableFootMotion() { }
    };
}  // motion
}  // message

#endif  // MESSAGE_MOTION_FOOTMOTIONCOMMAND_H