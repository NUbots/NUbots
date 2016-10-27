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

#ifndef MESSAGE_MOTION_FOOTPLACEMENTCOMMAND_H
#define MESSAGE_MOTION_FOOTPLACEMENTCOMMAND_H

#include <armadillo>
#include "utility/math/matrix/Transform2D.h"

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

    struct FootPlacementStopped {};
    
    struct NewStepTargetInfo
    {
        double targetTime;
        LimbID activeForwardLimb;
        Transform2D velocityCurrent;
        Transform2D leftFootSource;
        Transform2D rightFootSource;
        Transform2D leftFootDestination;
        Transform2D rightFootDestination;
        Transform2D supportMass;
        NewStepTargetInfo(  double targetTime, 
                            const LimbID& activeForwardLimb,
                            const Transform2D& velocityCurrent,
                            const Transform2D& leftFootSource, 
                            const Transform2D& rightFootSource,
                            const Transform2D& leftFootDestination,
                            const Transform2D& rightFootDestination,
                            const Transform2D& supportMass)
            : targetTime(targetTime)
            , activeForwardLimb(activeForwardLimb)
            , velocityCurrent(velocityCurrent)
            , leftFootSource(leftFootSource)
            , rightFootSource(rightFootSource)
            , leftFootDestination(leftFootDestination)
            , rightFootDestination(rightFootDestination)
            , supportMass(supportMass) {}
    };
    struct EnableFootPlacement
    {
        EnableFootPlacement()  { }
    };
    struct DisableFootPlacement
    {
        DisableFootPlacement() { }
    };
}  // motion
}  // message

#endif  // MESSAGE_MOTION_FOOTPLACEMENTCOMMAND_H