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
    using message::input::LimbID;
    using utility::math::matrix::Transform2D;

    struct FootPlacementStopped {};
    
    struct FootStepTarget
    {
        LimbID supportMass;
        double targetTime;
        Transform2D targetDestination;
        FootStepTarget(const LimbID& supportMass, 
                       double targetTime, 
                       const Transform2D& targetDestination)
            : supportMass(supportMass)
            , targetTime(targetTime)
            , targetDestination(targetDestination) {}
    };
    struct NewTargetInformation
    {
        Transform2D leftFootSource;
        Transform2D rightFootSource;
        Transform2D leftFootDestination;
        Transform2D rightFootDestination;
        Transform2D supportMass;
        NewTargetInformation(const Transform2D& leftFootSource, 
                             const Transform2D& rightFootSource,
                             const Transform2D& leftFootDestination,
                             const Transform2D& rightFootDestination,
                             const Transform2D& supportMass)
            : leftFootSource(leftFootSource)
            , rightFootSource(rightFootSource)
            , leftFootDestination(leftFootDestination)
            , rightFootDestination(rightFootDestination)
            , supportMass(supportMass) {}
    };
    struct EnableFootPlacement
    {
        EnableFootPlacement(size_t id) : subsumptionId(id) { }
        size_t subsumptionId = 1;
    };
    struct DisableFootPlacement
    {
        DisableFootPlacement(size_t id) : subsumptionId(id) { }
        size_t subsumptionId = 1;
    };
}  // motion
}  // message

#endif  // MESSAGE_MOTION_FOOTPLACEMENTCOMMAND_H