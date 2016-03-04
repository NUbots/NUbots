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

namespace message 
{
namespace motion 
{
    using utility::math::matrix::Transform2D;

    struct FootMotionStopped {};

    struct FootMotionUpdate 
    {
        double phase;
        Transform2D leftFoot;
        Transform2D rightFoot;
        FootMotionUpdate(double phase, const Transform2D& leftFoot, const Transform2D& rightFoot)
            : phase(phase)
            , leftFoot(leftFoot)
            , rightFoot(rightFoot) {}
    };

    struct FootStepCompleted
    {
        bool status;
        FootStepCompleted(bool status)
            : status(status) {}
    };

    struct EnableFootMotion 
    {
        EnableFootMotion(size_t id) : subsumptionId(id) { }
        size_t subsumptionId = 1;
    };

    struct DisableFootMotion
    {
        DisableFootMotion(size_t id) : subsumptionId(id) { }
        size_t subsumptionId = 1;
    };
}  // motion
}  // message

#endif  // MESSAGE_MOTION_FOOTMOTIONCOMMAND_H