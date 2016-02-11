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

#ifndef MESSAGE_MOTION_FOOTMOTION_H
#define MESSAGE_MOTION_FOOTMOTION_H

#include <armadillo>
#include "utility/math/matrix/Transform2D.h"

namespace message 
{
namespace motion 
{
    using utility::math::matrix::Transform2D;

    struct FootMotionUpdate 
    {
        FootMotionUpdate() = delete;
            double phase;
            Transform2D leftFoot;
            Transform2D rightFoot;
        FootMotionUpdate(double phase, Transform2D leftFoot, Transform2D rightFoot)
            : phase(phase)
            , leftFoot(leftFoot)
            , rightFoot(rightFoot) {}
    };
}  // motion
}  // message

#endif  // MESSAGE_MOTION_FOOTMOTION_H