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

#ifndef MESSAGE_MOTION_BALANCECOMMAND_H
#define MESSAGE_MOTION_BALANCECOMMAND_H

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

    struct BalanceBodyUpdate
    {
        double phase;
        Transform3D leftFoot;
        Transform3D rightFoot;
        arma::vec3 armLPosition;
        arma::vec3 armRPosition;
        BalanceBodyUpdate(
                            double phase,
                            const Transform3D& leftFoot,  
                            const Transform3D& rightFoot,
                            const arma::vec3& armLPosition,
                            const arma::vec3& armRPosition
                         )
            : phase(phase)
            , leftFoot(leftFoot)
            , rightFoot(rightFoot)
            , armLPosition(armLPosition) 
            , armRPosition(armRPosition) {}
    };

    struct EnableBalanceResponse
    {
        EnableBalanceResponse()  { }
    };
    struct DisableBalanceResponse
    {
        DisableBalanceResponse() { }
    };
}  // motion
}  // message

#endif  // MESSAGE_MOTION_BALANCECOMMAND_H