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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */
#ifndef UTILITY_LOCALISATION_TRANSFORM_H
#define UTILITY_LOCALISATION_TRANSFORM_H

#include <armadillo>

#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"

namespace utility {
namespace localisation {

    // Transforms the field state (x,y,theta) to the correct transform Hfw : World -> Field
    inline utility::math::matrix::Transform3D fieldStateToTransform3D(const arma::vec3& state) {
        utility::math::matrix::Transform3D Hfw;
        Hfw.translation() = arma::vec3({state[0], state[1], 0});
        Hfw               = Hfw.rotateZ(state[2]);
        return Hfw;
    }

    // Transforms the transform
    inline arma::vec3 transform3DToFieldState(const utility::math::matrix::Transform3D& m) {
        utility::math::matrix::Transform2D ax = m.projectTo2D(arma::vec3({0, 0, 1}), arma::vec3({1, 0, 0}));
        return arma::vec3({ax.x(), ax.y(), ax.angle()});
    }

}  // namespace localisation
}  // namespace utility

#endif
