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

#include <Eigen/Core>
#include <Eigen/Geometry>
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

    // Transforms the field state (x,y,theta) to the correct transform Hfw : World -> Field
    inline Eigen::Affine3d fieldStateToTransform3D(const Eigen::Vector3d& state) {
        Eigen::Affine3d Hfw;
        Hfw.translation() = Eigen::Vector3d(state.x(), state.y(), 0.0);
        Hfw.linear()      = Eigen::AngleAxisd(state.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        return Hfw;
    }

    inline Eigen::Affine2d projectTo2D(const Eigen::Affine3d& m,
                                       const Eigen::Vector3d& yawAxis,
                                       const Eigen::Vector3d& forwardAxis) {
        Eigen::Affine2d result;

        // Translation
        Eigen::Vector3d orthoForwardAxis = yawAxis.cross(forwardAxis.cross(yawAxis)).normalized();
        Eigen::Vector3d r                = m.translation();
        Eigen::Affine3d newSpaceToWorld;
        newSpaceToWorld.linear().block<3, 1>(0, 0) = orthoForwardAxis;
        newSpaceToWorld.linear().block<3, 1>(0, 1) = yawAxis.cross(orthoForwardAxis);
        newSpaceToWorld.linear().block<3, 1>(0, 2) = yawAxis;
        Eigen::Affine3d worldToNewSpace            = newSpaceToWorld.inverse();
        Eigen::Vector3d rNewSpace                  = worldToNewSpace * r;
        result.translation()                       = rNewSpace.head<2>();

        // Rotation
        Eigen::Affine3d rot(m);
        rot.translation()    = Eigen::Vector3d::Zero();
        Eigen::Vector3d x    = rot.linear().block<3, 1>(0, 0);
        Eigen::Vector3d xNew = worldToNewSpace * x;
        float theta_x_from_f = std::atan2(xNew.y(), xNew.x());  // sin/cos
        result.linear()      = Eigen::Rotation2Dd(theta_x_from_f).toRotationMatrix();

        return result;
    }

    // Transforms the transform
    inline Eigen::Affine2d transform3DToFieldState(const Eigen::Affine3d& m) {
        return projectTo2D(m, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX());
    }

}  // namespace localisation
}  // namespace utility

#endif
