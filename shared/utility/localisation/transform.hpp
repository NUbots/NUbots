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
#ifndef UTILITY_LOCALISATION_TRANSFORM_HPP
#define UTILITY_LOCALISATION_TRANSFORM_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace utility::localisation {

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
        const Eigen::Vector3d orthoForwardAxis = yawAxis.cross(forwardAxis.cross(yawAxis)).normalized();
        const Eigen::Vector3d r                = m.translation();
        Eigen::Affine3d newSpaceToWorld(Eigen::Affine3d::Identity());
        newSpaceToWorld.linear().leftCols<1>()    = orthoForwardAxis;
        newSpaceToWorld.linear().middleCols<1>(1) = yawAxis.cross(orthoForwardAxis);
        newSpaceToWorld.linear().rightCols<1>()   = yawAxis;
        const Eigen::Affine3d worldToNewSpace(newSpaceToWorld.inverse());
        Eigen::Vector3d rNewSpace = worldToNewSpace * r;
        result.translation()      = rNewSpace.head<2>();

        // Rotation
        Eigen::Affine3d rot(m);
        rot.translation() = Eigen::Vector3d::Zero();
        const Eigen::Vector3d x(rot.linear().leftCols<1>());
        const Eigen::Vector3d xNew(worldToNewSpace * x);
        const float theta_x_from_f = std::atan2(xNew.y(), xNew.x());  // sin/cos
        result.linear()            = Eigen::Rotation2Dd(theta_x_from_f).toRotationMatrix();

        return result;
    }

    // Transforms the transform
    inline Eigen::Affine2d transform3DToFieldState(const Eigen::Affine3d& m) {
        return projectTo2D(m, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX());
    }

}  // namespace utility::localisation

#endif
