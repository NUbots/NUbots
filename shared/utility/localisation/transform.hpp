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
    inline Eigen::Isometry3f fieldStateToTransform3D(const Eigen::Vector3f& state) {
        Eigen::Isometry3f Hfw;
        Hfw.translation() = Eigen::Vector3f(state.x(), state.y(), 0.0f);
        Hfw.linear()      = Eigen::AngleAxisf(state.z(), Eigen::Vector3f::UnitZ()).toRotationMatrix();
        return Hfw;
    }

    inline Eigen::Isometry2f projectTo2D(const Eigen::Isometry3f& m,
                                         const Eigen::Vector3f& yawAxis,
                                         const Eigen::Vector3f& forwardAxis) {
        Eigen::Isometry2f result;

        // Translation
        const Eigen::Vector3f orthoForwardAxis = yawAxis.cross(forwardAxis.cross(yawAxis)).normalized();
        const Eigen::Vector3f r                = m.translation();
        Eigen::Isometry3f newSpaceToWorld(Eigen::Isometry3f::Identity());
        newSpaceToWorld.linear().leftCols<1>()    = orthoForwardAxis;
        newSpaceToWorld.linear().middleCols<1>(1) = yawAxis.cross(orthoForwardAxis);
        newSpaceToWorld.linear().rightCols<1>()   = yawAxis;
        const Eigen::Isometry3f worldToNewSpace(newSpaceToWorld.inverse());
        Eigen::Vector3f rNewSpace = worldToNewSpace * r;
        result.translation()      = rNewSpace.head<2>();

        // Rotation
        Eigen::Isometry3f rot(m);
        rot.translation() = Eigen::Vector3f::Zero();
        const Eigen::Vector3f x(rot.linear().leftCols<1>());
        const Eigen::Vector3f xNew(worldToNewSpace * x);
        const float theta_x_from_f = std::atan2(xNew.y(), xNew.x());  // sin/cos
        result.linear()            = Eigen::Rotation2Df(theta_x_from_f).toRotationMatrix();

        return result;
    }

    // Transforms the transform
    inline Eigen::Isometry2f transform3DToFieldState(const Eigen::Isometry3f& m) {
        return projectTo2D(m, Eigen::Vector3f::UnitZ(), Eigen::Vector3f::UnitX());
    }

}  // namespace utility::localisation

#endif
