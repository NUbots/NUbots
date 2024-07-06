/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef UTILITY_LOCALISATION_TRANSFORM_HPP
#define UTILITY_LOCALISATION_TRANSFORM_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace utility::localisation {

    // Transforms the field state (x,y,theta) to the correct transform Hfw : World -> Field
    inline Eigen::Isometry3d fieldStateToTransform3D(const Eigen::Vector3d& state) {
        Eigen::Isometry3d Hfw;
        Hfw.translation() = Eigen::Vector3d(state.x(), state.y(), 0.0);
        Hfw.linear()      = Eigen::AngleAxisd(state.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        return Hfw;
    }

    inline Eigen::Isometry2d projectTo2D(const Eigen::Isometry3d& m,
                                         const Eigen::Vector3d& yawAxis,
                                         const Eigen::Vector3d& forwardAxis) {
        Eigen::Isometry2d result;

        // Translation
        const Eigen::Vector3d orthoForwardAxis = yawAxis.cross(forwardAxis.cross(yawAxis)).normalized();
        const Eigen::Vector3d r                = m.translation();
        Eigen::Isometry3d newSpaceToWorld(Eigen::Isometry3d::Identity());
        newSpaceToWorld.linear().leftCols<1>()    = orthoForwardAxis;
        newSpaceToWorld.linear().middleCols<1>(1) = yawAxis.cross(orthoForwardAxis);
        newSpaceToWorld.linear().rightCols<1>()   = yawAxis;
        const Eigen::Isometry3d worldToNewSpace(newSpaceToWorld.inverse());
        Eigen::Vector3d rNewSpace = worldToNewSpace * r;
        result.translation()      = rNewSpace.head<2>();

        // Rotation
        Eigen::Isometry3d rot(m);
        rot.translation() = Eigen::Vector3d::Zero();
        const Eigen::Vector3d x(rot.linear().leftCols<1>());
        const Eigen::Vector3d xNew(worldToNewSpace * x);
        const float theta_x_from_f = std::atan2(xNew.y(), xNew.x());  // sin/cos
        result.linear()            = Eigen::Rotation2Dd(theta_x_from_f).toRotationMatrix();

        return result;
    }

    // Transforms the transform
    inline Eigen::Isometry2d transform3DToFieldState(const Eigen::Isometry3d& m) {
        return projectTo2D(m, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX());
    }

}  // namespace utility::localisation

#endif
