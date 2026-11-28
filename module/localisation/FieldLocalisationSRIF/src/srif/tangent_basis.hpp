/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#ifndef MODULE_LOCALISATION_SRIF_TANGENT_BASIS_HPP
#define MODULE_LOCALISATION_SRIF_TANGENT_BASIS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace module::localisation::srif {

    /**
     * @brief Orthonormal basis for the tangent plane of the unit sphere at u.
     *
     * Unit-ray residuals have two degrees of freedom, so they are evaluated in this plane rather than the
     * rank-deficient 3D chordal space.
     *
     * @param u Unit vector
     * @return 3x2 matrix whose columns are orthonormal and perpendicular to u
     */
    inline Eigen::Matrix<double, 3, 2> tangent_basis(const Eigen::Vector3d& u) {
        Eigen::Vector3d t1 = u.cross(Eigen::Vector3d::UnitZ());
        if (t1.squaredNorm() < 1e-8) {
            t1 = u.cross(Eigen::Vector3d::UnitX());
        }
        t1.normalize();
        Eigen::Matrix<double, 3, 2> T;
        T.col(0) = t1;
        T.col(1) = u.cross(t1).normalized();
        return T;
    }

}  // namespace module::localisation::srif

#endif  // MODULE_LOCALISATION_SRIF_TANGENT_BASIS_HPP
