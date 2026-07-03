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

#include <Eigen/Eigenvalues>
#include <algorithm>

#include "FieldLocalisationNLopt.hpp"

namespace module::localisation {

    Eigen::Matrix3d FieldLocalisationNLopt::finite_difference_hessian(
        const Eigen::Vector3d& x,
        const std::vector<Eigen::Vector3d>& field_lines,
        const std::shared_ptr<const FieldIntersections>& field_intersections,
        const std::shared_ptr<const Goals>& goals,
        const Eigen::Vector3d& h) {

        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        const double f0    = evaluate_cost(x, field_lines, field_intersections, goals);

        // Diagonal terms: central second difference.
        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3d xp = x;
            Eigen::Vector3d xm = x;
            xp(i) += h(i);
            xm(i) -= h(i);
            const double fp = evaluate_cost(xp, field_lines, field_intersections, goals);
            const double fm = evaluate_cost(xm, field_lines, field_intersections, goals);
            H(i, i)         = (fp - 2.0 * f0 + fm) / (h(i) * h(i));
        }

        // Off-diagonal terms: central mixed partial (4-point stencil), symmetrised by construction.
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                Eigen::Vector3d xpp = x;
                Eigen::Vector3d xpm = x;
                Eigen::Vector3d xmp = x;
                Eigen::Vector3d xmm = x;
                xpp(i) += h(i);
                xpp(j) += h(j);
                xpm(i) += h(i);
                xpm(j) -= h(j);
                xmp(i) -= h(i);
                xmp(j) += h(j);
                xmm(i) -= h(i);
                xmm(j) -= h(j);

                const double fpp = evaluate_cost(xpp, field_lines, field_intersections, goals);
                const double fpm = evaluate_cost(xpm, field_lines, field_intersections, goals);
                const double fmp = evaluate_cost(xmp, field_lines, field_intersections, goals);
                const double fmm = evaluate_cost(xmm, field_lines, field_intersections, goals);

                const double value = (fpp - fpm - fmp + fmm) / (4.0 * h(i) * h(j));
                H(i, j)             = value;
                H(j, i)             = value;
            }
        }

        return H;
    }

    Eigen::Matrix3d FieldLocalisationNLopt::covariance_from_hessian(const Eigen::Matrix3d& H,
                                                                     double scale,
                                                                     double r_min,
                                                                     double r_max) {
        // Non-positive or near-singular curvature is treated as "this direction wasn't observed", so is
        // clamped to r_max ("don't trust this direction") rather than triggering a wholesale fallback.
        constexpr double min_trustworthy_eigenvalue = 1e-9;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(H);
        const Eigen::Vector3d& eigenvalues  = solver.eigenvalues();
        const Eigen::Matrix3d& eigenvectors = solver.eigenvectors();

        Eigen::Vector3d clamped_variances;
        for (int i = 0; i < 3; ++i) {
            const double lambda = eigenvalues(i);
            clamped_variances(i) =
                (lambda <= min_trustworthy_eigenvalue) ? r_max : std::clamp(scale / lambda, r_min, r_max);
        }

        return eigenvectors * clamped_variances.asDiagonal() * eigenvectors.transpose();
    }

}  // namespace module::localisation
