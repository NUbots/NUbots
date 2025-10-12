/*
* MIT License
*
* Copyright (c) 2025 NUbots
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

#include "funcmin.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCore>

namespace utility::slam::funcmin {

    int trsEig(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, double D, Eigen::VectorXd& p) {
        assert(g.cols() == 1);
        assert(H.rows() == H.cols());
        assert(H.rows() == g.rows());

        p.resize(g.rows(), 1);

        typedef Eigen::VectorXd Vector;
        typedef Eigen::MatrixXd Matrix;

        Eigen::SelfAdjointEigenSolver<Matrix> eigenH(H);
        const Vector& v = eigenH.eigenvalues();
        const Matrix& Q = eigenH.eigenvectors();

        return trsEig(Q, v, g, D, p);
    }

    int trsEig(const Eigen::MatrixXd& Q, const Eigen::VectorXd& v, const Eigen::VectorXd& g, double D, Eigen::VectorXd& p) {
        assert(g.cols() == 1);
        assert(v.cols() == 1);
        assert(Q.rows() == Q.cols());
        assert(Q.rows() == g.rows());
        assert(v.rows() == g.rows());

        p.resize(g.rows(), 1);

        typedef double Scalar;
        typedef Eigen::VectorXd Vector;
        typedef Eigen::MatrixXd Matrix;
        typedef Vector::Index Index;

        const Scalar eps     = std::numeric_limits<Scalar>::epsilon();
        const Scalar sqrteps = std::sqrt(eps);
        const int maxIterations = 20;

        Scalar l1 = v.minCoeff();  // Leftmost eigenvalue
        Vector a  = Q.transpose() * g;

        Scalar lam;
        if (l1 < 0)
            lam = 1.01 * std::fabs(l1);
        else
            lam = 0;

        Vector vlam = v.array() + lam;
        p           = -Q * a.cwiseQuotient(vlam);

        if (l1 < 0 || p.norm() > D || std::fabs(lam * (p.norm() - D)) > sqrteps) {
            bool isHardCase = std::fabs(a(0)) < eps && l1 < 0;
            if (isHardCase) {
                std::vector<Index> idxValid;
                for (Index i = 0; i < v.size(); ++i)
                    if (std::fabs(v(i) - l1) > sqrteps)
                        idxValid.push_back(i);

                Vector scaledValid(idxValid.size());
                Matrix QValid(v.size(), idxValid.size());
                for (std::size_t i = 0; i < idxValid.size(); ++i) {
                    Index idx       = idxValid[i];
                    scaledValid(i)  = a(idx) / (v(idx) - l1);
                    QValid.col(i)   = Q.col(idx);
                }
                Scalar t = std::sqrt(D * D - scaledValid.squaredNorm());
                Vector pvec(v.size());
                if (idxValid.size() > 0)
                    pvec = QValid * scaledValid;
                else
                    pvec.setZero();

                p = t * Q.col(0) - pvec;

                // Choose sign of t to give a reduction in cost
                Vector q = Q.transpose() * p;
                if (p.dot(g) + 0.5 * q.transpose() * v.asDiagonal() * q > 0.0)
                    p = -t * Q.col(0) - pvec;
            }
            else {
                int k;
                for (k = 0; k < maxIterations; ++k) {
                    Vector pp     = -a.cwiseQuotient(vlam);
                    Vector dp     = a.cwiseQuotient(vlam.cwiseAbs2());
                    Scalar ppnorm = pp.norm();
                    Scalar ff     = 1 / D - 1 / ppnorm;
                    Scalar gg     = dp.dot(pp) / (ppnorm * ppnorm * ppnorm);

                    // Ensure lam > 0 and lam > l1
                    lam = std::max(std::max(0.0, -l1) + sqrteps * std::max(0.0, -l1), lam - ff / gg);

                    vlam = v.array() + lam;

                    if (std::fabs(ff) < sqrteps)
                        break;
                }

                p = -Q * a.cwiseQuotient(vlam);
                if (k >= maxIterations)
                    return 1;
            }
        }

        return 0;
    }

    int trsSqrt(const Eigen::MatrixXd& Xi, const Eigen::VectorXd& g, double D, Eigen::VectorXd& p) {
        assert(g.cols() == 1);
        assert(Xi.rows() == Xi.cols());
        assert(Xi.rows() == g.rows());
        assert(Xi.isUpperTriangular());

        // Solve Xi^T*gtilde = g for gtilde
        Eigen::VectorXd gtilde = Xi.triangularView<Eigen::Upper>().transpose().solve(g);

        // Step length
        double alpha = std::min(1.0, D / gtilde.norm());

        // Solve Xi*p = -alpha*gtilde for p
        p = Xi.triangularView<Eigen::Upper>().solve(-alpha * gtilde);

        return 0;
    }

    int trsSqrtSparse(const Eigen::SparseMatrix<double>& Xi,
                    const Eigen::PermutationMatrix<Eigen::Dynamic>& Pi,
                    const Eigen::VectorXd& g,
                    double D,
                    Eigen::VectorXd& p) {
        assert(g.cols() == 1);
        assert(Xi.rows() == Xi.cols());
        assert(Xi.rows() == g.rows());
        assert(Xi.toDense().isUpperTriangular());
        assert(Pi.size() == Xi.cols());

        // Solve Xi^T*gtilde = Pi^T*g for gtilde
        Eigen::VectorXd gtilde = Xi.triangularView<Eigen::Upper>().transpose().solve(Pi.transpose() * g);

        // Step length
        double alpha = std::min(1.0, D / gtilde.norm());

        // Solve Xi*Pi^T*p = -alpha*gtilde for p
        // Pi^T*p = Xi^{-1}*(-alpha*gtilde)
        // p = Pi*Xi^{-1}*(-alpha*gtilde)
        p = Pi * Xi.triangularView<Eigen::Upper>().solve(-alpha * gtilde);

        return 0;
    }

    int trsSqrtInv(const Eigen::MatrixXd& S, const Eigen::VectorXd& g, double D, Eigen::VectorXd& p) {
        assert(g.cols() == 1);
        assert(S.rows() == S.cols());
        assert(S.rows() == g.rows());

        Eigen::VectorXd gtilde = S * g;

        // Step length
        double alpha = std::min(1.0, D / gtilde.norm());

        p = -alpha * S.transpose() * gtilde;

        return 0;
    }

}  // namespace utility::slam::funcmin
