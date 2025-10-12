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

#ifndef UTILITY_SLAM_FUNCMIN_HPP
#define UTILITY_SLAM_FUNCMIN_HPP

#include <cassert>
#include <cmath>
#include <limits>
#include <print>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>

namespace utility::slam::funcmin {

    /**
 * @brief Solve trust-region subproblem
 *
 * This function solves the following trust-region subproblem:
 * @f[
 * \begin{aligned}
 * \text{minimize} \quad & \frac12 \mathbf{p}^\mathsf{T} \mathbf{H} \mathbf{p} + \mathbf{g}^\mathsf{T} \mathbf{p} \\
 * \text{subject to} \quad & \Vert \mathbf{p} \Vert \leq D
 * \end{aligned}
 * @f]
 *
 * @param H Hessian matrix
 * @param g Gradient vector
 * @param D Trust region radius
 * @param[out] p Solution vector
 * @return 0 on success, non-zero on failure
 *
 * @see Moré, J.J. and D.C. Sorensen, "Computing a Trust Region Step",
 *      SIAM Journal on Scientific and Statistical Computing, Vol. 3, pp. 553-572, 1983.
 */
int trsEig(const Eigen::MatrixXd & H, const Eigen::VectorXd & g, double D, Eigen::VectorXd & p);

/**
 * @brief Solve trust-region subproblem with pre-computed eigendecomposition
 *
 * This function solves the following trust-region subproblem given the eigendecomposition H = Q*diag(v)*Q':
 * @f[
 * \begin{aligned}
 * \text{minimize} \quad & \frac12 \mathbf{p}^\mathsf{T} \mathbf{H} \mathbf{p} + \mathbf{g}^\mathsf{T} \mathbf{p} \\
 * \text{subject to} \quad & \Vert \mathbf{p} \Vert \leq D
 * \end{aligned}
 * @f]
 *
 * @param Q Eigenvectors of H
 * @param v Eigenvalues of H
 * @param g Gradient vector
 * @param D Trust region radius
 * @param[out] p Solution vector
 * @return 0 on success, non-zero on failure
 */
int trsEig(const Eigen::MatrixXd & Q, const Eigen::VectorXd & v, const Eigen::VectorXd & g, double D, Eigen::VectorXd & p);


/**
 * @brief Solve trust-region subproblem with square-root Hessian
 *
 * This function solves the following trust-region subproblem:
 * @f[
 * \begin{aligned}
 * \text{minimize} \quad & \frac12 \mathbf{p}^\mathsf{T} \boldsymbol\Xi^\mathsf{T}\boldsymbol\Xi \mathbf{p} + \mathbf{g}^\mathsf{T} \mathbf{p} \\
 * \text{subject to} \quad & \Vert \boldsymbol\Xi\mathbf{p} \Vert \leq D
 * \end{aligned}
 * @f]
 *
 * @param Xi Square-root Hessian matrix (upper triangular)
 * @param g Gradient vector
 * @param D Trust region radius
 * @param[out] p Solution vector
 * @return 0 on success, non-zero on failure
 */
int trsSqrt(const Eigen::MatrixXd & Xi, const Eigen::VectorXd & g, double D, Eigen::VectorXd & p);

/**
 * @brief Solve trust-region subproblem with sparse square-root Hessian
 *
 * This function solves the following trust-region subproblem with sparse matrix representation:
 * @f[
 * \begin{aligned}
 * \text{minimize} \quad & \frac12 \mathbf{p}^\mathsf{T} \boldsymbol\Pi \boldsymbol\Xi^\mathsf{T}\boldsymbol\Xi \boldsymbol\Pi^\mathsf{T} \mathbf{p} + \mathbf{g}^\mathsf{T} \mathbf{p} \\
 * \text{subject to} \quad & \Vert \boldsymbol\Xi \boldsymbol\Pi^\mathsf{T} \mathbf{p} \Vert \leq D
 * \end{aligned}
 * @f]
 *
 * where the Hessian approximation is H = Pi*Xi^T*Xi*Pi^T with Xi being a sparse upper triangular matrix
 * and Pi being a permutation matrix used for fill-in reduction during sparse factorization.
 *
 * @param Xi Sparse square-root Hessian matrix (upper triangular)
 * @param Pi Permutation matrix for column pivoting to reduce fill-in
 * @param g Gradient vector
 * @param D Trust region radius
 * @param[out] p Solution vector
 * @return 0 on success, non-zero on failure
 *
 * @note This variant is particularly efficient for large-scale optimization problems where
 *       the Hessian has a sparse structure that can be exploited to reduce memory usage
 *       and computational complexity.
 */
int trsSqrtSparse(const Eigen::SparseMatrix<double> & Xi, const Eigen::PermutationMatrix<Eigen::Dynamic> & Pi, const Eigen::VectorXd & g, double D, Eigen::VectorXd & p);

/**
 * @brief Solve trust-region subproblem with square-root inverse Hessian
 *
 * This function solves the following trust-region subproblem:
 * @f[
 * \begin{aligned}
 * \text{minimize} \quad & \frac12 \mathbf{p}^\mathsf{T} (\mathbf{S}^\mathsf{T}\mathbf{S})^{-1} \mathbf{p} + \mathbf{g}^\mathsf{T} \mathbf{p} \\
 * \text{subject to} \quad & \Vert \mathbf{S}^\mathsf{-T} \mathbf{p} \Vert \leq D
 * \end{aligned}
 * @f]
 *
 * @param S Square-root inverse Hessian matrix (upper triangular)
 * @param g Gradient vector
 * @param D Trust region radius
 * @param[out] p Solution vector
 * @return 0 on success, non-zero on failure
 */
int trsSqrtInv(const Eigen::MatrixXd & S, const Eigen::VectorXd & g, double D, Eigen::VectorXd & p);

/**
 * @brief Minimize f(x) using trust-region Newton method with eigendecomposition
 *
 * This function minimizes the cost function f(x) using a trust-region Newton method.
 * It uses eigendecomposition of the Hessian matrix for solving the trust-region subproblem.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient and Hessian
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param[out] Q Eigenvectors of the Hessian at the optimal solution
 * @param[out] v Eigenvalues of the Hessian at the optimal solution
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int NewtonTrustEig(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & Q, Eigen::VectorXd & v, int verbosity = 0)
{
    typedef double Scalar;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;

    assert(x.cols() == 1);
    g.resize(x.size());
    Q.resize(x.size(), x.size());
    v.resize(x.size());

    Matrix H(x.size(), x.size());

    // Storage for trial point, gradient and Hessian
    Vector xn(x.size());
    Vector gn(x.size());
    Matrix Hn(x.size(), x.size());

    // Evaluate initial cost, gradient and Hessian
    Scalar f = costFunc(x, g, H);
    if (!std::isfinite(f) || !g.allFinite() || !H.allFinite())      // if any nan, -inf or +inf
    {
        if (verbosity > 1)
            std::println("ERROR: Initial point is not in domain of cost function");
        return -1;
    }

    // Eigendecomposition of initial Hessian
    Eigen::SelfAdjointEigenSolver<Matrix> eigenH(H);
    v = eigenH.eigenvalues();
    Q = eigenH.eigenvectors();

    Scalar Delta = 1e0;     // Initial trust-region radius

    const int maxIterations = 5000;
    for (int i = 0; i < maxIterations; ++i)
    {
        // Solve trust-region subproblem
        Vector p;
        trsEig(Q, v, g, Delta, p);   // minimise 0.5*p.'*H*p + g.'*p subject to ||p|| <= Delta

        Scalar pg = p.dot(g);
        Scalar LambdaSq = -pg;  // The Newton decrement squared is g.'*inv(H)*g = p.'*H*p
        if (verbosity == 3)
            std::println("Iter = {:5}, Cost = {:10.2e}, Newton decr^2 = {:10.2e}, Delta = {:10.2e}", i, f, LambdaSq, Delta);;
        if (verbosity == 1)
            std::print(".");

        // const Scalar LambdaSqThreshold = std::sqrt(std::numeric_limits<Scalar>::epsilon());     // Loose convergence tolerance
        const Scalar LambdaSqThreshold = 2*std::numeric_limits<Scalar>::epsilon();              // Tight convergence tolerance
        if (std::fabs(LambdaSq) < LambdaSqThreshold)
        {
            if (verbosity >= 2)
                std::println("CONVERGED: Newton decrement below threshold in {} iterations", i);
            return 0;
        }

        // Evaluate cost, gradient and Hessian for trial step
        xn = x + p;
        Scalar fn = costFunc(xn, gn, Hn);
        bool outOfDomain = !std::isfinite(fn) || !gn.allFinite() || !Hn.allFinite();    // if any nan, -inf or +inf
        Scalar rho;
        if (outOfDomain)
        {
            rho = -1;                           // Force trust region reduction and reject step
        }
        else
        {
            Scalar dm = -pg - 0.5*p.dot(H*p);   // Predicted reduction f - fp, where fp = f + p'*g + 0.5*p'*H*p
            rho = (f - fn)/dm;                  // Actual reduction divided by predicted reduction
        }

        if (rho < 0.1)
        {
            Delta = 0.25*p.norm();              // Decrease trust region radius
        }
        else
        {
            if (rho > 0.75 && p.norm() > 0.8*Delta)
            {
                Delta = 2.0*Delta;              // Increase trust region radius
            }
        }

        if (rho >= 0.001)
        {
            // Accept the step
            x = xn;
            f = fn;
            g = gn;
            H = Hn;

            // Eigendecomposition of accepted Hessian
            Eigen::SelfAdjointEigenSolver<Matrix> eigenH_(H);
            v = eigenH_.eigenvalues();
            Q = eigenH_.eigenvectors();
        }
    }
    if (verbosity > 1)
        std::println("WARNING: maximum number of iterations reached");
    return 1;
}

/**
 * @brief Minimize f(x) using trust-region Newton method
 *
 * This function minimizes the cost function f(x) using a trust-region Newton method.
 * It computes the Hessian matrix as part of the optimization process.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient and Hessian
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param[out] H Hessian matrix at the optimal solution
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int NewtonTrust(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H, int verbosity = 0)
{
    Eigen::MatrixXd Q(x.size(), x.size());
    Eigen::VectorXd v(x.size());
    int retval = NewtonTrustEig(costFunc, x, g, Q, v, verbosity);
    H = Q*v.asDiagonal()*Q.transpose();
    return retval;
}

/**
 * @brief Minimize f(x) using trust-region Newton method
 *
 * This function minimizes the cost function f(x) using a trust-region Newton method.
 * It does not return the Hessian matrix.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient and Hessian
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int NewtonTrust(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, int verbosity = 0)
{
    Eigen::MatrixXd H(x.size(), x.size());
    return NewtonTrust(costFunc, x, g, H, verbosity);
}

/**
 * @brief Minimize f(x) using trust-region Newton method
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient and Hessian
 * @param[in,out] x  Initial guess on input, optimal solution on output
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int NewtonTrust(Func costFunc, Eigen::VectorXd & x, int verbosity = 0)
{
    Eigen::VectorXd g(x.size());
    return NewtonTrust(costFunc, x, g, verbosity);
}

/**
 * @brief Sign function
 *
 * Returns the sign of a value.
 *
 * @tparam T The type of the input value
 * @param val The input value
 * @return int
 * @retval -1 if val < 0
 * @retval 0 if val = 0
 * @retval 1 if val > 0
 */
template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

/**
 * @brief Minimize f(x) using trust-region quasi-Newton SR1 method
 *
 * This function minimizes the cost function f(x) using a trust-region quasi-Newton SR1
 * (Symmetric Rank-1) method. It updates the Hessian approximation using the SR1 update formula.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param[out] Q Eigenvectors of the Hessian at the optimal solution
 * @param[out] v Eigenvalues of the Hessian at the optimal solution
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return int 0 on success, non-zero on failure
 */
template <typename Func>
int SR1TrustEig(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & Q, Eigen::VectorXd & v, int verbosity = 0)
{
    typedef double Scalar;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;

    assert(x.cols() == 1);
    g.resize(x.size());
    Q.resize(x.size(), x.size());
    v.resize(x.size());

    Matrix H(x.size(),x.size());

    // Storage for trial point and gradient
    Vector xn(x.size());
    Vector gn(x.size());

    // Evaluate initial cost and gradient
    Scalar f = costFunc(x, g);
    if (!std::isfinite(f) || !g.allFinite())        // if any nan, -inf or +inf
    {
        if (verbosity > 1)
            std::println("ERROR: Initial point is not in domain of cost function");
        return -1;
    }

    // Initial Hessian
    H = Q*v.asDiagonal()*Q.transpose();

    Scalar Delta = 1e0;     // Initial trust-region radius

    const int maxIterations = 5000;
    for (int i = 0; i < maxIterations; ++i)
    {
        // Solve trust-region subproblem
        Vector p;
        trsEig(Q, v, g, Delta, p);   // minimise 0.5*p.'*H*p + g.'*p subject to ||p|| <= Delta

        Scalar pg = p.dot(g);
        Scalar LambdaSq = -pg;  // The Newton decrement squared is g.'*inv(H)*g = p.'*H*p
        if (verbosity == 3)
            std::println("Iter = {:5}, Cost = {:10.2e}, Newton decr^2 = {:10.2e}, Delta = {:10.2e}", i, f, LambdaSq, Delta);;
        if (verbosity == 1)
            std::print(".");

        const Scalar LambdaSqThreshold = std::sqrt(std::numeric_limits<Scalar>::epsilon());     // Loose convergence tolerance
        // const Scalar LambdaSqThreshold = 2*std::numeric_limits<Scalar>::epsilon();              // Tight convergence tolerance
        if (std::fabs(LambdaSq) < LambdaSqThreshold && v(0) > 0.0)
        {
            if (verbosity >= 2)
                std::println("CONVERGED: Newton decrement below threshold in {} iterations", i);
            return 0;
        }

        // Evaluate cost and gradient for trial step
        xn = x + p;
        Scalar fn = costFunc(xn, gn);
        bool outOfDomain = !std::isfinite(fn) || !gn.allFinite();   // if any nan, -inf or +inf
        Vector y = gn - g;

        Scalar rho;
        if (outOfDomain)
        {
            rho = -1;                           // Force trust region reduction and reject step
        }
        else
        {
            Scalar dm = -pg - 0.5*p.dot(H*p);   // Predicted reduction f - fp, where fp = f + p'*g + 0.5*p'*H*p
            rho = (f - fn)/dm;                  // Actual reduction divided by predicted reduction
        }

        if (rho < 0.1)
        {
            Delta = 0.25*p.norm();              // Decrease trust region radius
        }
        else
        {
            if (rho > 0.75 && p.norm() > 0.8*Delta)
            {
                Delta = 2.0*Delta;              // Increase trust region radius
            }
        }

        if (rho >= 0.001)
        {
            // Accept the step
            x = xn;
            f = fn;
            g = gn;
        }

        // Update Hessian approximation
        const Scalar sqrteps = std::sqrt(std::numeric_limits<Scalar>::epsilon());
        if (!outOfDomain)
        {
            Vector w = y - H*p;
            Scalar pw = p.dot(w);
            if (std::fabs(pw) > sqrteps*p.norm()*w.norm())
            {
                Scalar s = sgn(pw);
                Vector u = w.array()/std::sqrt(s*pw);
                H = H + s*(u*u.transpose());

                // Eigendecomposition of updated Hessian
                Eigen::SelfAdjointEigenSolver<Matrix> eigenH(H);
                v = eigenH.eigenvalues();
                Q = eigenH.eigenvectors();
                // We should do a rank-one update of Q and v instead of a full eigendecomposition using, e.g.,
                // [1] Bunch, J.R., Nielsen, C.P. and Sorensen, D.C., 1978. Rank-one modification of the symmetric eigenproblem. Numerische Mathematik, 31(1), pp.31-48.
            }
        }
    }
    if (verbosity > 1)
        std::println("WARNING: maximum number of iterations reached");
    return 1;
}

/**
 * @brief Minimize f(x) using trust-region quasi-Newton SR1 method
 *
 * This function minimizes the cost function f(x) using a trust-region quasi-Newton SR1 method.
 * It computes and returns the Hessian matrix as part of the optimization process.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param[out] H Hessian matrix at the optimal solution
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int SR1Trust(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H, int verbosity = 0)
{
    Eigen::MatrixXd Q(x.size(), x.size());
    Eigen::VectorXd v(x.size());
    Q.setIdentity();
    v.fill(1.0);
    int retval = SR1TrustEig(costFunc, x, g, Q, v, verbosity);
    H = Q*v.asDiagonal()*Q.transpose();
    return retval;
}

/**
 * @brief Minimize f(x) using trust-region quasi-Newton SR1 method
 *
 * This function minimizes the cost function f(x) using a trust-region quasi-Newton SR1 method.
 * It does not return the Hessian matrix.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int SR1Trust(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, int verbosity = 0)
{
    Eigen::MatrixXd H(x.size(), x.size());
    return SR1Trust(costFunc, x, g, H, verbosity);
}

/**
 * @brief Minimize f(x) using trust-region quasi-Newton SR1 method
 *
 * This function minimizes the cost function f(x) using a trust-region quasi-Newton SR1 method.
 * It does not return the gradient or Hessian matrix.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int SR1Trust(Func costFunc, Eigen::VectorXd & x, int verbosity = 0)
{
    Eigen::VectorXd g(x.size());
    return SR1Trust(costFunc, x, g, verbosity);
}

/**
 * @brief Minimize f(x) using trust-region quasi-Newton BFGS method with square-root Hessian
 *
 * This function minimizes the cost function f(x) using a trust-region quasi-Newton BFGS
 * (Broyden-Fletcher-Goldfarb-Shanno) method. It uses a square-root representation of the Hessian
 * matrix for improved numerical stability.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param[in,out] Xi Square-root of the Hessian matrix (upper triangular)
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int BFGSTrustSqrt(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & Xi, int verbosity = 0)
{
    typedef double Scalar;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;

    assert(x.cols() == 1);
    g.resize(x.size());
    assert(Xi.isUpperTriangular());

    // Storage for trial point and gradient
    Vector xn(x.size());
    Vector gn(x.size());

    // Evaluate initial cost and gradient
    Scalar f = costFunc(x, g);
    if (!std::isfinite(f) || !g.allFinite())        // if any nan, -inf or +inf
    {
        if (verbosity > 1)
            std::println("ERROR: Initial point is not in domain of cost function"); // initial means
        return -1;
    }

    Scalar Delta = 10e0;     // Initial trust-region radius

    const int maxIterations = 5000;
    for (int i = 0; i < maxIterations; ++i)
    {
        // Solve trust-region subproblem
        Vector p;
        trsSqrt(Xi, g, Delta, p);   // minimise 0.5*p.'*Xi.'*Xi*p + g.'*p subject to ||Xi*p|| <= Delta
        Vector z = Xi*p;

        Scalar pg = p.dot(g);
        Scalar LambdaSq = -pg;  // The Newton decrement squared is g.'*inv(H)*g = p.'*H*p = p.'*Xi.'*Xi*p
        if (verbosity == 3)
            std::println("Iter = {:5}, Cost = {:10.2e}, Newton decr^2 = {:10.2e}, Delta = {:10.2e}", i, f, LambdaSq, Delta);;
        if (verbosity == 1)
            std::print(".");

        // const Scalar LambdaSqThreshold = std::sqrt(std::numeric_limits<Scalar>::epsilon());     // Loose convergence tolerance
        const Scalar LambdaSqThreshold = 2*std::numeric_limits<Scalar>::epsilon();              // Tight convergence tolerance
        if (std::fabs(LambdaSq) < LambdaSqThreshold)
        {
            if (verbosity >= 2)
                std::println("CONVERGED: Newton decrement below threshold in {} iterations", i);
            return 0;
        }

        // Evaluate cost and gradient for trial step
        xn = x + p;
        Scalar fn = costFunc(xn, gn);
        bool outOfDomain = !std::isfinite(fn) || !gn.allFinite();   // if any nan, -inf or +inf
        Vector y = gn - g;

        Scalar rho;
        if (outOfDomain)
        {
            rho = -1;                           // Force trust region reduction and reject step
        }
        else
        {
            Scalar dm = -pg - 0.5*z.squaredNorm();   // Predicted reduction f - fp, where fp = f + p'*g + 0.5*p'*H*p
            rho = (f - fn)/dm;                  // Actual reduction divided by predicted reduction
        }

        if (rho < 0.1)
        {
            Delta = 0.25*z.norm();            // Decrease trust region radius
        }
        else
        {
            if (rho > 0.75 && z.norm() > 0.8*Delta)
            {
                Delta = 2.0*Delta;              // Increase trust region radius
            }
        }

        if (rho >= 0.001)
        {
            // Accept the step
            x = xn;
            f = fn;
            g = gn;
        }

        // Update Hessian approximation
        const Scalar sqrteps = std::sqrt(std::numeric_limits<Scalar>::epsilon());
        if (!outOfDomain)
        {
            Scalar py = p.dot(y);
            if (py > sqrteps*y.norm()*p.norm())
            {
                // Form
                // [ Xi*p,              Xi ]
                // [    0, y^T/sqrt(y^T*p) ]
                Matrix RR(Xi.rows() + 1, Xi.cols() + 1);
                RR << Xi*p, Xi,
                      0, y.transpose()/std::sqrt(py);
                // Q-less QR yields
                // [ R1, R2 ]
                // [  0, R3 ]
                Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(RR);    // In-place QR decomposition
                Xi = RR.bottomRightCorner(Xi.rows(), Xi.cols()).triangularView<Eigen::Upper>();
            }
        }
    }
    if (verbosity > 1)
        std::println("WARNING: maximum number of iterations reached");
    return 1;
}

/**
 * @brief Minimize f(x) using BFGS trust-region method
 *
 * This function minimizes the cost function f(x) using a BFGS trust-region method.
 * It computes and returns the Hessian matrix as part of the optimization process.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param[out] H Hessian matrix at the optimal solution
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int BFGSTrust(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H, int verbosity = 0)
{
    Eigen::MatrixXd Xi(x.size(), x.size());
    Xi.setIdentity();
    int retval = BFGSTrustSqrt(costFunc, x, g, Xi, verbosity);
    H = Xi.transpose()*Xi;
    return retval;
}

/**
 * @brief Minimize f(x) using BFGS trust-region method
 *
 * This function minimizes the cost function f(x) using a BFGS trust-region method.
 * It does not return the Hessian matrix.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int BFGSTrust(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, int verbosity = 0)
{
    Eigen::MatrixXd H(x.size(), x.size());
    return BFGSTrust(costFunc, x, g, H, verbosity);
}

/**
 * @brief Minimize f(x) using BFGS trust-region method
 *
 * This function minimizes the cost function f(x) using a BFGS trust-region method.
 * It does not return the gradient or Hessian matrix.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int BFGSTrust(Func costFunc, Eigen::VectorXd & x, int verbosity = 0)
{
    Eigen::VectorXd g(x.size());
    return BFGSTrust(costFunc, x, g, verbosity);
}

/**
 * @brief Minimize f(x) using trust-region quasi-Newton BFGS method with sparse square-root Hessian
 *
 * This function minimizes the cost function f(x) using a trust-region quasi-Newton BFGS
 * (Broyden-Fletcher-Goldfarb-Shanno) method. It uses a sparse square-root representation of the Hessian
 * matrix for improved numerical stability and memory efficiency when dealing with large sparse problems.
 * The Hessian approximation is maintained as H = Pi*Xi^T*Xi*Pi^T where Xi is sparse upper triangular
 * and Pi is a permutation matrix for fill-in reduction.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param[in,out] Xi Sparse square-root of the Hessian matrix (upper triangular)
 * @param[in,out] Pi Permutation matrix for column pivoting to reduce fill-in
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 *
 * @note This variant is particularly efficient for large-scale optimization problems where
 *       the Hessian has a sparse structure that can be exploited.
 */
template <typename Func>
int BFGSTrustSqrtSparse(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::SparseMatrix<double> & Xi, Eigen::PermutationMatrix<Eigen::Dynamic> & Pi, int verbosity = 0)
{
    typedef double Scalar;
    typedef Eigen::VectorXd Vector;
    // typedef Eigen::MatrixXd Matrix;

    assert(x.cols() == 1);
    g.resize(x.size());
    assert(Xi.toDense().isUpperTriangular());

    // Storage for trial point and gradient
    Vector xn(x.size());
    Vector gn(x.size());

    // Evaluate initial cost and gradient
    Scalar f = costFunc(x, g);
    if (!std::isfinite(f) || !g.allFinite())        // if any nan, -inf or +inf
    {
        if (verbosity > 1)
            std::println("ERROR: Initial point is not in domain of cost function");
        return -1;
    }

    Scalar Delta = 10e0;     // Initial trust-region radius

    const int maxIterations = 5000;
    for (int i = 0; i < maxIterations; ++i)
    {
        // Solve trust-region subproblem
        Vector p;
        trsSqrtSparse(Xi, Pi, g, Delta, p); // minimise 0.5*p.'*Pi*Xi.'*Xi*Pi.'*p + g.'*p subject to ||Xi*Pi.'*p|| <= Delta
        Vector z = Xi*Pi.transpose()*p;     // Xi*Pi.'*p

        Scalar pg = p.dot(g);
        Scalar LambdaSq = -pg;  // The Newton decrement squared is g.'*inv(H)*g = p.'*H*p = p.'*Xi.'*Xi*p
        if (verbosity == 3)
            std::println("Iter = {:5}, Cost = {:10.2e}, Newton decr^2 = {:10.2e}, Delta = {:10.2e}", i, f, LambdaSq, Delta);;
        if (verbosity == 1)
            std::print(".");

        // const Scalar LambdaSqThreshold = std::sqrt(std::numeric_limits<Scalar>::epsilon());     // Loose convergence tolerance
        const Scalar LambdaSqThreshold = 2*std::numeric_limits<Scalar>::epsilon();              // Tight convergence tolerance
        if (std::fabs(LambdaSq) < LambdaSqThreshold)
        {
            if (verbosity >= 2)
                std::println("CONVERGED: Newton decrement below threshold in {} iterations", i);
            return 0;
        }

        // Evaluate cost and gradient for trial step
        xn = x + p;
        Scalar fn = costFunc(xn, gn);
        bool outOfDomain = !std::isfinite(fn) || !gn.allFinite();   // if any nan, -inf or +inf
        Vector y = gn - g;

        Scalar rho;
        if (outOfDomain)
        {
            rho = -1;                           // Force trust region reduction and reject step
        }
        else
        {
            Scalar dm = -pg - 0.5*z.squaredNorm();   // Predicted reduction f - fp, where fp = f + p'*g + 0.5*p'*H*p
            rho = (f - fn)/dm;                  // Actual reduction divided by predicted reduction
        }

        if (rho < 0.1)
        {
            Delta = 0.25*z.norm();            // Decrease trust region radius
        }
        else
        {
            if (rho > 0.75 && z.norm() > 0.8*Delta)
            {
                Delta = 2.0*Delta;              // Increase trust region radius
            }
        }

        if (rho >= 0.001)
        {
            // Accept the step
            x = xn;
            f = fn;
            g = gn;
        }

        // Update Hessian approximation
        const Scalar sqrteps = std::sqrt(std::numeric_limits<Scalar>::epsilon());
        if (!outOfDomain)
        {
            Scalar py = p.dot(y);
            if (py > sqrteps*y.norm()*p.norm())
            {
                Eigen::SparseQR<Eigen::SparseMatrix<Scalar>, Eigen::COLAMDOrdering<int>> spqr_solver;

                // Compute column-pivoting QR decomposition: [Xi*Pi.'*p; 0]*Pi1 = Q1*RR1 = [Y1, Z1]*[R1; 0], where Pi1 reduces fill-in of R1
                Eigen::VectorX<Scalar> XiPiTp_0(Xi.rows() + 1);
                XiPiTp_0 << Xi*Pi.transpose()*p, 0;
                spqr_solver.compute(XiPiTp_0.sparseView());

                // [Xi*Pi.'; y.'/realsqrt(py)]
                Eigen::SparseMatrix<Scalar> XiPiT_yTsqrtpy(Xi.rows() + 1, Xi.cols());
                // Copy Xi*Pi.transpose() into top rows of XiPiT_yTsqrtpy
                Eigen::SparseMatrix<Scalar> XiPiT = Xi*Pi.transpose();
                for (int k = 0; k < XiPiT.outerSize(); ++k)
                    for (typename Eigen::SparseMatrix<Scalar>::InnerIterator it(XiPiT, k); it; ++it)
                        XiPiT_yTsqrtpy.insert(it.row(), it.col()) = it.value();
                // Copy y.transpose()/std::sqrt(py) into bottom row of XiPiT_yTsqrtpy
                Eigen::RowVectorX<Scalar> yTsqrtpy = y.transpose()/std::sqrt(py);
                for (int c = 0; c < Xi.cols(); ++c)
                    XiPiT_yTsqrtpy.insert(Xi.rows(), c) = yTsqrtpy(c);

                // Z1.'*[Xi*Pi.'; y.'/realsqrt(py)]
                Eigen::SparseMatrix<Scalar> Z1T_XiPiT_yTsqrtpy = (spqr_solver.matrixQ().transpose()*XiPiT_yTsqrtpy.toDense()).bottomRows(Xi.rows()).sparseView();

                // Compute column-pivoting QR decomposition: Z1.'*[Xi*Pi.'; y.'/realsqrt(py)]*Pi2 = Q2*R2, where Pi2 reduces fill-in of R2
                Z1T_XiPiT_yTsqrtpy.makeCompressed();
                spqr_solver.compute(Z1T_XiPiT_yTsqrtpy);

                Xi = spqr_solver.matrixR();
                Pi = spqr_solver.colsPermutation();
            }
        }
    }
    if (verbosity > 1)
        std::println("WARNING: maximum number of iterations reached");
    return 1;
}

/**
 * @brief Minimize f(x) using trust-region quasi-Newton BFGS method with sparse matrices
 *
 * This function minimizes the cost function f(x) using a trust-region quasi-Newton BFGS
 * (Broyden-Fletcher-Goldfarb-Shanno) method with sparse matrix representation for improved
 * memory efficiency in large-scale optimization problems. It computes and returns the
 * Hessian matrix as part of the optimization process.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param[out] H Hessian matrix at the optimal solution
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 *
 * @note This variant uses sparse matrix operations internally but returns a dense Hessian matrix.
 *       It is suitable for problems where the Hessian structure can benefit from sparse storage
 *       during optimization but a dense result is desired.
 */
template <typename Func>
int BFGSTrustSparse(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H, int verbosity = 0)
{
    Eigen::SparseMatrix<double> Xi(x.size(), x.size());
    Xi.setIdentity();
    Eigen::PermutationMatrix<Eigen::Dynamic> Pi(x.size());
    Pi.setIdentity();
    int retval = BFGSTrustSqrtSparse(costFunc, x, g, Xi, Pi, verbosity);
    H = Pi*(Xi.transpose()*Xi)*Pi.transpose();
    return retval;
}

/**
 * @brief Minimize f(x) using trust-region quasi-Newton BFGS method with sparse matrices
 *
 * This function minimizes the cost function f(x) using a trust-region quasi-Newton BFGS
 * (Broyden-Fletcher-Goldfarb-Shanno) method with sparse matrix representation for improved
 * memory efficiency in large-scale optimization problems. It does not return the Hessian matrix.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 *
 * @note This variant is optimized for large-scale problems where memory usage is a concern
 *       and the final Hessian matrix is not needed.
 */
template <typename Func>
int BFGSTrustSparse(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, int verbosity = 0)
{
    Eigen::MatrixXd H(x.size(), x.size());
    return BFGSTrustSparse(costFunc, x, g, H, verbosity);
}

/**
 * @brief Minimize f(x) using trust-region quasi-Newton BFGS method with sparse matrices
 *
 * This function minimizes the cost function f(x) using a trust-region quasi-Newton BFGS
 * (Broyden-Fletcher-Goldfarb-Shanno) method with sparse matrix representation for improved
 * memory efficiency in large-scale optimization problems. It does not return the gradient
 * or Hessian matrix.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 *
 * @note This is the most memory-efficient variant, suitable for large-scale optimization
 *       problems where only the optimal solution is needed.
 */
template <typename Func>
int BFGSTrustSparse(Func costFunc, Eigen::VectorXd & x, int verbosity = 0)
{
    Eigen::VectorXd g(x.size());
    return BFGSTrustSparse(costFunc, x, g, verbosity);
}

/**
 * @brief Minimize f(x) using trust-region quasi-Newton BFGS method with square-root inverse Hessian
 *
 * This function minimizes the cost function f(x) using a trust-region quasi-Newton BFGS
 * (Broyden-Fletcher-Goldfarb-Shanno) method. It uses a square-root representation of the
 * inverse Hessian matrix for improved numerical stability.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param[in,out] S Square-root of the inverse Hessian matrix (upper triangular)
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int BFGSTrustSqrtInv(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & S, int verbosity = 0)
{
    typedef double Scalar;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;

    assert(x.cols() == 1);
    g.resize(x.size());
    assert(S.isUpperTriangular());

    // Storage for trial point and gradient
    Vector xn(x.size());
    Vector gn(x.size());

    // Evaluate initial cost and gradient
    Scalar f = costFunc(x, g);
    if (!std::isfinite(f) || !g.allFinite())        // if any nan, -inf or +inf
    {
        if (verbosity > 1)
            std::println("ERROR: Initial point is not in domain of cost function");
        return -1;
    }

    Scalar Delta = 10e0;     // Initial trust-region radius

    const int maxIterations = 5000;
    for (int i = 0; i < maxIterations; ++i)
    {
        // Solve trust-region subproblem
        Vector p;
        trsSqrtInv(S, g, Delta, p);   // minimise 0.5*p.'*inv(S.'*S)*p + g.'*p subject to ||inv(S.')*p|| <= Delta
        // Solve S.'*z = p for z
        Vector z = S.triangularView<Eigen::Upper>().transpose().solve(p);

        Scalar pg = p.dot(g);
        Scalar LambdaSq = -pg;  // The Newton decrement squared is g.'*inv(H)*g = p.'*H*p = p.'*inv(S.'*S)*p
        if (verbosity == 3)
            std::println("Iter = {:5}, Cost = {:10.2e}, Newton decr^2 = {:10.2e}, Delta = {:10.2e}", i, f, LambdaSq, Delta);;
        if (verbosity == 1)
            std::print(".");

        // const Scalar LambdaSqThreshold = std::sqrt(std::numeric_limits<Scalar>::epsilon());     // Loose convergence tolerance
        const Scalar LambdaSqThreshold = 2*std::numeric_limits<Scalar>::epsilon();              // Tight convergence tolerance
        if (std::fabs(LambdaSq) < LambdaSqThreshold)
        {
            if (verbosity >= 2)
                std::println("CONVERGED: Newton decrement below threshold in {} iterations", i);
            return 0;
        }

        // Evaluate cost and gradient for trial step
        xn = x + p;
        Scalar fn = costFunc(xn, gn);
        bool outOfDomain = !std::isfinite(fn) || !gn.allFinite();   // if any nan, -inf or +inf
        Vector y = gn - g;

        Scalar rho;
        if (outOfDomain)
        {
            rho = -1;                           // Force trust region reduction and reject step
        }
        else
        {
            Scalar dm = -pg - 0.5*z.squaredNorm();   // Predicted reduction f - fp, where fp = f + p'*g + 0.5*p'*H*p
            rho = (f - fn)/dm;                  // Actual reduction divided by predicted reduction
        }

        if (rho < 0.1)
        {
            Delta = 0.25*z.norm();              // Decrease trust region radius
        }
        else
        {
            if (rho > 0.75 && z.norm() > 0.8*Delta)
            {
                Delta = 2.0*Delta;              // Increase trust region radius
            }
        }

        if (rho >= 0.001)
        {
            // Accept the step
            x = xn;
            f = fn;
            g = gn;
        }

        // Update Hessian approximation
        const Scalar sqrteps = std::sqrt(std::numeric_limits<Scalar>::epsilon());
        if (!outOfDomain)
        {
            Scalar py = p.dot(y);
            if (py > sqrteps*y.norm()*p.norm())
            {
                // Form
                // [       p^T/sqrt(y^T*p) ]
                // [ S*(I - y*p^T/(y^T*p)) ]
                Matrix RR(x.size() + 1, x.size());
                RR << p.transpose()/std::sqrt(py),
                      S*(Eigen::MatrixXd::Identity(x.size(), x.size()) - y*p.transpose()/py);
                // Q-less QR
                Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(RR);    // In-place QR decomposition
                S = RR.topRows(x.size()).triangularView<Eigen::Upper>();
            }
        }
    }
    if (verbosity > 1)
        std::println("WARNING: maximum number of iterations reached");
    return 1;
}

/**
 * @brief Minimize f(x) using BFGS trust-region method with inverse Hessian
 *
 * This function minimizes the cost function f(x) using a BFGS trust-region method
 * with inverse Hessian approximation. It computes and returns the Hessian matrix
 * as part of the optimization process.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param[out] H Hessian matrix at the optimal solution
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int BFGSTrustInv(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H, int verbosity = 0)
{
    Eigen::MatrixXd S(x.size(), x.size());
    S.setIdentity();
    int retval = BFGSTrustSqrtInv(costFunc, x, g, S, verbosity);
    Eigen::MatrixXd Xi = S.triangularView<Eigen::Upper>().transpose().solve(Eigen::MatrixXd::Identity(x.size(), x.size()));
    Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixXd>> qr(Xi);       // In-place QR decomposition
    Xi = Xi.triangularView<Eigen::Upper>();                         // Safe aliasing
    H = Xi.transpose()*Xi;
    return retval;
}

/**
 * @brief Minimize f(x) using BFGS trust-region method with inverse Hessian
 *
 * This function minimizes the cost function f(x) using a BFGS trust-region method
 * with inverse Hessian approximation. It does not return the Hessian matrix.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int BFGSTrustInv(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, int verbosity = 0)
{
    Eigen::MatrixXd H(x.size(), x.size());
    return BFGSTrustInv(costFunc, x, g, H, verbosity);
}

/**
 * @brief Minimize f(x) using BFGS trust-region method with inverse Hessian
 *
 * This function minimizes the cost function f(x) using a BFGS trust-region method
 * with inverse Hessian approximation. It does not return the gradient or Hessian matrix.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int BFGSTrustInv(Func costFunc, Eigen::VectorXd & x, int verbosity = 0)
{
    Eigen::VectorXd g(x.size());
    return BFGSTrustInv(costFunc, x, g, verbosity);
}

/**
 * @brief Minimize f(x) using Levenberg-Marquardt quasi-Newton BFGS method with square-root Hessian
 *
 * This function minimizes the cost function f(x) using a Levenberg-Marquardt quasi-Newton BFGS
 * (Broyden-Fletcher-Goldfarb-Shanno) method. It uses a square-root representation of the Hessian
 * matrix for improved numerical stability.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param[in,out] Xi Square-root of the Hessian matrix (upper triangular)
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int BFGSLMSqrt(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & Xi, int verbosity = 0)
{
    typedef double Scalar;
    typedef Eigen::VectorX<Scalar> Vector;
    typedef Eigen::MatrixX<Scalar> Matrix;

    assert(x.cols() == 1);
    g.resize(x.size());
    assert(Xi.isUpperTriangular());

    // Storage for trial point and gradient
    Vector xn(x.size());
    Vector gn(x.size());

    // Evaluate initial cost and gradient
    Scalar f = costFunc(x, g);
    if (!std::isfinite(f) || !g.allFinite())        // if any nan, -inf or +inf
    {
        if (verbosity > 1)
            std::println("ERROR: Initial point is not in domain of cost function");
        return -1;
    }

    // Scalar maxDiagH = (Xi.transpose()*Xi).diagonal().maxCoeff();
    // Scalar Delta = 10e0;     // Initial trust-region radius
    Scalar lambda = 1e-7;
    // Scalar lambda = 1e-10*(Xi.transpose()*Xi).diagonal().maxCoeff();
    // Scalar lambda = 1e-2;
    // Scalar nu = 2.0;
    // Scalar alpha = 0.00;         // 0: robust, 1: fast

    const int maxIterations = 5000;
    for (int i = 0; i < maxIterations; ++i)
    {
        // Solve the secular equation (Xi.'*Xi + lambda*I)*p = -g for p
        Matrix XiI(Xi.rows() + Xi.cols(), Xi.cols());
        XiI << Xi,
              std::sqrt(lambda)*Matrix::Identity(Xi.cols(), Xi.cols());
        Eigen::HouseholderQR<Eigen::Ref<Matrix>> qr(XiI);    // In-place QR decomposition
        Vector p = -XiI.topRows(Xi.cols()).triangularView<Eigen::Upper>().solve(
            XiI.topRows(Xi.cols()).triangularView<Eigen::Upper>().transpose().solve(g)
        );

        // // Solve ( H + lambda*diag(H) )*p = -g for p
        // Matrix XiI(Xi.rows() + Xi.cols(), Xi.cols());
        // Vector d = Xi.array().square().colwise().sum(); // diag(H)
        // Scalar tol = Xi.cols()*std::numeric_limits<Scalar>::epsilon()*d.maxCoeff();
        // d.cwiseMax(tol);
        // Matrix sqrtLambdaH = (lambda*d).cwiseSqrt().asDiagonal();
        // XiI << Xi, sqrtLambdaH;
        // Eigen::HouseholderQR<Eigen::Ref<Matrix>> qrXiI(XiI);    // In-place QR decomposition
        // Vector p = -XiI.topRows(Xi.cols()).triangularView<Eigen::Upper>().solve(
        //     XiI.topRows(Xi.cols()).triangularView<Eigen::Upper>().transpose().solve(g)
        // );

        // trsSqrt(Xi, g, Delta, p);   // minimise 0.5*p.'*Xi.'*Xi*p + g.'*p subject to ||Xi*p|| <= Delta
        Vector z = Xi*p;

        Scalar pg = p.dot(g);
        Scalar NewtonDecrSq = -pg;  // The Newton decrement squared is g.'*inv(H)*g = p.'*H*p = p.'*Xi.'*Xi*p if p is the Newton step
        // Scalar NewtonDecrSq = z.squaredNorm();
        if (verbosity == 3)
            std::println("Iter = {:5}, Cost = {:10.2e}, Newton decr^2 = {:10.2e}, Lambda = {:10.2e}", i, f, NewtonDecrSq, lambda);
        if (verbosity == 1)
            std::print(".");

        // const Scalar NewtonDecrSqThreshold = std::sqrt(std::numeric_limits<Scalar>::epsilon());     // Loose convergence tolerance
        // const Scalar NewtonDecrSqThreshold = 2*std::numeric_limits<Scalar>::epsilon();              // Tight convergence tolerance
        const Scalar NewtonDecrSqThreshold = 1e3*std::numeric_limits<Scalar>::epsilon();
        if (std::fabs(NewtonDecrSq) < NewtonDecrSqThreshold)
        {
            if (verbosity >= 2)
                std::println("CONVERGED: Newton decrement below threshold in {} iterations", i);
            return 0;
        }

        // Evaluate cost and gradient for trial step
        xn = x + p;
        Scalar fn = costFunc(xn, gn);
        bool outOfDomain = !std::isfinite(fn) || !gn.allFinite();   // if any nan, -inf or +inf
        Vector y = gn - g;

        Scalar rho;
        if (outOfDomain)
        {
            rho = -1;                           // Force trust region reduction and reject step
        }
        else
        {
            Scalar dm = -pg - 0.5*z.squaredNorm();   // Predicted reduction f - fp, where fp = f + p'*g + 0.5*p'*H*p
            rho = (f - fn)/dm;                  // Actual reduction divided by predicted reduction
        }

        if (rho < 0.25)
        {
            lambda = std::min(lambda*11.0, 1e7);
        }
        else
        {
            if (rho > 0.75 /*&& lambda*p.norm() > 0.8*/ /*&& lambda*p.norm() > 0.8*g.norm()*/)
            {
                lambda = std::max(lambda/9.0, 1e-7);
            }
        }

        if (rho >= 0.001)
        {
            // Accept the step
            x = xn;
            f = fn;
            g = gn;
        }

        // Update Hessian approximation
        const Scalar sqrteps = std::sqrt(std::numeric_limits<Scalar>::epsilon());
        if (!outOfDomain)
        {
            Scalar py = p.dot(y);
            if (py > sqrteps*y.norm()*p.norm())
            {
                // Form
                // [ Xi*p,              Xi ]
                // [    0, y^T/sqrt(y^T*p) ]
                Matrix RR(Xi.rows() + 1, Xi.cols() + 1);
                RR << Xi*p, Xi,
                      0, y.transpose()/std::sqrt(py);
                // Q-less QR yields
                // [ R1, R2 ]
                // [  0, R3 ]
                Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(RR);    // In-place QR decomposition
                Xi = RR.bottomRightCorner(Xi.rows(), Xi.cols()).triangularView<Eigen::Upper>();
            }
        }
    }
    if (verbosity > 1)
        std::println("WARNING: maximum number of iterations reached");
    return 1;
}

/**
 * @brief Minimize f(x) using BFGS Levenberg-Marquardt method
 *
 * This function minimizes the cost function f(x) using a BFGS Levenberg-Marquardt method.
 * It computes and returns the Hessian matrix as part of the optimization process.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param[out] H Hessian matrix at the optimal solution
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int BFGSLM(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H, int verbosity = 0)
{
    Eigen::MatrixXd Xi(x.size(), x.size());
    Xi.setIdentity();
    int retval = BFGSLMSqrt(costFunc, x, g, Xi, verbosity);
    H = Xi.transpose()*Xi;
    return retval;
}

/**
 * @brief Minimize f(x) using BFGS Levenberg-Marquardt method
 *
 * This function minimizes the cost function f(x) using a BFGS Levenberg-Marquardt method.
 * It does not return the Hessian matrix.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param[out] g Gradient vector at the optimal solution
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int BFGSLM(Func costFunc, Eigen::VectorXd & x, Eigen::VectorXd & g, int verbosity = 0)
{
    Eigen::MatrixXd H(x.size(), x.size());
    return BFGSLM(costFunc, x, g, H, verbosity);
}

/**
 * @brief Minimize f(x) using BFGS Levenberg-Marquardt method
 *
 * This function minimizes the cost function f(x) using a BFGS Levenberg-Marquardt method.
 * It does not return the gradient or Hessian matrix.
 *
 * @tparam Func Type of the cost function
 * @param costFunc Cost function that returns f(x) and computes gradient
 * @param[in,out] x Initial guess on input, optimal solution on output
 * @param verbosity Verbosity level (0: silent, 1: dots, 2: summary, 3: iteration details)
 * @return 0 on success, non-zero on failure
 */
template <typename Func>
int BFGSLM(Func costFunc, Eigen::VectorXd & x, int verbosity = 0)
{
    Eigen::VectorXd g(x.size());
    return BFGSLM(costFunc, x, g, verbosity);
}


}  // namespace utility::slam::funcmin

#endif  // UTILITY_SLAM_FUNCMIN_HPP
