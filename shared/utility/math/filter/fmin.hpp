#ifndef NEWTON_HPP
#define NEWTON_HPP

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>

int trs(const Eigen::MatrixXd& Q, const Eigen::VectorXd& v, const Eigen::VectorXd& g, double D, Eigen::VectorXd& p);
int trs(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, double D, Eigen::VectorXd& p);

/**
 * @brief Minimise f(x) using trust-region Newton method
 */

template <typename Func>
int fminNewtonTrustEig(Func costFunc,
                       Eigen::VectorXd& x,
                       Eigen::VectorXd& g,
                       Eigen::MatrixXd& Q,
                       Eigen::VectorXd& v,
                       int verbosity = 0) {
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

    // Eigendecomposition of initial Hessian
    Eigen::SelfAdjointEigenSolver<Matrix> eigenH(H);
    v            = eigenH.eigenvalues();
    Q            = eigenH.eigenvectors();
    Scalar Delta = 1e0;  // Initial trust-region radius

    const int maxIterations = 50;
    for (int i = 0; i < maxIterations; ++i) {
        // Solve trust-region subproblem
        Vector p;
        trs(Q, v, g, Delta, p);  // minimise 0.5*p.'*H*p + g.'*p subject to ||p|| <= Delta

        Scalar pg       = p.dot(g);
        Scalar LambdaSq = -pg;  // The Newton decrement squared is g.'*inv(H)*g = p.'*H*p
        if (verbosity == 3)
            std::printf("Iter = %5i, Cost = %10.2e, Newton decr^2 = %10.2e, Delta = %10.2e\n", i, f, LambdaSq, Delta);
        if (verbosity == 1)
            std::printf(".");

        const Scalar LambdaSqThreshold = 2 * std::numeric_limits<Scalar>::epsilon();
        if (std::fabs(LambdaSq) < LambdaSqThreshold) {
            if (verbosity >= 2)
                std::printf("CONVERGED: Newton decrement below threshold in %i iterations\n", i);
            return 0;
        }

        // Evaluate cost, gradient and Hessian for trial step
        xn = x + p;

        Scalar fn  = costFunc(xn, gn, Hn);
        Scalar dm  = -pg - 0.5 * p.dot(H * p);  // Predicted reduction f - fp, where fp = f + p'*g + 0.5*p'*H*p
        Scalar rho = (f - fn) / dm;             // Actual reduction divided by predicted reduction

        if (rho < 0.1) {
            // Decrease trust region radius
            Delta = 0.25 * p.norm();
        }
        else {
            if (rho > 0.75 && p.norm() > 0.8 * Delta) {
                // Increase trust region radius
                Delta = 2.0 * Delta;
            }
        }

        if (rho >= 0.001) {
            // Accept the step
            x = xn;
            f = fn;
            g = gn;
            H = Hn;

            // Eigendecomposition of accepted Hessian
            Eigen::SelfAdjointEigenSolver<Matrix> eigenH(H);
            v = eigenH.eigenvalues();
            Q = eigenH.eigenvectors();
        }
    }
    if (verbosity > 1)
        std::printf("WARNING: maximum number of iterations reached\n");
    return 1;
}

/**
 * @brief  Minimise f(x) using trust-region quasi-Newton SR1 method
 */
template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
template <typename Func>
int fminSR1TrustEig(Func costFunc,
                    Eigen::VectorXd& x,
                    Eigen::VectorXd& g,
                    Eigen::MatrixXd& Q,
                    Eigen::VectorXd& v,
                    int verbosity = 0) {
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

    // Evaluate initial cost and gradient
    Scalar f = costFunc(x, g);

    // Initial Hessian
    H = Q * v.asDiagonal() * Q.transpose();

    Scalar Delta = 1e0;  // Initial trust-region radius

    const int maxIterations = 100;  // 5000
    for (int i = 0; i < maxIterations; ++i) {
        // Solve trust-region subproblem
        Vector p;
        trs(Q, v, g, Delta, p);  // minimise 0.5*p.'*H*p + g.'*p subject to ||p|| <= Delta

        Scalar pg       = p.dot(g);
        Scalar LambdaSq = -pg;  // The Newton decrement squared is g.'*inv(H)*g = p.'*H*p
        if (verbosity == 3)
            std::printf("Iter = %5i, Cost = %10.2e, Newton decr^2 = %10.2e, Delta = %10.2e\n", i, f, LambdaSq, Delta);
        if (verbosity == 1)
            std::printf(".");

        const Scalar LambdaSqThreshold = 2 * std::numeric_limits<Scalar>::epsilon();
        if (std::fabs(LambdaSq) < LambdaSqThreshold && v(0) > 0.0) {
            if (verbosity >= 2)
                std::printf("CONVERGED: Newton decrement below threshold in %i iterations\n", i);
            return 0;
        }

        // Evaluate cost and gradient for trial step
        xn        = x + p;
        Scalar fn = costFunc(xn, gn);
        Vector y  = gn - g;

        Scalar dm  = -pg - 0.5 * p.dot(H * p);  // Predicted reduction f - fp, where fp = f + p'*g + 0.5*p'*H*p
        Scalar rho = (f - fn) / dm;             // Actual reduction divided by predicted reduction

        if (rho < 0.1) {
            // Decrease trust region radius
            Delta = 0.25 * p.norm();
        }
        else {
            if (rho > 0.75 && p.norm() > 0.8 * Delta) {
                // Increase trust region radius
                Delta = 2.0 * Delta;
            }
        }

        if (rho >= 0.001) {
            // Accept the step
            x = xn;
            f = fn;
            g = gn;
        }

        // Update Hessian approximation
        const Scalar sqrteps = std::sqrt(std::numeric_limits<Scalar>::epsilon());
        Vector w             = y - H * p;
        Scalar pw            = p.dot(w);
        if (std::fabs(pw) > sqrteps * p.norm() * w.norm()) {
            Scalar s = sgn(pw);
            Vector u = w.array() / std::sqrt(s * pw);
            H        = H + s * (u * u.transpose());

            // Eigendecomposition of updated Hessian
            Eigen::SelfAdjointEigenSolver<Matrix> eigenH(H);
            v = eigenH.eigenvalues();
            Q = eigenH.eigenvectors();
            // TODO: Do a rank-one update of Q and v instead of a full eigendecomposition using, e.g.,
            //  [1] Bunch, J.R., Nielsen, C.P. and Sorensen, D.C., 1978. Rank-one modification of the symmetric
            //  eigenproblem. Numerische Mathematik, 31(1), pp.31-48.
        }
    }
    if (verbosity > 1)
        std::printf("WARNING: maximum number of iterations reached\n");
    return 1;
}

template <typename Func>
int fminNewtonTrust(Func costFunc, Eigen::VectorXd& x, Eigen::VectorXd& g, Eigen::MatrixXd& H, int verbosity = 0) {
    typedef double Scalar;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;

    Matrix Q(x.size(), x.size());
    Vector v(x.size());

    int retval = fminNewtonTrustEig(costFunc, x, g, Q, v, verbosity);
    H          = Q * v.asDiagonal() * Q.transpose();
    return retval;
}

template <typename Func>
int fminNewtonTrust(Func costFunc, Eigen::VectorXd& x, Eigen::VectorXd& g, int verbosity = 0) {
    typedef double Scalar;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;

    Matrix H(x.size(), x.size());
    return fminNewtonTrust(costFunc, x, g, H, verbosity);
}

template <typename Func>
int fminNewtonTrust(Func costFunc, Eigen::VectorXd& x, int verbosity = 0) {
    typedef double Scalar;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;

    Vector g(x.size());
    return fminNewtonTrust(costFunc, x, g, verbosity);
}


#endif
