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

#include "SystemEstimator.hpp"

#include <Eigen/Core>

namespace utility::slam::system {

    SystemEstimator::SystemEstimator(const gaussian::GaussianInfo<double>& density)
        : SystemBase(), density(density) {}

    SystemEstimator::~SystemEstimator() = default;

    void SystemEstimator::predict(double time) {
        double dt = time - time_;
        assert(dt >= 0);
        if (dt == 0.0)
            return;

        // Augment state density with independent noise increment dw ~ N^{-1}(0, LambdaQ/dt)
        // [ x] ~ N^{-1}([ eta ], [ Lambda,          0 ])
        // [dw]         ([   0 ]  [      0, LambdaQ/dt ])

        auto pdw  = processNoiseDensity(dt);  // p(dw(idxQ)[k])
        auto pxdw = density * pdw;            // p(x[k], dw(idxQ)[k]) = p(x[k])*p(dw(idxQ)[k])

        // Phi maps [ x[k]; dw(idxQ)[k] ] to x[k+1]
        auto Phi = [&](const Eigen::VectorXd& xdw, Eigen::MatrixXd& J) { return RK4SDEHelper(xdw, dt, J); };

        // Map p(x[k], dw(idxQ)[k]) to p(x[k+1])
        density = pxdw.affineTransform(Phi);

        time_ = time;
    }

    Eigen::VectorXd SystemEstimator::dynamicsEst(double t, const Eigen::VectorXd& x) const {
        Eigen::VectorXd u = input(t, x);
        return dynamics(t, x, u);
    }

    Eigen::VectorXd SystemEstimator::dynamicsEst(double t, const Eigen::VectorXd& x, Eigen::MatrixXd& J) const {
        Eigen::VectorXd u = input(t, x);
        return dynamics(t, x, u, J);
    }

    // Evaluate F(X) from dX = F(X)*dt + dW
    Eigen::MatrixXd SystemEstimator::augmentedDynamicsEst(double t, const Eigen::MatrixXd& X) const {
        assert(X.size() > 0);
        int nx = X.rows();
        assert(X.cols() == 2 * nx + 1);

        Eigen::VectorXd x = X.col(0);
        Eigen::MatrixXd J;
        Eigen::VectorXd f = dynamicsEst(t, x, J);
        assert(f.rows() == nx);
        assert(J.rows() == nx);
        assert(J.cols() == nx);

        Eigen::MatrixXd dX(nx, 2 * nx + 1);
        dX << f, J * X.block(0, 1, nx, 2 * nx);
        return dX;
    }

    // Map [x[k]; dw(idxQ)[k]] to x[k+1] using RK4
    Eigen::VectorXd SystemEstimator::RK4SDEHelper(const Eigen::VectorXd& xdw, double dt, Eigen::MatrixXd& J) const {
        const std::vector<Eigen::Index>& idxQ = processNoiseIndex();

        const std::size_t nq = idxQ.size();
        const std::size_t nx = xdw.size() - nq;

        Eigen::VectorXd x(nx), dw(nx);
        x = xdw.head(nx);
        dw.setZero();
        dw(idxQ) = xdw.tail(nq);

        // Let \Delta t == n \delta t.
        // Determine minimum of substeps required such that \delta t <= \delta t_{max}
        int nSubsteps = std::max(1, static_cast<int>(std::ceil(dt / dtMaxEst)));
        dt            = dt / nSubsteps;  // \Delta t = n \delta t
        dw            = dw / nSubsteps;  // \Delta w = n \delta w

        typedef Eigen::MatrixXd Matrix;

        // X  = [ x,  dx/dx[k],   dx/dw[k] ]
        // dW = [dw, ddw/dx[k], ddw/ddw[k] ]
        Matrix X(nx, 2 * nx + 1), dW(nx, 2 * nx + 1);
        X << x, Matrix::Identity(nx, nx), Matrix::Zero(nx, nx);
        dW << dw, Matrix::Zero(nx, nx), Matrix::Identity(nx, nx);

        double t = time_;
        for (int j = 0; j < nSubsteps; ++j) {
            Matrix F1, F2, F3, F4;
            F1 = augmentedDynamicsEst(t, X);
            F2 = augmentedDynamicsEst(t + dt / 2, X + (F1 * dt + dW) / 2);
            F3 = augmentedDynamicsEst(t + dt / 2, X + (F2 * dt + dW) / 2);
            F4 = augmentedDynamicsEst(t + dt, X + F3 * dt + dW);
            X  = X + (F1 + 2 * F2 + 2 * F3 + F4) * dt / 6 + dW;
            t  = t + dt;
        }

        // X[k+1] = [ x[k+1], dx[k+1]/dx[k], dx[k+1]/dw[k] ]
        J.resize(nx, nx + nq);
        J << X.middleCols(1, nx), X.middleCols(nx + 1, nx)(Eigen::all, idxQ) / nSubsteps;
        // Since \Delta w = n \delta w, then
        // \frac{\partial \mathbf{x}}{\partial \Delta\mathbf{w}} = \frac{\partial \mathbf{x}}{\partial
        // \delta\mathbf{w}} \frac{1}{n} therefore we divide Jdw by nSubsteps.
        return X.col(0);
    }

}  // namespace utility::slam::system
