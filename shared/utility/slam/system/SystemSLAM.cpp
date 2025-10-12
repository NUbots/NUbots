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

#include "SystemSLAM.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include <opencv2/core/mat.hpp>

#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

#include "../rotation.hpp"

namespace utility::slam::system {

    SystemSLAM::SystemSLAM(const gaussian::GaussianInfo<double>& density) : SystemEstimator(density) {}

    // Evaluate f(x) from the SDE dx = f(x)*dt + dw
    Eigen::VectorXd SystemSLAM::dynamics(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
        assert(density.dim() == x.size());
        //
        //  dnu/dt =          0 + dwnu/dt
        // deta/dt = JK(eta)*nu +       0
        //   dm/dt =          0 +       0
        // \_____/   \________/   \_____/
        //  dx/dt  =    f(x)    +  dw/dt
        //
        //        [          0 ]
        // f(x) = [ JK(eta)*nu ]
        //        [          0 ] for all map states
        //
        //        [                    0 ]
        //        [                    0 ]
        // f(x) = [    Rnb(thetanb)*vBNb ]
        //        [ TK(thetanb)*omegaBNb ]
        //        [                    0 ] for all map states
        //
        Eigen::VectorXd f(x.size());
        f.setZero();
        // set nu and eta
        Eigen::Vector6d nu      = x.segment<6>(0);
        Eigen::Vector6d eta     = x.segment<6>(6);
        Eigen::Matrix<double, 6, 6> J_eta = eulerKinematicTransformation(eta);
        f.segment<6>(6)         = J_eta * nu;
        return f;
    }

    // Evaluate f(x) and its Jacobian J = df/fx from the SDE dx = f(x)*dt + dw
    Eigen::VectorXd SystemSLAM::dynamics(double t,
                                        const Eigen::VectorXd& x,
                                        const Eigen::VectorXd& u,
                                        Eigen::MatrixXd& J) const {
        Eigen::VectorXd f = dynamics(t, x, u);

        // Jacobian J = df/dx
        //
        //     [  0                  0 0 ]
        // J = [ JK d(JK(eta)*nu)/deta 0 ]
        //     [  0                  0 0 ]
        //
        J.resize(f.size(), x.size());
        J.setZero();
        // build dRnb(thetanb)vBNb/dvBNb
        // extract states
        double ustate = x(0);
        double v      = x(1);
        double w      = x(2);
        double q      = x(4);
        double r      = x(5);
        double phi    = x(9);
        double theta  = x(10);
        double psi    = x(11);
        Eigen::Vector3d Thetanb = Eigen::Vector3d(phi, theta, psi);
        double cphi             = std::cos(phi);
        double sphi             = std::sin(phi);
        double ctheta           = std::cos(theta);
        double stheta           = std::sin(theta);
        double cpsi             = std::cos(psi);
        double spsi             = std::sin(psi);
        double ttheta           = std::tan(theta);
        double sec_theta        = 1 / std::cos(theta);
        // build matrix
        Eigen::Matrix3d dRnb_dvBNb      = rpy2rot(Thetanb);
        Eigen::Matrix3d dRnb_dThetanb   = Eigen::Matrix3d::Zero();
        dRnb_dThetanb(0, 0) = (spsi * sphi + cpsi * stheta * cphi) * v + (spsi * cphi - cpsi * sphi * stheta) * w;  // df1/dphi
        dRnb_dThetanb(0, 1) =
            -cpsi * stheta * ustate + cpsi * ctheta * sphi * v + cpsi * cphi * ctheta * w;  // df1/dtheta
        dRnb_dThetanb(0, 2) = -spsi * ctheta * ustate - (cpsi * cphi + spsi * stheta * sphi) * v
                            + (cpsi * sphi - spsi * cphi * stheta) * w;  // df1/dpsi
        dRnb_dThetanb(1, 0) =
            (-cpsi * sphi + cphi * stheta * spsi) * v + (-cpsi * cphi - spsi * sphi * stheta) * w;  // df2/dphi
        dRnb_dThetanb(1, 1) =
            -spsi * stheta * ustate + sphi * ctheta * spsi * v + spsi * cphi * ctheta * w;  // df2/dtheta
        dRnb_dThetanb(1, 2) = cpsi * ctheta * ustate - (spsi * cphi - sphi * stheta * cpsi) * v
                            + (spsi * sphi + cpsi * cphi * stheta) * w;  // df2/dpsi
        dRnb_dThetanb(2, 0) = ctheta * cphi * v - ctheta * sphi * w;       // df3/dphi
        dRnb_dThetanb(2, 1) = -ctheta * ustate - stheta * sphi * v - stheta * cphi * w;  // df3/dtheta
        dRnb_dThetanb(2, 2) = 0;                                                          // df3/dpsi
        // build matrix dTK(thetanb)/dThetanb
        Eigen::Matrix3d dTk_dwBNb       = TK(Thetanb);
        Eigen::Matrix3d dTK_dThetanb    = Eigen::Matrix3d::Zero();
        dTK_dThetanb(0, 0)              = cphi * ttheta * q - sphi * ttheta * r;                      // df1/dphi
        dTK_dThetanb(0, 1)              = (sphi / (ctheta * ctheta)) * q + (cphi / (ctheta * ctheta)) * r;  // df1/dtheta
        dTK_dThetanb(1, 0)              = -sphi * q - cphi * r;                                       // df2/dphi
        dTK_dThetanb(2, 0)              = (cphi / ctheta) * q - (sphi / ctheta) * r;                  // df3/dphi
        dTK_dThetanb(2, 1) = sphi * ttheta * (1 / ctheta) * q + cphi * ttheta * (1 / ctheta) * r;  // df3/dtheta
        // build jacobian from above
        J.block<3, 3>(6, 0) = dRnb_dvBNb;
        J.block<3, 3>(6, 9) = dRnb_dThetanb;
        J.block<3, 3>(9, 3) = dTk_dwBNb;
        J.block<3, 3>(9, 9) = dTK_dThetanb;
        return f;
    }

    Eigen::VectorXd SystemSLAM::input(double t, const Eigen::VectorXd& x) const {
        return Eigen::VectorXd(0);
    }

    gaussian::GaussianInfo<double> SystemSLAM::processNoiseDensity(double dt) const {
        // SQ is an upper triangular matrix such that SQ.'*SQ = Q is the power spectral density of the continuous time
        // process noise

        // Tuning parameters
        const double sigma_vx = 0.08;  // m/s / sqrt(s)
        const double sigma_vy = 0.08;  // m/s / sqrt(s)
        const double sigma_vz = 0.04;  // m/s / sqrt(s)
        const double sigma_p  = 0.06;  // rad/s / sqrt(s)
        const double sigma_q  = 0.06;  // rad/s / sqrt(s)
        const double sigma_r  = 0.09;  // rad/s / sqrt(s)

        Eigen::Matrix<double, 6, 6> SQ = Eigen::Matrix<double, 6, 6>::Zero();
        SQ(0, 0)                       = sigma_vx;
        SQ(1, 1)                       = sigma_vy;
        SQ(2, 2)                       = sigma_vz;
        SQ(3, 3)                       = sigma_p;
        SQ(4, 4)                       = sigma_q;
        SQ(5, 5)                       = sigma_r;


        // Distribution of noise increment dw ~ N(0, Q*dt) for time increment dt
        return gaussian::GaussianInfo<double>::fromSqrtMoment(SQ * std::sqrt(dt));
    }

    std::vector<Eigen::Index> SystemSLAM::processNoiseIndex() const {
        // Indices of process model equations where process noise is injected
        std::vector<Eigen::Index> idxQ;
        idxQ = {0, 1, 2, 3, 4, 5};
        return idxQ;
    }

    cv::Mat& SystemSLAM::view() {
        return view_;
    };

    const cv::Mat& SystemSLAM::view() const {
        return view_;
    };

    gaussian::GaussianInfo<double> SystemSLAM::bodyPositionDensity() const {
        return density.marginal(Eigen::seqN(6, 3));
    }

    gaussian::GaussianInfo<double> SystemSLAM::bodyOrientationDensity() const {
        return density.marginal(Eigen::seqN(9, 3));
    }

    gaussian::GaussianInfo<double> SystemSLAM::bodyTranslationalVelocityDensity() const {
        return density.marginal(Eigen::seqN(0, 3));
    }

    gaussian::GaussianInfo<double> SystemSLAM::bodyAngularVelocityDensity() const {
        return density.marginal(Eigen::seqN(3, 3));
    }

    Eigen::Vector3d SystemSLAM::cameraPosition(const Camera& camera, const Eigen::VectorXd& x, Eigen::MatrixXd& J) {
        Eigen::Vector3<autodiff::dual> rCNn_dual;
        Eigen::VectorX<autodiff::dual> x_dual = x.cast<autodiff::dual>();
        J = jacobian(cameraPosition<autodiff::dual>, wrt(x_dual), at(camera, x_dual), rCNn_dual);
        return rCNn_dual.cast<double>();
    };

    gaussian::GaussianInfo<double> SystemSLAM::cameraPositionDensity(const Camera& camera) const {
        auto f = [&](const Eigen::VectorXd& x, Eigen::MatrixXd& J) { return cameraPosition(camera, x, J); };
        return density.affineTransform(f);
    }

    Eigen::Vector3d SystemSLAM::cameraOrientationEuler(const Camera& camera,
                                                        const Eigen::VectorXd& x,
                                                        Eigen::MatrixXd& J) {
        Eigen::Vector3<autodiff::dual> Thetanc_dual;
        Eigen::VectorX<autodiff::dual> x_dual = x.cast<autodiff::dual>();
        J = jacobian(cameraOrientationEuler<autodiff::dual>, wrt(x_dual), at(camera, x_dual), Thetanc_dual);
        return Thetanc_dual.cast<double>();
    };

    gaussian::GaussianInfo<double> SystemSLAM::cameraOrientationEulerDensity(const Camera& camera) const {
        auto f = [&](const Eigen::VectorXd& x, Eigen::MatrixXd& J) { return cameraOrientationEuler(camera, x, J); };
        return density.affineTransform(f);
    }

    gaussian::GaussianInfo<double> SystemSLAM::landmarkPositionDensity(std::size_t idxLandmark) const {
        assert(idxLandmark < numberLandmarks());
        std::size_t idx = landmarkPositionIndex(idxLandmark);
        return density.marginal(Eigen::seqN(idx, 3));
    }

}  // namespace utility::slam::system
