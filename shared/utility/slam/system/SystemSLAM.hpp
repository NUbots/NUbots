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

#ifndef UTILITY_SLAM_SYSTEM_SLAM_HPP
#define UTILITY_SLAM_SYSTEM_SLAM_HPP

#include <vector>

#include <Eigen/Core>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>

#include "../camera/Camera.hpp"
#include "../camera/Pose.hpp"
#include "../gaussian/GaussianInfo.hpp"
#include "SystemEstimator.hpp"
#include "../rotation.hpp"

namespace utility::slam::system {

    // Bring camera types into scope for template functions
    using utility::slam::camera::Camera;
    using utility::slam::camera::Pose;

    /*
    * State contains body velocities, body pose and landmark states
    *
    *     [ vBNb     ]  Body translational velocity (body-fixed)
    *     [ omegaBNb ]  Body angular velocity (body-fixed)
    * x = [ rBNn     ]  Body position (world-fixed)
    *     [ Thetanb  ]  Body orientation (world-fixed)
    *     [ m        ]  Landmark map states (undefined in this class)
    *
    */
    class SystemSLAM : public SystemEstimator {
    public:
        explicit SystemSLAM(const gaussian::GaussianInfo<double>& density);
        virtual SystemSLAM* clone() const                                                             = 0;
        virtual Eigen::VectorXd dynamics(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override;
        virtual Eigen::VectorXd dynamics(double t,
                                        const Eigen::VectorXd& x,
                                        const Eigen::VectorXd& u,
                                        Eigen::MatrixXd& J) const override;
        virtual Eigen::VectorXd input(double t, const Eigen::VectorXd& x) const override;
        virtual gaussian::GaussianInfo<double> processNoiseDensity(double dt) const override;
        virtual std::vector<Eigen::Index> processNoiseIndex() const override;

        virtual gaussian::GaussianInfo<double> bodyPositionDensity() const;
        virtual gaussian::GaussianInfo<double> bodyOrientationDensity() const;
        virtual gaussian::GaussianInfo<double> bodyTranslationalVelocityDensity() const;
        virtual gaussian::GaussianInfo<double> bodyAngularVelocityDensity() const;

        template <typename Scalar>
        static Eigen::Vector3<Scalar> cameraPosition(const Camera& cam, const Eigen::VectorX<Scalar>& x);
        static Eigen::Vector3d cameraPosition(const Camera& cam, const Eigen::VectorXd& x, Eigen::MatrixXd& J);
        template <typename Scalar>
        static Eigen::Matrix3<Scalar> cameraOrientation(const Camera& cam, const Eigen::VectorX<Scalar>& x);
        template <typename Scalar>
        static Eigen::Vector3<Scalar> cameraOrientationEuler(const Camera& cam, const Eigen::VectorX<Scalar>& x);
        static Eigen::Vector3d cameraOrientationEuler(const Camera& cam, const Eigen::VectorXd& x, Eigen::MatrixXd& J);

        template <typename Scalar>
        static Eigen::VectorX<Scalar> dynamics(const Eigen::VectorX<Scalar>& x, const Eigen::VectorX<Scalar>& u);

        virtual gaussian::GaussianInfo<double> cameraPositionDensity(const Camera& cam) const;
        virtual gaussian::GaussianInfo<double> cameraOrientationEulerDensity(const Camera& cam) const;

        virtual std::size_t numberLandmarks() const                                 = 0;
        virtual gaussian::GaussianInfo<double> landmarkPositionDensity(std::size_t idxLandmark) const;
        virtual std::size_t landmarkPositionIndex(std::size_t idxLandmark) const = 0;

        cv::Mat& view();
        const cv::Mat& view() const;

    protected:
        cv::Mat view_;
    };

    template <typename Scalar>
    Eigen::Vector3<Scalar> SystemSLAM::cameraPosition(const Camera& camera, const Eigen::VectorX<Scalar>& x) {
        Pose<Scalar> Tnb;
        Tnb.rotationMatrix     = rpy2rot(x.template segment<3>(9));  // Rnb
        Tnb.translationVector  = x.template segment<3>(6);           // rBNn
        Pose<Scalar> Tnc       = camera.bodyToCamera(Tnb);
        return Tnc.translationVector;  // rCNn
    }

    template <typename Scalar>
    Eigen::Matrix3<Scalar> SystemSLAM::cameraOrientation(const Camera& camera, const Eigen::VectorX<Scalar>& x) {
        Pose<Scalar> Tnb;
        Tnb.rotationMatrix    = rpy2rot(x.template segment<3>(9));  // Rnb
        Tnb.translationVector = x.template segment<3>(6);           // rBNn
        Pose<Scalar> Tnc      = camera.bodyToCamera(Tnb);
        return Tnc.rotationMatrix;  // Rnc
    }

    template <typename Scalar>
    Eigen::Vector3<Scalar> SystemSLAM::cameraOrientationEuler(const Camera& camera, const Eigen::VectorX<Scalar>& x) {
        return rot2rpy(cameraOrientation(camera, x));
    }

    template <typename Scalar>
    Eigen::VectorX<Scalar> SystemSLAM::dynamics(const Eigen::VectorX<Scalar>& x, const Eigen::VectorX<Scalar>& u) {
        // Same logic as the non-templated version but with Scalar types
        Eigen::VectorX<Scalar> f(x.size());
        f.setZero();

        // Extract nu and eta
        Eigen::Matrix<Scalar, 6, 1> nu  = x.template segment<6>(0);
        Eigen::Matrix<Scalar, 6, 1> eta = x.template segment<6>(6);

        // Compute kinematic transformation matrix
        Eigen::Matrix<Scalar, 6, 6> J_eta = eulerKinematicTransformation(eta);

        // Apply transformation: f = J_eta * nu
        f.template segment<6>(6) = J_eta * nu;

        return f;
    }

}  // namespace utility::slam::system

#endif  // UTILITY_SLAM_SYSTEM_SLAM_HPP
