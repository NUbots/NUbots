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

#ifndef UTILITY_SLAM_CAMERA_CAMERA_HPP
#define UTILITY_SLAM_CAMERA_CAMERA_HPP

#include <cmath>
#include <filesystem>
#include <vector>

#include <Eigen/Core>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>

#include "Pose.hpp"
#include "../serialisation.hpp"

// Define Eigen types in Eigen namespace
namespace Eigen {
    using Matrix23d = Eigen::Matrix<double, 2, 3>;
    using Vector6d  = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

namespace utility::slam::camera {

    struct Chessboard {
        cv::Size boardSize;
        float squareSize;

        void write(cv::FileStorage& fs) const;    // OpenCV serialisation
        void read(const cv::FileNode& node);      // OpenCV serialisation

        std::vector<cv::Point3f> gridPoints() const;
        friend std::ostream& operator<<(std::ostream&, const Chessboard&);
    };

    // Free functions for OpenCV serialization (required for operator>> to work)
    inline void read(const cv::FileNode& node, Chessboard& value, const Chessboard& default_value = Chessboard()) {
        if (node.empty())
            value = default_value;
        else
            value.read(node);
    }

    inline void write(cv::FileStorage& fs, const std::string&, const Chessboard& value) {
        value.write(fs);
    }

    struct Camera;

    struct ChessboardImage {
        ChessboardImage(const cv::Mat&, const Chessboard&, const std::filesystem::path& = "");
        cv::Mat image;
        std::filesystem::path filename;
        Pose<double> Tnc;                      // Extrinsic camera parameters
        std::vector<cv::Point2f> corners;      // Chessboard corners in image [rQOi]
        bool isFound;
        void drawCorners(const Chessboard&);
        void drawBox(const Chessboard&, const Camera&);
        void recoverPose(const Chessboard&, const Camera&);
    };

    struct ChessboardData {
        explicit ChessboardData(const std::filesystem::path&);  // Load from config file

        Chessboard chessboard;
        std::vector<ChessboardImage> chessboardImages;

        void drawCorners();
        void drawBoxes(const Camera&);
        void recoverPoses(const Camera&);
    };

    struct Camera {
        void calibrate(ChessboardData&);  // Calibrate camera from chessboard data
        void printCalibration() const;

        template <typename Scalar>
        Pose<Scalar> cameraToBody(const Pose<Scalar>& Tnc) const {
            return Tnc * Tbc.inverse();
        }  // Tnb = Tnc*Tcb
        template <typename Scalar>
        Pose<Scalar> bodyToCamera(const Pose<Scalar>& Tnb) const {
            return Tnb * Tbc;
        }  // Tnc = Tnb*Tbc
        cv::Vec3d worldToVector(const cv::Vec3d& rPNn, const Pose<double>& Tnb) const;
        cv::Vec2d worldToPixel(const cv::Vec3d&, const Pose<double>&) const;
        cv::Vec2d vectorToPixel(const cv::Vec3d&) const;
        template <typename Scalar>
        Eigen::Vector2<Scalar> vectorToPixel(const Eigen::Vector3<Scalar>&) const;
        Eigen::Vector2d vectorToPixel(const Eigen::Vector3d&, Eigen::Matrix23d&) const;

        cv::Vec3d pixelToVector(const cv::Vec2d&) const;

        bool isWorldWithinFOV(const cv::Vec3d& rPNn, const Pose<double>& Tnb) const;
        bool isVectorWithinFOV(const cv::Vec3d& rPCc) const;

        void calcFieldOfView();
        void write(cv::FileStorage&) const;  // OpenCV serialisation
        void read(const cv::FileNode&);      // OpenCV serialisation

        cv::Mat cameraMatrix;   // Camera matrix
        cv::Mat distCoeffs;     // Lens distortion coefficients
        int flags = 0;          // Calibration flags
        cv::Size imageSize;     // Image size

        Pose<double> Tbc;  // Relative pose of camera in body coordinates (Rbc, rCBb)

    private:
        double hFOV = 0.0;  // Horizonal field of view
        double vFOV = 0.0;  // Vertical field of view
        double dFOV = 0.0;  // Diagonal field of view
    };

    template <typename Scalar>
    Eigen::Vector2<Scalar> Camera::vectorToPixel(const Eigen::Vector3<Scalar>& rPCc) const {
        bool isRationalModel  = (flags & cv::CALIB_RATIONAL_MODEL) == cv::CALIB_RATIONAL_MODEL;
        bool isThinPrismModel = (flags & cv::CALIB_THIN_PRISM_MODEL) == cv::CALIB_THIN_PRISM_MODEL;
        assert(isRationalModel && isThinPrismModel);

        // Camera intrinsics
        const Scalar fx = Scalar(cameraMatrix.at<double>(0, 0));
        const Scalar fy = Scalar(cameraMatrix.at<double>(1, 1));
        const Scalar cx = Scalar(cameraMatrix.at<double>(0, 2));
        const Scalar cy = Scalar(cameraMatrix.at<double>(1, 2));

        // Distortion coefficients (OpenCV order)
        Scalar k1(0), k2(0), p1(0), p2(0), k3(0), k4(0), k5(0), k6(0);
        Scalar s1(0), s2(0), s3(0), s4(0);
        if (distCoeffs.total() > 0)
            k1 = Scalar(distCoeffs.at<double>(0));
        if (distCoeffs.total() > 1)
            k2 = Scalar(distCoeffs.at<double>(1));
        if (distCoeffs.total() > 2)
            p1 = Scalar(distCoeffs.at<double>(2));
        if (distCoeffs.total() > 3)
            p2 = Scalar(distCoeffs.at<double>(3));
        if (distCoeffs.total() > 4)
            k3 = Scalar(distCoeffs.at<double>(4));
        if (distCoeffs.total() > 5)
            k4 = Scalar(distCoeffs.at<double>(5));
        if (distCoeffs.total() > 6)
            k5 = Scalar(distCoeffs.at<double>(6));
        if (distCoeffs.total() > 7)
            k6 = Scalar(distCoeffs.at<double>(7));
        if (distCoeffs.total() > 8)
            s1 = Scalar(distCoeffs.at<double>(8));
        if (distCoeffs.total() > 9)
            s2 = Scalar(distCoeffs.at<double>(9));
        if (distCoeffs.total() > 10)
            s3 = Scalar(distCoeffs.at<double>(10));
        if (distCoeffs.total() > 11)
            s4 = Scalar(distCoeffs.at<double>(11));

        // Normalized coordinates
        const Scalar X = rPCc(0), Y = rPCc(1), Z = rPCc(2);
        const Scalar u = X / Z;
        const Scalar v = Y / Z;

        // Radius powers
        const Scalar r2 = u * u + v * v;
        const Scalar r4 = r2 * r2;
        const Scalar r6 = r4 * r2;

        // Rational radial distortion
        const Scalar alpha = k1 * r2 + k2 * r4 + k3 * r6;
        const Scalar beta  = k4 * r2 + k5 * r4 + k6 * r6;
        const Scalar c     = (Scalar(1) + alpha) / (Scalar(1) + beta);

        // Apply distortions
        const Scalar u_prime =
            c * u + (Scalar(2) * p1 * u * v + p2 * (r2 + Scalar(2) * u * u)) + (s1 * r2 + s2 * r4);
        const Scalar v_prime =
            c * v + (p1 * (r2 + Scalar(2) * v * v) + Scalar(2) * p2 * u * v) + (s3 * r2 + s4 * r4);

        // Final pixel coordinates
        Eigen::Vector2<Scalar> rQOi;
        rQOi(0) = fx * u_prime + cx;
        rQOi(1) = fy * v_prime + cy;
        return rQOi;
    }

}  // namespace utility::slam::camera

#endif  // UTILITY_SLAM_CAMERA_CAMERA_HPP
