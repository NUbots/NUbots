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

#include "Camera.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <format>
#include <limits>
#include <print>
#include <regex>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "../rotation.hpp"
#include "../to_string.hpp"

namespace utility::slam::camera {

    void Chessboard::write(cv::FileStorage& fs) const {
        fs << "{"
        << "grid_width" << boardSize.width << "grid_height" << boardSize.height << "square_size" << squareSize
        << "}";
    }

    void Chessboard::read(const cv::FileNode& node) {
        node["grid_width"] >> boardSize.width;
        node["grid_height"] >> boardSize.height;
        node["square_size"] >> squareSize;
    }

    std::vector<cv::Point3f> Chessboard::gridPoints() const {
        std::vector<cv::Point3f> rPNn_all;
        rPNn_all.reserve(boardSize.height * boardSize.width);
        for (int i = 0; i < boardSize.height; ++i)
            for (int j = 0; j < boardSize.width; ++j)
                rPNn_all.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        return rPNn_all;
    }

    std::ostream& operator<<(std::ostream& os, const Chessboard& chessboard) {
        return os << "boardSize: " << chessboard.boardSize << ", squareSize: " << chessboard.squareSize;
    }

    ChessboardImage::ChessboardImage(const cv::Mat& image_,
                                    const Chessboard& chessboard,
                                    const std::filesystem::path& filename_)
        : image(image_), filename(filename_), isFound(false) {
        // Detect chessboard corners in image and set the corners member
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> detectedCorners;
        isFound = cv::findChessboardCorners(gray,
                                            chessboard.boardSize,
                                            detectedCorners,
                                            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
                                                + cv::CALIB_CB_FAST_CHECK);
        // (optional) Do subpixel refinement of detected corners
        if (isFound) {
            corners = detectedCorners;
            cv::cornerSubPix(gray,
                            corners,
                            cv::Size(11, 11),
                            cv::Size(-1, -1),
                            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
        }
    }

    void ChessboardImage::drawCorners(const Chessboard& chessboard) {
        cv::drawChessboardCorners(image, chessboard.boardSize, corners, isFound);
    }

    void ChessboardImage::drawBox(const Chessboard& chessboard, const Camera& camera) {
        // Define the box height
        double height = 0.23;

        // Calculate the interior corners of the chessboard
        // The interior corners are one square in from each edge
        double minX = 0;                                                  // One square from left edge
        double maxX = (chessboard.boardSize.width - 1) * chessboard.squareSize;   // One square from right edge
        double minY = 0;                                                  // One square from top edge
        double maxY = (chessboard.boardSize.height - 1) * chessboard.squareSize;  // One square from bottom edge

        // Define the base corners of the box (on the chessboard plane, z = 0)
        std::vector<cv::Vec3d> baseCorners = {{minX, minY, 0},  // Bottom-left
                                            {maxX, minY, 0},  // Bottom-right
                                            {maxX, maxY, 0},  // Top-right
                                            {minX, maxY, 0}};  // Top-left

        // Define the top corners of the box (at height = 0.23 m)
        std::vector<cv::Vec3d> topCorners;
        for (const auto& corner : baseCorners) {
            topCorners.push_back(corner + cv::Vec3d(0, 0, -height));
        }

        // Function to draw a line segment between two 3D points, handling curvature due to distortion
        auto drawCurvedLine = [&](const cv::Vec3d& P1,
                                const cv::Vec3d& P2,
                                const cv::Scalar& color,
                                int thickness   = 4,
                                int numSegments = 1000) {
            for (int i = 0; i < numSegments; ++i) {
                double t1 = static_cast<double>(i) / numSegments;
                double t2 = static_cast<double>(i + 1) / numSegments;

                cv::Vec3d point1 = P1 + t1 * (P2 - P1);
                cv::Vec3d point2 = P1 + t2 * (P2 - P1);

                if (camera.isWorldWithinFOV(point1, Tnc) || camera.isWorldWithinFOV(point2, Tnc)) {
                    cv::Vec2d pixel1 = camera.worldToPixel(point1, Tnc);
                    cv::Vec2d pixel2 = camera.worldToPixel(point2, Tnc);
                    cv::line(image, cv::Point(pixel1), cv::Point(pixel2), color, thickness);
                }
            }
        };

        // Draw the base rectangle (4 edges on the chessboard plane)
        for (int i = 0; i < 4; ++i) {
            drawCurvedLine(baseCorners[i], baseCorners[(i + 1) % 4], cv::Scalar(0, 255, 0));
        }

        // Draw the top rectangle (4 edges at the top of the box)
        for (int i = 0; i < 4; ++i) {
            drawCurvedLine(topCorners[i], topCorners[(i + 1) % 4], cv::Scalar(255, 0, 0));
        }

        // Draw the vertical edges (4 edges connecting base to top)
        for (int i = 0; i < 4; ++i) {
            drawCurvedLine(baseCorners[i], topCorners[i], cv::Scalar(0, 0, 255));
        }
    }

    void ChessboardImage::recoverPose(const Chessboard& chessboard, const Camera& camera) {
        std::vector<cv::Point3f> rPNn_all = chessboard.gridPoints();

        cv::Mat Thetacn, rNCc;
        cv::solvePnP(rPNn_all, corners, camera.cameraMatrix, camera.distCoeffs, Thetacn, rNCc);

        Pose<double> Tcn(Thetacn, rNCc);
        Tnc = Tcn.inverse();
    }

    ChessboardData::ChessboardData(const std::filesystem::path& configPath) {
        // Ensure the config file exists
        if (!std::filesystem::exists(configPath)) {
            throw std::runtime_error("Config file does not exist: " + configPath.string());
        }

        // Open the config file
        cv::FileStorage fs(configPath.string(), cv::FileStorage::READ);
        if (!fs.isOpened()) {
            throw std::runtime_error("Failed to open config file: " + configPath.string());
        }

        // Read chessboard configuration
        cv::FileNode node = fs["chessboard_data"];
        node["chessboard"] >> chessboard;
        std::println("Chessboard: {}", to_string(chessboard));

        // Read file pattern for chessboard images
        std::string pattern;
        node["file_regex"] >> pattern;
        fs.release();

        // Create regex object from pattern
        std::regex re(pattern, std::regex_constants::basic | std::regex_constants::icase);

        // Get the directory containing the config file
        std::filesystem::path root = configPath.parent_path();
        std::println("Scanning directory {} for file pattern \"{}\"", root.string(), pattern);

        // Populate chessboard images from regex
        chessboardImages.clear();
        if (std::filesystem::exists(root) && std::filesystem::is_directory(root)) {
            // Iterate through all files in the directory and its subdirectories
            for (const auto& p : std::filesystem::recursive_directory_iterator(root)) {
                if (std::filesystem::is_regular_file(p)) {
                    // Check if the file matches the regex pattern
                    if (std::regex_match(p.path().filename().string(), re)) {
                        std::print("Loading {}...", p.path().filename().string());

                        // Try to load the file as an image
                        cv::Mat image = cv::imread(p.path().string(), cv::IMREAD_COLOR);

                        bool isImage = !image.empty();
                        if (isImage) {
                            // If it's an image, detect chessboard
                            std::print(" done, detecting chessboard...");
                            ChessboardImage ci(image, chessboard, p.path().filename());
                            std::println("{}", ci.isFound ? " found" : " not found");
                            if (ci.isFound) {
                                chessboardImages.push_back(ci);
                            }
                        }
                        else {
                            // If it's not an image, try to load it as a video
                            cv::VideoCapture cap(p.path().string());
                            bool isVideo = cap.isOpened();
                            if (isVideo) {
                                // Get number of video frames
                                int nFrames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
                                std::println(" done, found {} frames", nFrames);

                                // Choose a sampling step (approximately 1 frame per second if fps is known, else 30)
                                double fps       = cap.get(cv::CAP_PROP_FPS);
                                int frameStep    = (fps > 0.0) ? std::max(1, static_cast<int>(std::lround(fps))) : 30;

                                // Loop through selected frames
                                for (int idxFrame = 0; idxFrame < nFrames; idxFrame += frameStep) {
                                    // Read frame
                                    std::print("Reading {} frame {}...", p.path().filename().string(), idxFrame);
                                    cap.set(cv::CAP_PROP_POS_FRAMES, idxFrame);
                                    cv::Mat frame;
                                    cap >> frame;

                                    if (frame.empty()) {
                                        std::println(" end of file found");
                                        break;
                                    }

                                    // Detect chessboard in frame
                                    std::print(" done, detecting chessboard...");
                                    std::string baseName      = p.path().stem().string();
                                    std::string frameFilename = std::format("{}_{:05d}.jpg", baseName, idxFrame);
                                    ChessboardImage ci(frame, chessboard, frameFilename);
                                    std::println("{}", ci.isFound ? " found" : " not found");
                                    if (ci.isFound) {
                                        chessboardImages.push_back(ci);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    void ChessboardData::drawCorners() {
        for (auto& chessboardImage : chessboardImages) {
            chessboardImage.drawCorners(chessboard);
        }
    }

    void ChessboardData::drawBoxes(const Camera& camera) {
        for (auto& chessboardImage : chessboardImages) {
            chessboardImage.drawBox(chessboard, camera);
        }
    }

    void ChessboardData::recoverPoses(const Camera& camera) {
        for (auto& chessboardImage : chessboardImages) {
            chessboardImage.recoverPose(chessboard, camera);
        }
    }

    void Camera::calibrate(ChessboardData& chessboardData) {
        std::vector<cv::Point3f> rPNn_all = chessboardData.chessboard.gridPoints();

        std::vector<std::vector<cv::Point2f>> rQOi_all;
        for (const auto& chessboardImage : chessboardData.chessboardImages) {
            rQOi_all.push_back(chessboardImage.corners);
        }
        assert(!rQOi_all.empty());

        imageSize = chessboardData.chessboardImages[0].image.size();

        flags = cv::CALIB_RATIONAL_MODEL | cv::CALIB_THIN_PRISM_MODEL;

        // Find intrinsic and extrinsic camera parameters
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        distCoeffs   = cv::Mat::zeros(12, 1, CV_64F);
        std::vector<cv::Mat> Thetacn_all, rNCc_all;
        double rms;
        std::print("Calibrating camera...");
        // Calibrate camera from detected chessboard corners
        std::vector<std::vector<cv::Point3f>> rPNn_all_repeated(rQOi_all.size(),  // number of corners in one image
                                                                rPNn_all          // 3D chessboard points
        );

        rms = cv::calibrateCamera(rPNn_all_repeated,
                                rQOi_all,
                                imageSize,
                                cameraMatrix,
                                distCoeffs,
                                Thetacn_all,
                                rNCc_all,
                                flags);
        std::println(" done");

        // Pre-compute constants used in isVectorWithinFOV
        calcFieldOfView();

        // Write extrinsic camera parameters for each chessboard image
        assert(chessboardData.chessboardImages.size() == rNCc_all.size());
        assert(chessboardData.chessboardImages.size() == Thetacn_all.size());
        for (std::size_t k = 0; k < chessboardData.chessboardImages.size(); ++k) {
            // Set the camera orientation and position (extrinsic camera parameters)
            Pose<double>& Tnc = chessboardData.chessboardImages[k].Tnc;
            // OpenCV tells us: where is the world origin with respect to the camera in camera frame coordinates
            // We want where is the camera in the world frame in world coordinates
            Pose<double> Tcn(Thetacn_all[k], rNCc_all[k]);
            Tnc = Tcn.inverse();  // rCNn
        }

        printCalibration();
        std::println("{:>30} {}", "RMS reprojection error:", rms);

        assert(cv::checkRange(cameraMatrix));
        assert(cv::checkRange(distCoeffs));
    }

    void Camera::printCalibration() const {
        std::bitset<8 * sizeof(flags)> bitflag(flags);
        std::println("\nCalibration data:");
        std::println("{:>30} {}", "Bit flags:", bitflag.to_string());
        std::println("{:>30}\n{}", "cameraMatrix:", to_string(cameraMatrix));
        std::println("{:>30}\n{}", "distCoeffs:", to_string(distCoeffs.t()));
        std::println("{:>30} (fx, fy) = ({}, {})",
                    "Focal lengths:",
                    cameraMatrix.at<double>(0, 0),
                    cameraMatrix.at<double>(1, 1));
        std::println("{:>30} (cx, cy) = ({}, {})",
                    "Principal point:",
                    cameraMatrix.at<double>(0, 2),
                    cameraMatrix.at<double>(1, 2));
        std::println("{:>30} {} deg", "Field of view (horizontal):", 180.0 / CV_PI * hFOV);
        std::println("{:>30} {} deg", "Field of view (vertical):", 180.0 / CV_PI * vFOV);
        std::println("{:>30} {} deg", "Field of view (diagonal):", 180.0 / CV_PI * dFOV);
    }

    void Camera::calcFieldOfView() {
        assert(cameraMatrix.rows == 3);
        assert(cameraMatrix.cols == 3);
        assert(cameraMatrix.type() == CV_64F);

        int width  = imageSize.width;
        int height = imageSize.height;

        // Horizontal FOV from center to left
        cv::Vec2d center(width / 2.0, height / 2.0);
        cv::Vec2d left_pixel(0, height / 2.0);
        cv::Vec2d right_pixel(width - 1, height / 2.0);

        cv::Vec3d u_center = pixelToVector(center);
        cv::Vec3d u_left   = pixelToVector(left_pixel);
        cv::Vec3d u_right  = pixelToVector(right_pixel);

        double angle_left  = std::atan2(u_left[0], u_left[2]);
        double angle_right = std::atan2(u_right[0], u_right[2]);
        hFOV               = std::abs(angle_right - angle_left);  // total horizontal FOV

        // Vertical FOV from center to top and bottom
        cv::Vec2d top_pixel(width / 2.0, 0);
        cv::Vec2d bottom_pixel(width / 2.0, height - 1);
        cv::Vec3d u_top    = pixelToVector(top_pixel);
        cv::Vec3d u_bottom = pixelToVector(bottom_pixel);

        double angle_top    = std::atan2(u_top[1], u_top[2]);
        double angle_bottom = std::atan2(u_bottom[1], u_bottom[2]);
        vFOV                = std::abs(angle_bottom - angle_top);

        // Diagonal (for info only, same fix as above)
        cv::Vec2d top_left(0, 0);
        cv::Vec2d bottom_right(width - 1, height - 1);
        cv::Vec3d u_tl = pixelToVector(top_left);
        cv::Vec3d u_br = pixelToVector(bottom_right);

        double angle_tl = std::atan2(std::sqrt(u_tl[0] * u_tl[0] + u_tl[1] * u_tl[1]), u_tl[2]);
        double angle_br = std::atan2(std::sqrt(u_br[0] * u_br[0] + u_br[1] * u_br[1]), u_br[2]);
        dFOV            = std::abs(angle_br + angle_tl);  // total diagonal FOV
    }

    cv::Vec3d Camera::worldToVector(const cv::Vec3d& rPNn, const Pose<double>& Tnb) const {
        // Camera pose Tnc (i.e., Rnc, rCNn)
        Pose<double> Tnc = bodyToCamera(Tnb);  // Tnb*Tbc

        // Compute the unit vector uPCc from the world position rPNn and camera pose Tnc
        cv::Vec3d rPCc = Tnc.inverse() * rPNn;       // transform world point to camera frame
        cv::Vec3d uPCc = rPCc / cv::norm(rPCc);      // compute the norm: gives us a unit vector
        return uPCc;
    }

    cv::Vec2d Camera::worldToPixel(const cv::Vec3d& rPNn, const Pose<double>& Tnb) const {
        return vectorToPixel(worldToVector(rPNn, Tnb));
    }

    cv::Vec2d Camera::vectorToPixel(const cv::Vec3d& rPCc) const {
        // Compute the pixel location (rQOi) for the given vector (rPCc)
        cv::Vec2d rQOi;
        std::vector<cv::Point3f> objectPoints = {cv::Point3f(rPCc)};
        std::vector<cv::Point2f> rQOi_vec;

        cv::projectPoints(objectPoints,
                          cv::Vec3d(0, 0, 0),
                          cv::Vec3d(0, 0, 0),
                          cameraMatrix,
                          distCoeffs,
                          rQOi_vec);  // planar camera model.

        rQOi = cv::Vec2d(rQOi_vec[0].x, rQOi_vec[0].y);

        return rQOi;
    }

    Eigen::Vector2d Camera::vectorToPixel(const Eigen::Vector3d& rPCc, Eigen::Matrix23d& J) const {
        // Extract camera intrinsic parameters
        double fx = cameraMatrix.at<double>(0, 0);
        double fy = cameraMatrix.at<double>(1, 1);
        double cx = cameraMatrix.at<double>(0, 2);
        double cy = cameraMatrix.at<double>(1, 2);

        // Extract distortion coefficients (rational + thin prism model)
        double k1 = distCoeffs.at<double>(0);
        double k2 = distCoeffs.at<double>(1);
        double p1 = distCoeffs.at<double>(2);
        double p2 = distCoeffs.at<double>(3);
        double k3 = distCoeffs.at<double>(4);
        double k4 = distCoeffs.at<double>(5);
        double k5 = distCoeffs.at<double>(6);
        double k6 = distCoeffs.at<double>(7);
        double s1 = distCoeffs.at<double>(8);
        double s2 = distCoeffs.at<double>(9);
        double s3 = distCoeffs.at<double>(10);
        double s4 = distCoeffs.at<double>(11);

        // Extract coordinates from camera vector
        double x = rPCc[0];
        double y = rPCc[1];
        double z = rPCc[2];

        // Normalized coordinates (equations 2 from PDF)
        double u = x / z;
        double v = y / z;

        // Radius squared
        double r2 = u * u + v * v;
        double r4 = r2 * r2;
        double r6 = r4 * r2;

        // Rational distortion model coefficients (equations 4-5)
        double alpha = k1 * r2 + k2 * r4 + k3 * r6;
        double beta  = k4 * r2 + k5 * r4 + k6 * r6;

        // Radial distortion coefficient (equation 3)
        double c = (1 + alpha) / (1 + beta);

        // Distorted coordinates with radial, decentering, and thin prism distortion (equation 1)
        double u_prime = c * u + (2 * p1 * u * v + p2 * (r2 + 2 * u * u)) + (s1 * r2 + s2 * r4);
        double v_prime = c * v + (p1 * (r2 + 2 * v * v) + 2 * p2 * u * v) + (s3 * r2 + s4 * r4);

        // Final pixel coordinates (equation 6)
        Eigen::Vector2d rQOi;
        rQOi[0] = fx * u_prime + cx;  // u coordinate
        rQOi[1] = fy * v_prime + cy;  // v coordinate
        // First compute intermediate derivatives (equations 15-25)
        double du_dx = 1.0 / z;
        double du_dy = 0.0;
        double du_dz = -x / (z * z);

        double dv_dx = 0.0;
        double dv_dy = 1.0 / z;
        double dv_dz = -y / (z * z);

        double r     = std::sqrt(r2);
        double dr_du = (r > 0) ? u / r : 0.0;  // Handle r = 0 case
        double dr_dv = (r > 0) ? v / r : 0.0;

        double dalpha_dr = 2 * k1 * r + 4 * k2 * r * r2 + 6 * k3 * r * r4;
        double dbeta_dr  = 2 * k4 * r + 4 * k5 * r * r2 + 6 * k6 * r * r4;

        double dc_dr = (dalpha_dr * (1 + beta) - (1 + alpha) * dbeta_dr) / ((1 + beta) * (1 + beta));

        double du_prime_du =
            (dc_dr * dr_du) * u + c + 2 * p1 * v + p2 * (2 * dr_du * r + 4 * u) + 2 * s1 * dr_du * r + 4 * s2 * r * r2 * dr_du;
        double du_prime_dv = (dc_dr * dr_dv) * u + 2 * p1 * u + p2 * (2 * dr_dv * r) + 2 * s1 * dr_dv * r + 4 * s2 * r * r2 * dr_dv;

        double dv_prime_du = (dc_dr * dr_du) * v + p1 * (2 * dr_du * r) + 2 * p2 * v + 2 * s3 * dr_du * r + 4 * s4 * r * r2 * dr_du;
        double dv_prime_dv =
            (dc_dr * dr_dv) * v + c + p1 * (2 * dr_dv * r + 4 * v) + 2 * p2 * u + 2 * s3 * dr_dv * r + 4 * s4 * r * r2 * dr_dv;

        J(0, 0) = fx * (du_prime_du * du_dx + du_prime_dv * dv_dx);
        J(0, 1) = fx * (du_prime_du * du_dy + du_prime_dv * dv_dy);
        J(0, 2) = fx * (du_prime_du * du_dz + du_prime_dv * dv_dz);

        J(1, 0) = fy * (dv_prime_du * du_dx + dv_prime_dv * dv_dx);
        J(1, 1) = fy * (dv_prime_du * du_dy + dv_prime_dv * dv_dy);
        J(1, 2) = fy * (dv_prime_du * du_dz + dv_prime_dv * dv_dz);

        return rQOi;
    }

    cv::Vec3d Camera::pixelToVector(const cv::Vec2d& rQOi) const {
        // Compute unit vector (uPCc) for the given pixel location (rQOi)
        std::vector<cv::Point2f> distorted   = {cv::Point2f(rQOi[0], rQOi[1])};
        std::vector<cv::Point2f> undistorted;

        cv::undistortPoints(distorted, undistorted, cameraMatrix, distCoeffs);

        cv::Vec3d uPCc(undistorted[0].x, undistorted[0].y, 1.0);
        return uPCc / cv::norm(uPCc);
    }

    bool Camera::isVectorWithinFOV(const cv::Vec3d& rPCc) const {
        // Normalize the input vector
        cv::Vec3d uPCc = rPCc / cv::norm(rPCc);

        // The camera looks along +Z, so the central (optical) axis is (0, 0, 1)
        const cv::Vec3d optical_axis(0.0, 0.0, 1.0);

        // Compute the angle between the ray and the optical axis
        double cos_angle = uPCc.dot(optical_axis);  // = cos(theta)

        // Clamp to avoid numerical errors slightly exceeding [-1, 1]
        cos_angle = std::clamp(cos_angle, -1.0, 1.0);

        double angle = std::acos(cos_angle);  // radians

        // Use the diagonal FOV for a symmetric cone check
        return angle <= dFOV / 2.0;
    }

    bool Camera::isWorldWithinFOV(const cv::Vec3d& rPNn, const Pose<double>& Tnb) const {
        return isVectorWithinFOV(worldToVector(rPNn, Tnb));
    }

    void Camera::write(cv::FileStorage& fs) const {
        fs << "{"
           << "camera_matrix" << cameraMatrix << "distortion_coefficients" << distCoeffs << "flags" << flags
           << "imageSize" << imageSize << "}";
    }

    void Camera::read(const cv::FileNode& node) {
        node["camera_matrix"] >> cameraMatrix;
        node["distortion_coefficients"] >> distCoeffs;
        node["flags"] >> flags;
        node["imageSize"] >> imageSize;

        // Pre-compute constants used in isVectorWithinFOV
        calcFieldOfView();

        assert(cameraMatrix.cols == 3);
        assert(cameraMatrix.rows == 3);
        assert(cameraMatrix.type() == CV_64F);
        assert(distCoeffs.cols == 1);
        assert(distCoeffs.type() == CV_64F);
    }

}  // namespace utility::slam::camera
