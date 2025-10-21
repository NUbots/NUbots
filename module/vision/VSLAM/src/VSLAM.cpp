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

#include "VSLAM.hpp"

#include <filesystem>
#include <opencv2/core/persistence.hpp>
#include <opencv2/opencv.hpp>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/input/VSLAM.hpp"
#include "message/output/CompressedImage.hpp"

#include "utility/slam/camera/Camera.hpp"
#include "utility/slam/camera/Pose.hpp"
#include "utility/slam/gaussian/GaussianInfo.hpp"
#include "utility/slam/measurement/MeasurementSLAMPointBundle.hpp"
#include "utility/slam/system/SystemSLAMPointLandmarks.hpp"
#include "utility/slam/vision/imagefeatures.hpp"
#include "utility/vision/Vision.hpp"
#include "utility/vision/fourcc.hpp"

namespace module::vision {

    using extension::Configuration;
    using message::input::Image;
    using message::output::CompressedImage;
    using VSLAMMsg = message::input::VSLAM;

    using utility::slam::camera::Camera;
    using utility::slam::camera::Pose;
    using utility::slam::gaussian::GaussianInfo;
    using utility::slam::measurement::MeasurementPointBundle;
    using utility::slam::system::SystemSLAMPointLandmarks;
    using utility::slam::vision::detectAndDrawShiAndTomasi;

    VSLAM::VSLAM(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("VSLAM.yaml").then([this](const Configuration& config) {
            // Set log level
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Load configuration parameters
            cfg.enableVisualization   = config["enable_visualization"].as<bool>(false);
            cfg.cameraCalibrationPath = config["camera_calibration"].as<std::string>();
            cfg.initialCameraHeight   = config["initial_camera_height"].as<double>(0.58);

            // Load initial covariance parameters
            cfg.initialCovariance.velocity        = config["initial_covariance"]["velocity"].as<double>(0.3);
            cfg.initialCovariance.angularVelocity = config["initial_covariance"]["angular_velocity"].as<double>(0.4);
            cfg.initialCovariance.position        = config["initial_covariance"]["position"].as<double>(0.01);
            cfg.initialCovariance.orientation     = config["initial_covariance"]["orientation"].as<double>(0.01);

            // Max Features
            cfg.features.maxFeatures = config["features"]["max_features"].as<int>(100);

            // Load camera calibration
            if (!std::filesystem::exists(cfg.cameraCalibrationPath)) {
                log<INFO>("Camera calibration file not found:", cfg.cameraCalibrationPath);
                return;
            }

            cv::FileStorage fs(cfg.cameraCalibrationPath, cv::FileStorage::READ);
            if (!fs.isOpened()) {
                log<INFO>("Failed to open camera calibration file:", cfg.cameraCalibrationPath);
                return;
            }

            fs["camera"] >> camera_;
            fs.release();
            // Align frame of body to camera frame
            // b1 = c3, b2 = c1, b3 = c2
            Eigen::Matrix3d Rbc;
            Rbc << 1, 0, 0,  // b1 = c3
                0, 1, 0,     // b2 = c1
                0, 0, 1;     // b3 = c2

            camera_.Tbc.rotationMatrix    = Rbc;
            camera_.Tbc.translationVector = Eigen::Vector3d::Zero();
            log<INFO>("Camera calibration loaded successfully", camera_.Tbc.rotationMatrix);
            camera_.printCalibration();
        });

        on<Trigger<Image>, Single, MainThread>().then([this](const Image& image) {
            // Convert NUClear Image to cv::Mat based on image format
            int width  = image.dimensions.x();
            int height = image.dimensions.y();

            camera_.imageSize = cv::Size(width, height);

            cv::Mat img_cv;
            cv::Mat img_rgb;
            switch (image.format) {
                case utility::vision::fourcc("BGR3"):
                    img_cv  = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(image.data.data()));
                    img_rgb = img_cv.clone();
                    break;
                case utility::vision::fourcc("RGBA"):
                    img_cv = cv::Mat(height, width, CV_8UC4, const_cast<uint8_t*>(image.data.data()));
                    cv::cvtColor(img_cv, img_rgb, cv::COLOR_RGBA2RGB);
                    break;
                case utility::vision::FOURCC::RGGB:
                    img_cv = cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(image.data.data()));
                    cv::cvtColor(img_cv, img_rgb, cv::COLOR_BayerRG2RGB);
                    break;
                default: log<WARN>("Unsupported image format: ", utility::vision::fourcc(image.format)); return;
            }

            if (img_rgb.empty()) {
                log<INFO>("Received empty image frame");
                return;
            }

            // Calculate timestamp from NUClear clock
            double timestamp = NUClear::clock::now().time_since_epoch().count() / 1e9;

            // Process the frame through SLAM pipeline and get debug image with features drawn
            cv::Mat debug_frame = processSLAMFrame(img_rgb, timestamp);

            // Emit debug frame as CompressedImage for visualization in NUsight
            if (!debug_frame.empty() && cfg.enableVisualization) {
                // Compress to JPEG using OpenCV
                std::vector<uint8_t> jpeg_data;
                std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 85};  // 85% quality
                cv::imencode(".jpg", debug_frame, jpeg_data, compression_params);

                // Create CompressedImage message
                auto debug_image = std::make_unique<CompressedImage>();

                // Set basic fields
                debug_image->format            = utility::vision::fourcc("JPEG");
                debug_image->dimensions        = Eigen::Vector2<unsigned int>(debug_frame.cols, debug_frame.rows);
                debug_image->id                = 2;  // Different ID from main camera
                debug_image->name              = "vslam_debug_frame";
                debug_image->timestamp         = image.timestamp;
                debug_image->Hcw               = image.Hcw;
                debug_image->lens.projection   = static_cast<int>(image.lens.projection);
                debug_image->lens.focal_length = image.lens.focal_length;
                debug_image->lens.fov          = image.lens.fov;
                debug_image->lens.centre       = image.lens.centre;
                debug_image->lens.k            = image.lens.k;

                // Copy compressed JPEG data
                debug_image->data = jpeg_data;

                // Emit the compressed debug image
                emit(std::move(debug_image));

                if (log_level <= DEBUG) {
                    log<DEBUG>("Emitted VSLAM debug frame with feature detections, size: ", jpeg_data.size(), " bytes");
                }
            }

            // emit vslam message
            if (systemInitialized_ && system_) {
                auto vslam = std::make_unique<VSLAMMsg>();

                // Get camera pose from SLAM system (in OpenCV conventions)
                // SLAM uses OpenCV camera frame {k} internally (from cv::undistortPoints)
                auto cameraDensity   = system_->cameraPositionDensity(camera_);
                Eigen::Vector3d rCNk = cameraDensity.mean();  // Camera position in OpenCV frame

                auto orientationDensity = system_->cameraOrientationEulerDensity(camera_);
                Eigen::Vector3d Thetank = orientationDensity.mean();  // Camera orientation in OpenCV

                // Convert Euler angles to rotation matrix
                Eigen::Matrix3d Rnk = utility::slam::rpy2rot(Thetank);

                // Build Hnk transform (world {n} to OpenCV camera {k})
                Eigen::Isometry3d Hnk = Eigen::Isometry3d::Identity();
                Hnk.linear()          = Rnk;
                Hnk.translation()     = rCNk;

                // Transform from OpenCV camera {k} to NUbots camera {c}
                // Same as Stella's Hkc
                Eigen::Isometry3d Hkc = Eigen::Isometry3d::Identity();
                Hkc.matrix() << 0, -1,  0, 0,
                                0,  0, -1, 0,
                                1,  0,  0, 0,
                                0,  0,  0, 1;

                // Compose: Hnc = Hnk * Hkc
                Eigen::Isometry3d Hnc = Hnk * Hkc;

                vslam->Hnc = Hnc;

                // Get map points (landmarks) from SLAM system
                size_t numLandmarks = system_->numberLandmarks();
                for (size_t i = 0; i < numLandmarks; ++i) {
                    auto landmarkDensity = system_->landmarkPositionDensity(i);
                    Eigen::Vector3d rJNn = landmarkDensity.mean();  // Landmark J position in VSLAM world frame {n}
                    vslam->map_points.push_back(rJNn);
                }

                // Emit the VSLAM message
                emit(std::move(vslam));

                log<INFO>("Emitted VSLAM pose and",
                          numLandmarks,
                          "map points. Position: [",
                          rCNk.x(),
                          ",",
                          rCNk.y(),
                          ",",
                          rCNk.z(),
                          "]");
            }
        });
    }

    void VSLAM::initializeSystem() {
        // Only initialize once
        if (systemInitialized_) {
            return;
        }

        // State vector is 12-dimensional: [vBNb, omegaBNb, rBNn, Thetanb]
        // Landmarks will be added dynamically as they are detected
        const int stateDim = 12;

        // Create initial mean state
        Eigen::VectorXd initialMean = Eigen::VectorXd::Zero(stateDim);
        initialMean(8)              = cfg.initialCameraHeight;  // Initial camera height above ground

        // Create initial covariance matrix from configuration
        Eigen::MatrixXd initialCov = Eigen::MatrixXd::Identity(stateDim, stateDim);

        // Velocity components (vx, vy, vz)
        initialCov.diagonal()(0) *= cfg.initialCovariance.velocity;  // vx
        initialCov.diagonal()(1) *= cfg.initialCovariance.velocity;  // vy
        initialCov.diagonal()(2) *= cfg.initialCovariance.velocity;  // vz

        // Angular velocity components (wx, wy, wz)
        initialCov.diagonal()(3) *= cfg.initialCovariance.angularVelocity;  // wx
        initialCov.diagonal()(4) *= cfg.initialCovariance.angularVelocity;  // wy
        initialCov.diagonal()(5) *= cfg.initialCovariance.angularVelocity;  // wz

        // Position components (x, y, z)
        initialCov.diagonal()(6) *= cfg.initialCovariance.position;  // x
        initialCov.diagonal()(7) *= cfg.initialCovariance.position;  // y
        initialCov.diagonal()(8) *= cfg.initialCovariance.position;  // z

        // Orientation components (roll, pitch, yaw)
        initialCov.diagonal()(9) *= cfg.initialCovariance.orientation;   // roll
        initialCov.diagonal()(10) *= cfg.initialCovariance.orientation;  // pitch
        initialCov.diagonal()(11) *= cfg.initialCovariance.orientation;  // yaw

        // Create initial density
        auto initialDensity = GaussianInfo<double>::fromMoment(initialMean, initialCov);

        // Create system
        system_ = std::make_unique<SystemSLAMPointLandmarks>(initialDensity);
        log<INFO>("Created SystemSLAMPointLandmarks");

        systemInitialized_ = true;
        log<INFO>("SLAM system initialized successfully");
    }

    cv::Mat VSLAM::processSLAMFrame(const cv::Mat& img_rgb, double timestamp) {
        // Initialize system on first frame
        if (!systemInitialized_) {
            initializeSystem();
        }

        return processPointFeatureScenario(img_rgb, timestamp);
    }

    cv::Mat VSLAM::processPointFeatureScenario(const cv::Mat& img_rgb, double timestamp) {
        // Detect Shi-Tomasi corner features
        auto result = detectAndDrawShiAndTomasi(img_rgb, cfg.features.maxFeatures);

        if (result.points.empty()) {
            log<INFO>("No point features detected in frame");
            return cv::Mat();  // Return empty mat
        }
        log<INFO>("Detected", result.points.size(), "point features");
        // Create measurement matrix from detected points
        const int numPoints = result.points.size();
        Eigen::Matrix<double, 2, Eigen::Dynamic> Y(2, numPoints);

        for (int i = 0; i < numPoints; ++i) {
            Y(0, i) = result.points[i].x;  // x coordinate
            Y(1, i) = result.points[i].y;  // y coordinate
        }

        // Create and process measurement
        auto measurement = std::make_unique<MeasurementPointBundle>(timestamp, Y, camera_);
        measurement->process(*system_);

        log<INFO>("Processed point feature measurement");

        // Return the debug image with features drawn
        return result.image;
    }

}  // namespace module::vision
