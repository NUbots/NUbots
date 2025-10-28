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
#include "message/input/Sensors.hpp"
#include "message/input/VSLAM.hpp"
#include "message/output/CompressedImage.hpp"
#include "message/vision/FieldIntersections.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/slam/camera/Camera.hpp"
#include "utility/slam/camera/Pose.hpp"
#include "utility/slam/gaussian/GaussianInfo.hpp"
#include "utility/slam/measurement/MeasurementAccelerometer.hpp"
#include "utility/slam/measurement/MeasurementAnchor.hpp"
#include "utility/slam/measurement/MeasurementGyroscope.hpp"
#include "utility/slam/measurement/MeasurementSLAMPointBundle.hpp"
#include "utility/slam/system/SystemSLAMPointLandmarks.hpp"
#include "utility/vision/Vision.hpp"
#include "utility/vision/fourcc.hpp"

namespace module::vision {

    using extension::Configuration;
    using message::input::Image;
    using message::input::Sensors;
    using message::output::CompressedImage;
    using VSLAMMsg = message::input::VSLAM;
    using message::vision::FieldIntersections;
    using utility::input::FrameID;
    using utility::nusight::graph;

    using utility::slam::camera::Camera;
    using utility::slam::camera::Pose;
    using utility::slam::gaussian::GaussianInfo;
    using utility::slam::measurement::MeasurementAccelerometer;
    using utility::slam::measurement::MeasurementAnchor;
    using utility::slam::measurement::MeasurementGyroscope;
    using utility::slam::measurement::MeasurementPointBundle;
    using utility::slam::system::SystemSLAMPointLandmarks;

    VSLAM::VSLAM(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("VSLAM.yaml").then([this](const Configuration& config) {
            // Set log level
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Load configuration parameters
            cfg.enableVisualization   = config["enable_visualization"].as<bool>(false);
            cfg.cameraCalibrationPath = config["camera_calibration"].as<std::string>();

            // Load initial covariance parameters
            cfg.initialCovariance.velocity        = config["initial_covariance"]["velocity"].as<double>(0.3);
            cfg.initialCovariance.angularVelocity = config["initial_covariance"]["angular_velocity"].as<double>(0.4);
            cfg.initialCovariance.position        = config["initial_covariance"]["position"].as<double>(0.01);
            cfg.initialCovariance.orientation     = config["initial_covariance"]["orientation"].as<double>(0.01);

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
            camera_.printCalibration();
        });

        // Subscribe to Image with FieldIntersections from YOLO
        on<Trigger<Image>, With<FieldIntersections>, With<Sensors>, Single, MainThread>().then(
            [this](const Image& image, const FieldIntersections& field_intersections, const Sensors& sensors) {
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

                // Get camera transform from kinematics (world-to-camera)
                Eigen::Isometry3d Htc = Eigen::Isometry3d(sensors.Htx[FrameID::L_CAMERA]);
                Eigen::Isometry3d Hwc = sensors.Htw.inverse() * Htc;

                // Process YOLO field intersections through SLAM pipeline
                // Pass sensors for gyroscope data
                cv::Mat debug_frame = processSLAMFrame(img_rgb, timestamp, field_intersections, Hwc, sensors);

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
                        log<DEBUG>("Emitted VSLAM debug frame with field intersections, size: ",
                                   jpeg_data.size(),
                                   " bytes");
                    }
                }

                // Emit VSLAM message with camera pose and map points
                if (systemInitialized_ && system_) {
                    auto vslam = std::make_unique<VSLAMMsg>();

                    // Get camera pose from SLAM system (in OpenCV frame)
                    auto cameraDensity   = system_->cameraPositionDensity(camera_);
                    Eigen::Vector3d rCNk = cameraDensity.mean();

                    auto orientationDensity = system_->cameraOrientationEulerDensity(camera_);
                    Eigen::Vector3d Thetank = orientationDensity.mean();

                    // Convert Euler angles to rotation matrix
                    Eigen::Matrix3d Rnk = utility::slam::rpy2rot(Thetank);

                    // Build Hnk transform (OpenCV to NUbots)
                    Eigen::Isometry3d Hnk = Eigen::Isometry3d::Identity();
                    Hnk.linear()          = Rnk;
                    Hnk.translation()     = rCNk;

                    //  TRANSFORM CAMERA TO NUBOTS FRAME
                    Eigen::Isometry3d Hkc = Eigen::Isometry3d::Identity();
                    Hkc.matrix() << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;

                    // Transform pose: Hnc = Hnk * Hkc
                    Eigen::Isometry3d Hnc = Hnk * Hkc;
                    vslam->Hnc            = Hnc;

                    // Transform map points using SAME Hkc rotation
                    size_t numLandmarks = system_->numberLandmarks();
                    for (size_t i = 0; i < numLandmarks; ++i) {
                        auto landmarkDensity = system_->landmarkPositionDensity(i);
                        Eigen::Vector3d rJNn = landmarkDensity.mean();
                        vslam->map_points.push_back(rJNn);
                    }

                    // Emit the VSLAM message
                    emit(std::move(vslam));

                    log<INFO>("Emitted VSLAM pose and", numLandmarks, "map points");
                    log<INFO>("Camera position:", Hnc.translation().transpose());
                }
            });
    }

    void VSLAM::initializeSystem(const Eigen::Isometry3d& Hwc) {
        // Only initialize once
        if (systemInitialized_) {
            return;
        }

        // State vector is 12-dimensional: [vBNb, omegaBNb, rBNn, Thetanb]
        const int stateDim = 12;

        // Create initial mean state
        Eigen::VectorXd initialMean = Eigen::VectorXd::Zero(stateDim);

        // Hnk = Hnw * Hwc * Hck
        Eigen::Isometry3d Hkc = Eigen::Isometry3d::Identity();
        Hkc.matrix() << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
        Eigen::Isometry3d Hck = Hkc.inverse();
        Eigen::Isometry3d Hwn = Eigen::Isometry3d::Identity();
        Hwn.linear()          = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix()
                       * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Isometry3d Hnw = Hwn.inverse();
        Eigen::Isometry3d Hnk = Hnw * Hwc * Hck;

        initialMean(6)  = Hnk.translation().x();
        initialMean(7)  = Hnk.translation().y();
        initialMean(8)  = Hnk.translation().z();
        initialMean(9)  = utility::slam::rot2rpy(Hnk.linear()).x();
        initialMean(10) = utility::slam::rot2rpy(Hnk.linear()).y();
        initialMean(11) = utility::slam::rot2rpy(Hnk.linear()).z();
        log<INFO>("Initial mean:", initialMean.transpose());

        //   powerplant.shutdown();

        // Create initial covariance matrix from configuration
        Eigen::MatrixXd initialCov = Eigen::MatrixXd::Identity(stateDim, stateDim);

        // Velocity components (vx, vy, vz)
        initialCov.diagonal()(0) *= cfg.initialCovariance.velocity;
        initialCov.diagonal()(1) *= cfg.initialCovariance.velocity;
        initialCov.diagonal()(2) *= cfg.initialCovariance.velocity;

        // Angular velocity components (wx, wy, wz)
        initialCov.diagonal()(3) *= cfg.initialCovariance.angularVelocity;
        initialCov.diagonal()(4) *= cfg.initialCovariance.angularVelocity;
        initialCov.diagonal()(5) *= cfg.initialCovariance.angularVelocity;

        // Position components (x, y, z)
        initialCov.diagonal()(6) *= cfg.initialCovariance.position;
        initialCov.diagonal()(7) *= cfg.initialCovariance.position;
        initialCov.diagonal()(8) *= cfg.initialCovariance.position;

        // Orientation components (roll, pitch, yaw)
        initialCov.diagonal()(9) *= cfg.initialCovariance.orientation;
        initialCov.diagonal()(10) *= cfg.initialCovariance.orientation;
        initialCov.diagonal()(11) *= cfg.initialCovariance.orientation;

        // Create initial density
        auto initialDensity = GaussianInfo<double>::fromSqrtMoment(initialMean, initialCov);

        // Create system
        system_ = std::make_unique<SystemSLAMPointLandmarks>(initialDensity);
        log<INFO>("Created SystemSLAMPointLandmarks");

        systemInitialized_ = true;
        log<INFO>("SLAM system initialized successfully");
    }

    cv::Mat VSLAM::processSLAMFrame(const cv::Mat& img_rgb,
                                    double timestamp,
                                    const FieldIntersections& field_intersections,
                                    const Eigen::Isometry3d& Hwc,
                                    const Sensors& sensors) {
        // Initialize system on first frame
        if (!systemInitialized_) {
            initializeSystem(Hwc);
        }

        // Clone image for visualization
        cv::Mat debug_img = img_rgb.clone();

        Eigen::Isometry3d Hkc = Eigen::Isometry3d::Identity();
        Hkc.matrix() << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
        Eigen::Isometry3d Hck = Hkc.inverse();

        Eigen::Isometry3d Hwn = Eigen::Isometry3d::Identity();
        Hwn.linear()          = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix()
                       * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Isometry3d Hnw = Hwn.inverse();

        // Transform camera from {w} to VSLAM frame {n} with OpenCV convention
        Eigen::Isometry3d Hnk = Hnw * Hwc * Hck;

        // Extract height (matching your prediction model)
        Eigen::VectorXd height_measurement(1);
        height_measurement(0) = Hnk.translation()(2);  // + 1e-12;  // Matches h(0) = -state(8)

        log<INFO>("Height measurement:", height_measurement(0));
        auto anchor_measurement = std::make_unique<MeasurementAnchor>(timestamp, height_measurement);
        anchor_measurement->process(*system_);

        // log<INFO>("Kinematic height:", height_measurement(0), "m");
        // emit(graph("Kinematic Height", height_measurement(0)));
        // emit(graph("SLAM Height", system_->density.mean()(8)));
        log<INFO>("SLAM Height:", system_->density.mean()(8));
        // powerplant.shutdown();

        // Process accelerometer measurement
        // The accelerometer is in the torso frame, need to transform to camera frame (OpenCV convention)
        Eigen::Isometry3d Htc = Eigen::Isometry3d(sensors.Htx[FrameID::L_CAMERA]);  // Torso to camera (NUbots)
        Eigen::Matrix3d Rtc   = Htc.linear();

        // Get accelerometer measurement from torso frame (NUbots convention: x forward, y left, z up)
        Eigen::Vector3d accel_torso = sensors.accelerometer;  // a_t (IMU measurement in torso frame)

        // Transform accelerometer from torso to camera frame (NUbots convention)
        Eigen::Vector3d accel_camera_nubots = Rtc.transpose() * accel_torso;  // a_c = R_tc^T * a_t

        // Transform from camera (NUbots) to camera (OpenCV/VSLAM convention)
        // NUbots camera: x forward, y left, z up
        // OpenCV camera: z forward, x right, y down
        Eigen::Vector3d accel_camera_opencv;
        accel_camera_opencv(0) = -accel_camera_nubots(1);  // x_opencv = -y_nubots (left -> -right)
        accel_camera_opencv(1) = accel_camera_nubots(2);   // y_opencv = -z_nubots (up -> -down)
        accel_camera_opencv(2) = -accel_camera_nubots(0);  // z_opencv = x_nubots (forward)

        // Predict what the accelerometer should measure given current state (for debugging)
        auto accel_measurement_temp     = std::make_unique<MeasurementAccelerometer>(timestamp, accel_camera_opencv);
        Eigen::VectorXd predicted_accel = accel_measurement_temp->predict(system_->density.mean(), *system_);

        // Emit predicted vs measured for comparison
        emit(graph("Measured Acceleration (Camera)",
                   accel_camera_opencv.x(),
                   accel_camera_opencv.y(),
                   accel_camera_opencv.z()));
        emit(graph("Predicted Acceleration (Camera)", predicted_accel(0), predicted_accel(1), predicted_accel(2)));

        // Calculate innovation (measurement - prediction)
        Eigen::Vector3d innovation = accel_camera_opencv - predicted_accel;
        emit(graph("Acceleration Innovation", innovation.x(), innovation.y(), innovation.z()));

        // Create accelerometer measurement and process it
        auto accel_measurement = std::make_unique<MeasurementAccelerometer>(timestamp, accel_camera_opencv);
        accel_measurement->process(*system_);

        log<INFO>("Accelerometer measurement processed");

        // Loop throguh each landmark and emit graph of landmark position
        for (size_t i = 0; i < system_->numberLandmarks(); ++i) {
            auto landmarkDensity = system_->landmarkPositionDensity(i);
            Eigen::Vector3d rLNn = landmarkDensity.mean();
            emit(graph("Landmark " + std::to_string(i), rLNn(0), rLNn(1), rLNn(2)));
        }

        // Transform gyroscope from torso to camera frame
        Eigen::Vector3d gyro_torso = sensors.gyroscope;  // ω_t (IMU measurement)
        // plot gyroscope in plotjuggler
        Eigen::Vector3d gyro_camera = Rtc.transpose() * gyro_torso;  // ω_c = R_tc^T * ω_t - torso to camera frame
        // Rotate in camera frame to OpenCV convention
        Eigen::Vector3d gyro_camera_opencv = Hkc * gyro_camera;
        emit(graph("Measured Angular Velocity (OpenCV)",
                   gyro_camera_opencv.x(),
                   gyro_camera_opencv.y(),
                   gyro_camera_opencv.z()));

        // Temp measurement
        auto gyro_measurement_temp     = std::make_unique<MeasurementGyroscope>(timestamp, gyro_camera_opencv);
        Eigen::VectorXd predicted_gyro = gyro_measurement_temp->predict(system_->density.mean(), *system_);
        emit(graph("Predicted Angular Velocity", predicted_gyro(0), predicted_gyro(1), predicted_gyro(2)));

        // Create gyroscope measurement and process it
        auto gyro_measurement = std::make_unique<MeasurementGyroscope>(timestamp, gyro_camera);
        // gyro_measurement->process(*system_);
        // Emit estimated roll, pitch, yaw in plotjuggler
        emit(graph("Estimated Roll", system_->density.mean()(9)));
        emit(graph("Estimated Pitch", system_->density.mean()(10)));
        emit(graph("Estimated Yaw", system_->density.mean()(11)));

        // Check if we have field intersections
        if (field_intersections.intersections.empty()) {
            log<INFO>("No field intersections detected in frame");
            return debug_img;
        }

        const int numIntersections = field_intersections.intersections.size();
        log<INFO>("Processing", numIntersections, "field intersections from YOLO");

        // Create measurement matrix from field intersection pixel centres
        Eigen::Matrix<double, 2, Eigen::Dynamic> Y(2, numIntersections);

        for (int i = 0; i < numIntersections; ++i) {
            const auto& intersection = field_intersections.intersections[i];

            // Extract pixel centre coordinates (computed by YOLO)
            Y(0, i) = intersection.pixel_centre.x();  // u coordinate
            Y(1, i) = intersection.pixel_centre.y();  // v coordinate

            // Draw on debug image for visualization
            cv::Point2f centre(intersection.pixel_centre.x(), intersection.pixel_centre.y());

            // Choose color and label based on intersection type
            cv::Scalar color;
            std::string label;
            switch (static_cast<int>(intersection.type)) {
                case static_cast<int>(message::vision::FieldIntersection::IntersectionType::L_INTERSECTION):
                    color = cv::Scalar(0, 0, 255);  // Red (BGR format)
                    label = "L";
                    break;
                case static_cast<int>(message::vision::FieldIntersection::IntersectionType::T_INTERSECTION):
                    color = cv::Scalar(0, 255, 0);  // Green
                    label = "T";
                    break;
                case static_cast<int>(message::vision::FieldIntersection::IntersectionType::X_INTERSECTION):
                    color = cv::Scalar(255, 0, 0);  // Blue
                    label = "X";
                    break;
                default:
                    color = cv::Scalar(255, 255, 255);  // White
                    label = "?";
                    break;
            }

            // Draw circle at intersection centre
            cv::circle(debug_img, centre, 6, color, 2);
            // Draw label
            cv::putText(debug_img, label, centre + cv::Point2f(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
        }

        // Create and process measurement using MeasurementPointBundle
        auto measurement = std::make_unique<MeasurementPointBundle>(timestamp, Y, camera_);
        measurement->process(*system_);
        // emit in plotjuggler our estimated angular velocity
        emit(graph("Estimated Angular Velocity",
                   system_->density.mean()(3),
                   system_->density.mean()(4),
                   system_->density.mean()(5)));

        // Keep measurement pointer for visualization
        MeasurementPointBundle* measurement_ptr = measurement.get();

        // Get camera pose from SLAM
        if (system_ && measurement_ptr) {
            auto cameraDensity   = system_->cameraPositionDensity(camera_);
            Eigen::Vector3d rCNn = cameraDensity.mean();  // Camera position in SLAM world

            auto orientationDensity = system_->cameraOrientationEulerDensity(camera_);
            Eigen::Vector3d Thetanc = orientationDensity.mean();  // Camera orientation

            // Build camera transform
            Eigen::Matrix3d Rnc   = utility::slam::rpy2rot(Thetanc);
            Eigen::Isometry3d Hnc = Eigen::Isometry3d::Identity();
            Hnc.linear()          = Rnc;
            Hnc.translation()     = rCNn;

            // Get world to camera transform
            Eigen::Isometry3d Hcn = Hnc.inverse();

            size_t numLandmarks = system_->numberLandmarks();
            int numVisible      = 0;

            for (size_t i = 0; i < numLandmarks; ++i) {
                // Get landmark position in SLAM world frame
                auto landmarkDensity = system_->landmarkPositionDensity(i);
                Eigen::Vector3d rLNn = landmarkDensity.mean();

                // Transform to camera frame
                Eigen::Vector3d rLCc = Hcn * rLNn;

                // Only project landmarks in front of camera
                if (rLCc(2) <= 0.1) {  // Behind camera or too close
                    continue;
                }

                // Project to pixel coordinates
                Eigen::Vector2d pixel = camera_.vectorToPixel(rLCc);

                // Check if pixel is within image bounds
                if (pixel(0) < 0 || pixel(0) >= camera_.imageSize.width || pixel(1) < 0
                    || pixel(1) >= camera_.imageSize.height) {
                    continue;
                }

                numVisible++;

                // Use measurement to predict feature density in pixel space
                // This automatically handles all the Jacobian computation and uncertainty propagation
                GaussianInfo<double> prQOi = measurement_ptr->predictFeatureDensity(*system_, i);
                Eigen::Vector2d center     = prQOi.mean();

                // Plot confidence ellipse (cyan, 3-sigma)
                cv::Scalar cyan(255, 255, 0);  // BGR
                Eigen::MatrixXd ellipse = prQOi.confidenceEllipse(3, 100);

                for (int j = 0; j < ellipse.cols() - 1; ++j) {
                    Eigen::VectorXd p1 = ellipse.col(j);
                    Eigen::VectorXd p2 = ellipse.col(j + 1);

                    bool inBounds1 =
                        (0 <= p1(0) && p1(0) <= debug_img.cols - 1 && 0 <= p1(1) && p1(1) <= debug_img.rows - 1);
                    bool inBounds2 =
                        (0 <= p2(0) && p2(0) <= debug_img.cols - 1 && 0 <= p2(1) && p2(1) <= debug_img.rows - 1);

                    if (inBounds1 && inBounds2) {
                        cv::line(debug_img, cv::Point(p1(0), p1(1)), cv::Point(p2(0), p2(1)), cyan, 2);
                    }
                }

                // Draw X marker at mean
                cv::drawMarker(debug_img, cv::Point(center(0), center(1)), cyan, cv::MARKER_CROSS, 12, 2);

                // Draw landmark ID
                cv::putText(debug_img,
                            std::to_string(i),
                            cv::Point(center(0) + 8, center(1) - 8),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.4,
                            cyan,
                            1);
            }

            // Draw legend in top-left corner
            int y_offset = 30;
            cv::putText(debug_img,
                        "YOLO Detections: Circles (L=Red, T=Green, X=Blue)",
                        cv::Point(10, y_offset),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(255, 255, 255),
                        1);
            y_offset += 25;

            cv::putText(debug_img,
                        "SLAM Landmarks: Cyan X + 3-sigma ellipses",
                        cv::Point(10, y_offset),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(255, 255, 0),
                        1);
            y_offset += 25;

            // Draw statistics
            std::string stats = "YOLO: " + std::to_string(numIntersections) + " | SLAM: " + std::to_string(numLandmarks)
                                + " (" + std::to_string(numVisible) + " visible)";
            cv::putText(debug_img,
                        stats,
                        cv::Point(10, y_offset),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(255, 255, 255),
                        1);

            log<INFO>("Visualization: ",
                      numIntersections,
                      " YOLO detections, ",
                      numLandmarks,
                      " SLAM landmarks (",
                      numVisible,
                      " visible)");
        }

        return debug_img;
    }

}  // namespace module::vision
