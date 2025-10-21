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

#ifndef MODULE_VISION_VSLAM_HPP
#define MODULE_VISION_VSLAM_HPP

#include <memory>
#include <nuclear>
#include <string>

#include "utility/slam/camera/Camera.hpp"
#include "utility/slam/rotation.hpp"
#include "utility/slam/system/SystemSLAM.hpp"

namespace module::vision {

    /**
     * @brief Visual SLAM reactor for state estimation using camera input
     *
     * This reactor implements visual SLAM (Simultaneous Localization and Mapping) using
     * camera images to estimate the robot's pose and map landmarks in the environment.
     * It supports multiple landmark types:
     * - Scenario 1: ArUco marker pose landmarks (6-DOF landmarks)
     * - Scenario 2: Duck detection with area (point landmarks)
     * - Scenario 3: Generic point features (corner detection)
     */
    class VSLAM : public NUClear::Reactor {
    private:
        /// @brief Configuration structure
        struct Config {
            /// Scenario type (1: ArUco tags, 2: Ducks, 3: Point features)
            int scenario = 3;

            /// Enable VTK visualization (expensive, disable for performance)
            bool enableVisualization = false;

            /// Path to camera calibration file
            std::string cameraCalibrationPath = "";

            /// Initial camera height above ground plane (meters)
            double initialCameraHeight = 0.58;

            /// Initial covariance scaling factors
            struct InitialCovariance {
                double velocity        = 0.3;
                double angularVelocity = 0.4;
                double position        = 0.01;
                double orientation     = 0.01;
            } initialCovariance;
            struct Features {
                int maxFeatures = 100;  // Maximum features to detect per frame
            } features;

        } cfg;

        /// Camera object containing calibration and intrinsic parameters
        utility::slam::camera::Camera camera_;

        /// SLAM system (polymorphic pointer to support different landmark types)
        std::unique_ptr<utility::slam::system::SystemSLAM> system_;

        /// Flag indicating whether the system has been initialized
        bool systemInitialized_ = false;

        /// Current time for the SLAM system
        double currentTime_ = 0.0;

        /// Optional: VTK plot for visualization (if enabled)
        // std::unique_ptr<Plot> plot_;

        /**
         * @brief Initialize the SLAM system with appropriate landmark type
         *
         * Creates the initial state density and constructs the system based on
         * the configured scenario (pose landmarks vs point landmarks).
         */
        void initializeSystem();

        /**
         * @brief Process an image frame through the SLAM pipeline
         *
         * Steps:
         * 1. Initialize system on first frame
         * 2. Predict system state forward in time
         * 3. Detect features based on scenario
         * 4. Create measurement from detections
         * 5. Update system state with measurement
         * 6. Optional: visualize results
         *
         * @param img_rgb The input image frame
         * @param timestamp Timestamp of the frame in seconds
         * @return cv::Mat Debug image with detected features drawn (empty if no features detected)
         */
        cv::Mat processSLAMFrame(const cv::Mat& img_rgb, double timestamp);

        /**
         * @brief Scenario 3: Process generic point features
         *
         * Detects corner features (Shi-Tomasi) and creates point landmark measurements.
         *
         * @param frame The input image frame
         * @param timestamp Timestamp of the frame in seconds
         * @return cv::Mat Debug image with detected features drawn (empty if no features detected)
         */
        cv::Mat processPointFeatureScenario(const cv::Mat& img_rgb, double timestamp);

    public:
        /**
         * @brief Called by the powerplant to build and setup the VSLAM reactor.
         * @param environment The NUClear environment
         */
        explicit VSLAM(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::vision

#endif  // MODULE_VISION_VSLAM_HPP
