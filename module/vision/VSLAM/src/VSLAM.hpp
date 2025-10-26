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

#include <Eigen/Geometry>

#include "message/input/Sensors.hpp"
#include "message/vision/FieldIntersections.hpp"
#include "utility/slam/camera/Camera.hpp"
#include "utility/slam/rotation.hpp"
#include "utility/slam/system/SystemSLAM.hpp"

 namespace module::vision {

     /**
      * @brief Visual SLAM reactor for state estimation using YOLO field intersections
      *
      * This reactor implements visual SLAM using YOLO-detected field intersections
      * (L, T, X intersections) as point landmarks for localization and mapping.
      * It integrates both visual measurements and gyroscope measurements for improved
      * orientation estimation.
      */
     class VSLAM : public NUClear::Reactor {
     private:
         /// @brief Configuration structure
         struct Config {
             /// Enable debug frame visualization (expensive, disable for performance)
             bool enableVisualization = false;

             /// Path to camera calibration file
             std::string cameraCalibrationPath = "";

             /// Initial covariance scaling factors
             struct InitialCovariance {
                 double velocity        = 0.3;
                 double angularVelocity = 0.4;
                 double position        = 0.01;
                 double orientation     = 0.01;
             } initialCovariance;

         } cfg;

         /// Camera object containing calibration and intrinsic parameters
         utility::slam::camera::Camera camera_;

         /// SLAM system for point landmarks
         std::unique_ptr<utility::slam::system::SystemSLAM> system_;

         /// Flag indicating whether the system has been initialized
         bool systemInitialized_ = false;

        /**
         * @brief Initialize the SLAM system aligned with the world frame
         *
         * Creates the initial state density with pose from kinematics, aligning the SLAM
         * coordinate frame with the world frame so that ground plane is at Z=0.
         *
         * @param Hwc World-to-camera transform from kinematics
         */
        void initializeSystem(const Eigen::Isometry3d& Hwc);

         /**
          * @brief Process an image frame with field intersections through the SLAM pipeline
          *
          * Steps:
          * 1. Initialize system on first frame (aligned with world frame)
          * 2. Process gyroscope measurement (transformed to SLAM frame)
          * 3. Extract pixel centres from field intersections
          * 4. Create MeasurementPointBundle measurement
          * 5. Update system state with measurement
          * 6. Return debug visualization image
          *
          * @param img_rgb The input image frame
          * @param timestamp Timestamp of the frame in seconds
          * @param field_intersections Field intersections detected by YOLO
          * @param Hwc World-to-camera transform from kinematics
          * @param sensors Sensor data including gyroscope measurements
          * @return cv::Mat Debug image with field intersections drawn
          */
         cv::Mat processSLAMFrame(const cv::Mat& img_rgb,
                                  double timestamp,
                                  const message::vision::FieldIntersections& field_intersections,
                                  const Eigen::Isometry3d& Hwc,
                                  const message::input::Sensors& sensors);

     public:
         /**
          * @brief Called by the powerplant to build and setup the VSLAM reactor.
          * @param environment The NUClear environment
          */
         explicit VSLAM(std::unique_ptr<NUClear::Environment> environment);
     };

 }  // namespace module::vision

 #endif  // MODULE_VISION_VSLAM_HPP
