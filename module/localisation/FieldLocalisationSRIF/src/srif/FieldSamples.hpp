/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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

#ifndef MODULE_LOCALISATION_SRIF_FIELDSAMPLES_HPP
#define MODULE_LOCALISATION_SRIF_FIELDSAMPLES_HPP

#include <Eigen/Core>
#include <filesystem>
#include <string>
#include <vector>

#include "utility/gaussian_filtering/Pose.hpp"

namespace module::localisation::srif {

    using utility::gaussian_filtering::Pose;

    /// @brief A single message.input.Sensors sample
    struct SensorsSample {
        double t;                       ///< capture time [s since epoch]
        Pose<double> Htw;               ///< world -> torso
        Eigen::Vector3d accelerometer;  ///< [m/s^2], torso frame
        Eigen::Vector3d gyroscope;      ///< [rad/s], torso frame
        /// @brief Body-fixed linear velocity [m/s], torso frame, from
        ///        Config::odometry_velocity_source. Non-finite when there is none.
        Eigen::Vector3d vBb;
    };

    /// @brief A single detected bounding box within a message.vision.BoundingBoxes sample
    struct Detection {
        std::string name;  ///< YOLO class name
        double confidence;
        Eigen::Matrix<double, 3, 4> corners;  ///< unit rays in camera frame {c}; columns TL, TR, BR, BL
    };

    /// @brief A single message.vision.BoundingBoxes sample
    struct VisionSample {
        double t;          ///< image capture time [s since epoch]
        int video_frame;   ///< index of the matching video frame, or -1
        Pose<double> Hcw;  ///< world -> camera
        std::vector<Detection> detections;
    };

    /// @brief A single message.vision.FieldLines sample (field-line points as camera rays)
    struct LinePointsSample {
        double t;          ///< image capture time [s since epoch]
        int video_frame;   ///< index of the matching video frame, or -1
        Pose<double> Hcw;  ///< world -> camera
        Eigen::Matrix<double, 3, Eigen::Dynamic>
            rays;  ///< unit rays in camera frame {c}, one column per field-line point
    };
}  // namespace module::localisation::srif

#endif  // MODULE_LOCALISATION_SRIF_FIELDSAMPLES_HPP
