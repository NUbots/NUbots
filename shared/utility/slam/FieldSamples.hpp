
#ifndef FIELDSAMPLES_HPP
#define FIELDSAMPLES_HPP

#include <Eigen/Core>
#include <filesystem>
#include <string>
#include <vector>

#include "camera/Pose.hpp"

namespace utility::slam {

    using utility::slam::camera::Pose;

    /**
     * @brief A single message.input.Sensors sample (IMU + torso pose estimate at capture time)
     */
    struct SensorsSample {
        double t;                       ///< capture time [s since epoch]
        Pose<double> Htw;               ///< world -> torso
        Eigen::Vector3d accelerometer;  ///< [m/s^2], torso frame
        Eigen::Vector3d gyroscope;      ///< [rad/s], torso frame
    };

    /**
     * @brief A single detected bounding box within a message.vision.BoundingBoxes sample
     */
    struct Detection {
        std::string name;  ///< YOLO class name
        double confidence;
        Eigen::Matrix<double, 3, 4> corners;  ///< unit rays in camera frame {c}; columns TL, TR, BR, BL
    };

    /**
     * @brief A single message.vision.BoundingBoxes sample (camera pose + detections at capture time)
     */
    struct VisionSample {
        double t;          ///< image capture time [s since epoch]
        int videoFrame;    ///< index into frameTimes / Left.mp4, or -1 if no matching frame
        Pose<double> Hcw;  ///< world -> camera
        std::vector<Detection> detections;
    };

    /**
     * @brief A single message.vision.FieldLines sample (field-line points as camera rays)
     */
    struct LinePointsSample {
        double t;          ///< image capture time [s since epoch]
        int videoFrame;    ///< index into frameTimes / Left.mp4, or -1
        Pose<double> Hcw;  ///< world -> camera
        Eigen::Matrix<double, 3, Eigen::Dynamic>
            rays;  ///< unit rays in camera frame {c}, one column per field-line point
    };
}  // namespace utility::slam

#endif
