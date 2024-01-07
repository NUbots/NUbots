#include "Yolo.hpp"

#include <chrono>
#include <getopt.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/Robot.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/projection.hpp"

namespace module::vision {

    using extension::Configuration;

    using message::input::Image;
    using message::vision::Ball;
    using message::vision::Balls;
    using message::vision::Robot;
    using message::vision::Robots;


    using utility::math::coordinates::cartesianToReciprocalSpherical;
    using utility::support::Expression;
    using utility::vision::unproject;

    Eigen::Vector2d correct_distortion(const Eigen::Vector2d& pixel, const Image& img) {
        double k1 = img.lens.k.x();
        double k2 = img.lens.k.y();

        // Shift point by centre offset
        double x = pixel.x() - img.lens.centre.x();
        double y = pixel.y() - img.lens.centre.y();

        // Calculate r^2
        double r2 = x * x + y * y;

        // Apply distortion k1 and k2
        double x_distorted = x * (1 + k1 * r2 + k2 * r2 * r2);
        double y_distorted = y * (1 + k1 * r2 + k2 * r2 * r2);

        return Eigen::Vector2d(x_distorted + img.lens.centre.x(), y_distorted + img.lens.centre.y());
    }

    /**
     * @brief Unprojects a pixel coordinate into a unit vector working out which lens model to use via the lens
     * parameters.
     *
     * @details
     *  This function expects a pixel coordinate having (0,0) at the top left of the image, with x to the right and y
     * down. It will then convert this into a unit vector in camera space. For this camera space is defined as a
     * coordinate system with the x axis going down the viewing direction of the camera, y is to the left of the image,
     * and z is up.
     *
     *
     * @param pixel the pixel coordinate to unproject
     * @param img the image that the pixel coordinate is from
     *
     * @return Unit vector that this pixel represents in camera {c} space
     */
    Eigen::Vector3d unproject(const Eigen::Vector2d& pixel, const Image& img) {
        // Correct for distortion
        Eigen::Vector2d corrected_pixel = correct_distortion(pixel, img);

        // Convert pixel to normalized device coordinates (NDC), between [-1, 1]
        double x_ndc = (2.0 * (corrected_pixel.x()) / img.dimensions.x()) - 1.0;
        double y_ndc = 1.0 - (2.0 * (corrected_pixel.y()) / img.dimensions.y());

        // Compute aspect ratio
        const double width  = img.dimensions.x();
        const double height = img.dimensions.y();
        double aspect_ratio = height / width;

        // Calculate unit vector in camera {c} space
        Eigen::Vector3d uRCc(1.0,
                             -x_ndc * std::tan(0.5 * img.lens.fov),
                             y_ndc * std::tan(0.5 * img.lens.fov) * aspect_ratio);
        uRCc.normalize();

        return uRCc;
    }

    Yolo::Yolo(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Yolo.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Yolo.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.model_path  = config["model_path"].as<std::string>();
        });

        on<Startup>().then("Load Yolo Model",
                           [this] { inf = Inference(cfg.model_path, cv::Size(640, 640), "classes.txt", false); });

        on<Trigger<Image>, Single>().then([this](const Image& img) {
            const Eigen::Isometry3d& Hwc = img.Hcw.inverse();

            // -------- Convert Image to cv::Mat --------
            const int width  = img.dimensions.x();
            const int height = img.dimensions.y();
            cv::Mat frame    = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(img.data.data()));


            // -------- Run Inference --------
            auto start                    = std::chrono::high_resolution_clock::now();
            std::vector<Detection> output = inf.runInference(frame);
            auto stop                     = std::chrono::high_resolution_clock::now();
            auto duration                 = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            int detections                = output.size();
            log<NUClear::DEBUG>("Inference took: ", duration.count(), "ms");
            log<NUClear::DEBUG>("Number of detections: ", detections);

            // -------- Emit Detections --------

            // Balls
            auto balls       = std::make_unique<Balls>();
            balls->id        = img.id;         // camera id
            balls->timestamp = img.timestamp;  // time when the image was taken
            balls->Hcw       = img.Hcw;        // world to camera transform at the time the image was taken

            // Robots
            auto robots       = std::make_unique<Robots>();
            robots->timestamp = img.timestamp;
            robots->id        = img.id;
            robots->Hcw       = img.Hcw;

            for (int i = 0; i < detections; ++i) {
                Detection detection = output[i];

                if (log_level <= NUClear::DEBUG) {
                    cv::Rect box     = detection.box;
                    cv::Scalar color = detection.color;

                    // Detection box with thinner line
                    cv::rectangle(frame, box, color, 1);  // Thickness reduced to 1

                    // Detection box text with smaller font
                    std::string classString =
                        detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
                    cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 0.5, 1, 0);
                    cv::Rect textBox(box.x, box.y - 20, textSize.width + 10, textSize.height + 10);  // Adjusted size

                    cv::rectangle(frame, textBox, color, cv::FILLED);
                    cv::putText(frame,
                                classString,
                                cv::Point(box.x + 5, box.y - 5),
                                cv::FONT_HERSHEY_DUPLEX,
                                0.5,
                                cv::Scalar(0, 0, 0),
                                1,
                                0);
                }

                if (detection.className == "ball") {
                    // Calculate the middle of the bottom border of the detection box
                    Eigen::Matrix<double, 2, 1> box_centre(detection.box.x + detection.box.width / 2.0,
                                                           detection.box.y + detection.box.height / 2.0);
                    // Calculate the bottom left corner of the detection box
                    Eigen::Matrix<double, 2, 1> box_left(detection.box.x, detection.box.y + detection.box.height);

                    // Correct for lens distortion
                    // box_centre = undistort_point(box_centre, img);
                    // Convert to unit vector in camera space
                    Eigen::Matrix<double, 3, 1> uBCc = unproject(box_centre, img);
                    Eigen::Matrix<double, 3, 1> uLCc = unproject(box_left, img);

                    // Project the unit vector onto the ground plane
                    Eigen::Vector3d uBCw = Hwc.rotation() * uBCc;
                    Eigen::Vector3d rPWw = uBCw * std::abs(Hwc.translation().z() / uBCw.z()) + Hwc.translation();
                    Eigen::Vector3d rBCc = Hwc.inverse() * rPWw;

                    // Add ball to balls message
                    Ball b;
                    b.uBCc = rBCc.normalized();
                    b.measurements.emplace_back();
                    b.measurements.back().type = Ball::MeasurementType::PROJECTION;
                    b.measurements.back().rBCc = rBCc;
                    // Calculate the angular radius of the ball in camera space
                    b.radius = uBCc.dot(uLCc);
                    b.colour.fill(1.0);
                    balls->balls.push_back(b);
                }

                if (detection.className == "robot") {
                    // Calculate the bottom of box centre
                    Eigen::Matrix<double, 2, 1> box_bottom_centre(detection.box.x + detection.box.width / 2.0f,
                                                                  detection.box.y + detection.box.height);

                    // Correct for lens distortion
                    // box_bottom_centre = undistort_point(box_bottom_centre, img);
                    // Convert to unit vector in camera space
                    Eigen::Matrix<double, 3, 1> uRCc = unproject(box_bottom_centre, img);

                    // Project the unit vector onto the ground plane
                    Eigen::Vector3d uRCw = Hwc.rotation() * uRCc;
                    Eigen::Vector3d rRWw = uRCw * std::abs(Hwc.translation().z() / uRCw.z()) + Hwc.translation();
                    Eigen::Vector3d rRCc = Hwc.inverse() * rRWw;

                    // Add robot to robots message
                    Robot r;
                    r.rRCc   = rRCc.normalized();
                    r.radius = 0.3;
                    robots->robots.push_back(r);
                }
            }

            emit(std::move(balls));
            emit(std::move(robots));

            if (log_level <= NUClear::DEBUG) {
                cv::resize(frame, frame, cv::Size(frame.cols, frame.rows));
                cv::imwrite("recordings/yolo.jpg", frame);
            }
        });
    }

}  // namespace module::vision
