#include "Yolo.hpp"

#include <chrono>
#include <getopt.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/vision/Ball.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/projection.hpp"

namespace module::vision {

    using extension::Configuration;

    using message::input::Image;
    using message::vision::Ball;
    using message::vision::Balls;


    using utility::math::coordinates::cartesianToReciprocalSpherical;
    using utility::support::Expression;
    using utility::vision::unproject;

    cv::Point2f correct_distortion(const cv::Point2f& point, const Image& img) {
        float k1 = img.lens.k.x();
        float k2 = img.lens.k.y();

        // Shift point by centre offset
        float x = point.x - img.lens.centre.x();
        float y = point.y - img.lens.centre.y();

        // Calculate r^2
        float r2 = x * x + y * y;

        // Apply distortion k1 and k2
        float x_distorted = x * (1 + k1 * r2 + k2 * r2 * r2);
        float y_distorted = y * (1 + k1 * r2 + k2 * r2 * r2);

        return cv::Point2f(x_distorted + img.lens.centre.x(), y_distorted + img.lens.centre.y());
    }

    // Convert pixel coordinates to a unit vector (ray) in camera space
    Eigen::Vector3d compute_ray(const Eigen::Vector2d& pixel, const Image& img) {
        // Convert to Normalized Device Coordinates (NDC)
        double x_ndc = (2.0 * (pixel.x() - img.lens.centre.x()) / img.dimensions.x()) - 1.0;
        double y_ndc = 1.0 - (2.0 * (pixel.y() - img.lens.centre.y()) / img.dimensions.y());

        // Aspect ratio
        double aspect_ratio = img.dimensions.x() / img.dimensions.y();

        // Apply field of view and aspect ratio to convert to camera space
        double x_camera = 1.0;
        double y_camera = -x_ndc * aspect_ratio * img.lens.focal_length * std::tan(img.lens.fov / 2.0);
        double z_camera = y_ndc * img.lens.focal_length * std::tan(img.lens.fov / 2.0);

        Eigen::Vector3d ray_direction(x_camera, y_camera, z_camera);
        ray_direction.normalize();

        return ray_direction;
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
            log<NUClear::DEBUG>("Image width: ", width);
            log<NUClear::DEBUG>("Image height: ", height);
            cv::Mat frame = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(img.data.data()));


            // -------- Run Inference --------
            auto start                    = std::chrono::high_resolution_clock::now();
            std::vector<Detection> output = inf.runInference(frame);
            auto stop                     = std::chrono::high_resolution_clock::now();
            auto duration                 = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            int detections                = output.size();
            log<NUClear::DEBUG>("Inference took: ", duration.count(), "ms");
            log<NUClear::DEBUG>("Number of detections: ", detections);

            // -------- Draw Detections --------

            auto balls       = std::make_unique<Balls>();
            balls->id        = img.id;         // camera id
            balls->timestamp = img.timestamp;  // time when the image was taken
            balls->Hcw       = img.Hcw;        // world to camera transform at the time the image was taken

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
                    Eigen::Matrix<double, 2, 1> box_bottom_centre(detection.box.x + detection.box.width / 2.0f,
                                                                  detection.box.y + detection.box.height / 2.0f);
                    // Correct for lens distortion
                    // box_bottom_centre = undistort_point(box_bottom_centre, img);
                    // Convert to unit vector in camera space
                    Eigen::Matrix<double, 3, 1> uPCc = compute_ray(box_bottom_centre, img);
                    // Project the unit vector onto the ground plane
                    Eigen::Vector3d uPCw = Hwc.rotation() * uPCc;
                    Eigen::Vector3d rPWw = uPCw * std::abs(Hwc.translation().z() / uPCw.z()) + Hwc.translation();
                    Eigen::Vector3d rPCc = Hwc.inverse() * rPWw;

                    // Add ball to balls message
                    Ball b;
                    b.uBCc = rPCc.normalized();
                    b.measurements.emplace_back();
                    b.measurements.back().type  = Ball::MeasurementType::PROJECTION;
                    b.measurements.back().srBCc = cartesianToReciprocalSpherical(rPCc);
                    b.radius                    = 0.1;
                    balls->balls.push_back(b);
                }
            }

            emit(std::move(balls));

            if (log_level <= NUClear::DEBUG) {
                cv::resize(frame, frame, cv::Size(frame.cols, frame.rows));
                cv::imwrite("recordings/yolo.jpg", frame);
            }
        });
    }

}  // namespace module::vision
