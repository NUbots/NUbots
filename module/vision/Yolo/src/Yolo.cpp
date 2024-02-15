#include "Yolo.hpp"

#include <chrono>
#include <getopt.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/FieldIntersections.hpp"
#include "message/vision/Goal.hpp"
#include "message/vision/Robot.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/projection.hpp"

namespace module::vision {

    using extension::Configuration;

    using message::input::Image;
    using message::vision::Ball;
    using message::vision::Balls;
    using message::vision::FieldIntersection;
    using message::vision::FieldIntersections;
    using message::vision::Goal;
    using message::vision::Goals;
    using message::vision::Robot;
    using message::vision::Robots;

    using utility::math::coordinates::cartesianToReciprocalSpherical;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::support::Expression;
    using utility::vision::unproject;

    Eigen::Vector2d correct_distortion(const Eigen::Vector2d& pixel, const Image& img) {
        // Shift point by centre offset
        double x = pixel.x() - img.lens.centre.x();
        double y = pixel.y() - img.lens.centre.y();

        // Calculate r^2
        double r2 = x * x + y * y;

        // Apply distortion k1 and k2
        double k1          = img.lens.k.x();
        double k2          = img.lens.k.y();
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
        double x_ndc = 2.0 * corrected_pixel.x() / img.dimensions.x() - 1.0;
        double y_ndc = 1.0 - 2.0 * corrected_pixel.y() / img.dimensions.y();

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

    std::vector<cv::Scalar> colors = {cv::Scalar(0, 0, 255),
                                      cv::Scalar(0, 255, 0),
                                      cv::Scalar(255, 0, 0),
                                      cv::Scalar(255, 100, 50),
                                      cv::Scalar(50, 100, 255),
                                      cv::Scalar(255, 50, 100)};
    const std::vector<std::string> class_names =
        {"ball", "goal post", "robot", "L-intersection", "T-intersection", "X-intersection"};

    using namespace cv;
    using namespace dnn;

    // Keep the ratio before resize
    Mat letterbox(const cv::Mat& source) {
        int col    = source.cols;
        int row    = source.rows;
        int _max   = MAX(col, row);
        Mat result = Mat::zeros(_max, _max, CV_8UC3);
        source.copyTo(result(Rect(0, 0, col, row)));
        return result;
    }

    Yolo::Yolo(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Yolo.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Yolo.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.model_path  = config["model_path"].as<std::string>();
        });

        on<Startup>().then("Load Yolo Model", [this] {
            compiled_model = core.compile_model(cfg.model_path, "CPU");
            infer_request  = compiled_model.create_infer_request();
        });

        on<Trigger<Image>, Single>().then([this](const Image& img) {
            const Eigen::Isometry3d& Hwc = img.Hcw.inverse();

            // -------- Convert image to cv::Mat and preprocess --------
            const int width   = img.dimensions.x();
            const int height  = img.dimensions.y();
            cv::Mat img_cv    = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(img.data.data()));
            Mat letterbox_img = letterbox(img_cv);
            float scale       = letterbox_img.size[0] / 640.0;
            Mat blob          = blobFromImage(letterbox_img, 1.0 / 255.0, Size(640, 640), Scalar(), true);

            // -------- Feed the blob into the input node of the Model -------
            // Get input port for model with one input
            auto input_port = compiled_model.input();
            // Create tensor from external memory
            ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), blob.ptr(0));
            // Set input tensor for model with one input
            infer_request.set_input_tensor(input_tensor);

            // -------- Start inference --------

            // Start timer
            auto start = std::chrono::high_resolution_clock::now();
            infer_request.infer();
            // Stop the timer
            auto end      = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            log<NUClear::DEBUG>("Inference took: ", duration.count(), "ms");
            log<NUClear::DEBUG>("FPS: ", 1000.0 / duration.count());

            // -------- Get the inference result --------
            auto output       = infer_request.get_output_tensor(0);
            auto output_shape = output.get_shape();

            // -------- Postprocess the result --------
            float* data = output.data<float>();
            Mat output_buffer(output_shape[1], output_shape[2], CV_32F, data);
            transpose(output_buffer, output_buffer);  //[8400,84]
            float score_threshold = 0.1;
            float nms_threshold   = 0.5;
            std::vector<int> class_ids;
            std::vector<float> class_scores;
            std::vector<Rect> boxes;

            // Figure out the bbox, class_id and class_score
            for (int i = 0; i < output_buffer.rows; i++) {
                Mat classes_scores = output_buffer.row(i).colRange(4, 10);
                Point class_id;
                double maxClassScore;
                minMaxLoc(classes_scores, 0, &maxClassScore, 0, &class_id);

                if (maxClassScore > score_threshold) {
                    class_scores.push_back(maxClassScore);
                    class_ids.push_back(class_id.x);
                    float cx = output_buffer.at<float>(i, 0);
                    float cy = output_buffer.at<float>(i, 1);
                    float w  = output_buffer.at<float>(i, 2);
                    float h  = output_buffer.at<float>(i, 3);

                    int left   = int((cx - 0.5 * w) * scale);
                    int top    = int((cy - 0.5 * h) * scale);
                    int width  = int(w * scale);
                    int height = int(h * scale);

                    boxes.push_back(Rect(left, top, width, height));
                }
            }

            // NMS
            std::vector<int> indices;
            NMSBoxes(boxes, class_scores, score_threshold, nms_threshold, indices);

            // -------- Emit Detections --------

            auto balls       = std::make_unique<Balls>();
            balls->id        = img.id;
            balls->timestamp = img.timestamp;
            balls->Hcw       = img.Hcw;

            auto robots       = std::make_unique<Robots>();
            robots->timestamp = img.timestamp;
            robots->id        = img.id;
            robots->Hcw       = img.Hcw;

            auto goals       = std::make_unique<Goals>();
            goals->timestamp = img.timestamp;
            goals->id        = img.id;
            goals->Hcw       = img.Hcw;

            auto field_intersections       = std::make_unique<FieldIntersections>();
            field_intersections->timestamp = img.timestamp;
            field_intersections->id        = img.id;
            field_intersections->Hcw       = img.Hcw;

            for (size_t i = 0; i < indices.size(); i++) {
                int index    = indices[i];
                int class_id = class_ids[index];
                rectangle(img_cv, boxes[index], colors[class_id % 6], 2, 8);
                std::string class_name = class_names[class_id];
                std::string label      = class_name + ":" + std::to_string(class_scores[index]).substr(0, 4);
                Size textSize          = cv::getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, 0);
                Rect textBox(boxes[index].tl().x, boxes[index].tl().y - 15, textSize.width, textSize.height + 5);
                cv::rectangle(img_cv, textBox, colors[class_id % 6], FILLED);
                putText(img_cv,
                        label,
                        Point(boxes[index].tl().x, boxes[index].tl().y - 5),
                        FONT_HERSHEY_SIMPLEX,
                        0.5,
                        Scalar(255, 255, 255));

                if (class_name == "ball") {
                    // Calculate the middle of the bottom border of the detection box
                    Eigen::Matrix<double, 2, 1> box_centre(boxes[index].x + boxes[index].width / 2.0,
                                                           boxes[index].y + boxes[index].height / 2.0);
                    // Calculate the bottom left corner of the detection box
                    Eigen::Matrix<double, 2, 1> box_left(boxes[index].x, boxes[index].y + boxes[index].height / 2.0);

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

                if (class_name == "robot") {
                    // Calculate the bottom of box centre
                    Eigen::Matrix<double, 2, 1> box_bottom_centre(boxes[index].x + boxes[index].width / 2.0,
                                                                  boxes[index].y + boxes[index].height);

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

                if (class_name == "goal post") {
                    // Calculate the middle of the bottom border of the detection box
                    Eigen::Matrix<double, 2, 1> box_bottom_middle(boxes[index].x + boxes[index].width / 2.0,
                                                                  boxes[index].y + boxes[index].height);
                    // Calculate the middle of the top border of the detection box
                    Eigen::Matrix<double, 2, 1> box_top_middle(boxes[index].x + boxes[index].width / 2.0,
                                                               boxes[index].y);

                    // Convert to unit vector in camera space
                    Eigen::Matrix<double, 3, 1> uGbCc = unproject(box_bottom_middle, img);
                    Eigen::Matrix<double, 3, 1> uGtCc = unproject(box_top_middle, img);

                    // Project the bottom unit vector onto the ground plane
                    Eigen::Vector3d uGbCw = Hwc.rotation() * uGbCc;
                    Eigen::Vector3d rGbWw = uGbCw * std::abs(Hwc.translation().z() / uGbCw.z()) + Hwc.translation();
                    Eigen::Vector3d rGbCc = Hwc.inverse() * rGbWw;

                    // Add goal to goals message
                    Goal g;
                    g.measurements.emplace_back();
                    g.measurements.back().type  = Goal::MeasurementType::CENTRE;
                    g.measurements.back().srGCc = cartesianToReciprocalSpherical(rGbCc);
                    g.post.top                  = uGtCc;
                    g.post.bottom               = uGbCc;
                    g.post.distance             = rGbCc.norm();
                    g.side                      = Goal::Side::UNKNOWN_SIDE;
                    g.screen_angular            = cartesianToSpherical(g.post.bottom).tail<2>();
                    goals->goals.push_back(std::move(g));
                }

                if (class_name == "L-intersection" || class_name == "T-intersection"
                    || class_name == "X-intersection") {
                    // Calculate the middle of the bottom border of the detection box
                    Eigen::Matrix<double, 2, 1> box_middle(boxes[index].x + boxes[index].width / 2.0,
                                                           boxes[index].y + boxes[index].height / 2.0);

                    // Convert to unit vector in camera space
                    Eigen::Matrix<double, 3, 1> uICc = unproject(box_middle, img);

                    // Project the unit vector onto the ground plane
                    Eigen::Vector3d uICw = Hwc.rotation() * uICc;
                    Eigen::Vector3d rIWw = uICw * std::abs(Hwc.translation().z() / uICw.z()) + Hwc.translation();

                    // Add intersection to intersections message
                    FieldIntersection i;
                    i.rIWw = rIWw;
                    if (class_name == "L-intersection") {
                        i.type = FieldIntersection::IntersectionType::L_INTERSECTION;
                    }
                    else if (class_name == "T-intersection") {
                        i.type = FieldIntersection::IntersectionType::T_INTERSECTION;
                    }
                    else if (class_name == "X-intersection") {
                        i.type = FieldIntersection::IntersectionType::X_INTERSECTION;
                    }
                    field_intersections->intersections.push_back(std::move(i));
                }
            }

            emit(std::move(balls));
            emit(std::move(robots));
            emit(std::move(goals));
            emit(std::move(field_intersections));

            if (log_level <= NUClear::DEBUG) {
                cv::resize(img_cv, img_cv, cv::Size(img_cv.cols, img_cv.rows));
                cv::imwrite("recordings/yolo.jpg", img_cv);
            }
        });
    }

}  // namespace module::vision
