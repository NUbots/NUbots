#include "Yolo.hpp"

#include <chrono>
#include <getopt.h>
#include <iostream>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/BoundingBoxes.hpp"
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
    using message::vision::BoundingBox;
    using message::vision::BoundingBoxes;
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

    Yolo::Yolo(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Yolo.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Yolo.yaml
            this->log_level                       = config["log_level"].as<NUClear::LogLevel>();
            cfg.model_path                        = config["model_path"].as<std::string>();
            cfg.ball_confidence_threshold         = config["ball_confidence_threshold"].as<double>();
            cfg.goal_confidence_threshold         = config["goal_confidence_threshold"].as<double>();
            cfg.robot_confidence_threshold        = config["robot_confidence_threshold"].as<double>();
            cfg.intersection_confidence_threshold = config["intersection_confidence_threshold"].as<double>();
            cfg.image_format                      = config["image_format"].as<std::string>();
            cfg.device                            = config["device"].as<std::string>();
        });

        on<Startup>().then("Load Yolo Model", [this] {
            compiled_model = core.compile_model(cfg.model_path, cfg.device);
            infer_request  = compiled_model.create_infer_request();
        });

        on<Trigger<Image>, Single>().then([this](const Image& img) {
            // Start timer
            auto start = std::chrono::high_resolution_clock::now();

            // -------- Convert image to cv::Mat -------
            const int width  = img.dimensions.x();
            const int height = img.dimensions.y();
            cv::Mat img_cv;
            if (cfg.image_format == "BayerRG8") {
                img_cv = cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(img.data.data()));
                cv::cvtColor(img_cv, img_cv, cv::COLOR_BayerRG2RGB);
            }
            else if (cfg.image_format == "BRGA") {
                img_cv = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(img.data.data()));
            }
            else {
                log<NUClear::ERROR>("Unsupported image format", cfg.image_format);
                powerplant.shutdown();
            }

            // -------- Preprocess the image -------
            cv::Mat letterbox_img = letterbox(img_cv);
            float scale           = letterbox_img.size[0] / 640.0;
            cv::Mat blob = cv::dnn::blobFromImage(letterbox_img, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true);

            // -------- Feed the blob into the input node of the Model -------
            // Get input port for model with one input
            auto input_port = compiled_model.input();
            // Create tensor from external memory
            ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), blob.ptr(0));
            // Set input tensor for model with one input
            infer_request.set_input_tensor(input_tensor);

            // -------- Perform Inference --------
            infer_request.infer();
            auto output       = infer_request.get_output_tensor(0);
            auto output_shape = output.get_shape();

            // -------- Postprocess the result --------
            float* data = output.data<float>();
            cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, data);
            transpose(output_buffer, output_buffer);  //[8400,84]
            float score_threshold = 0.1;
            float nms_threshold   = 0.5;
            std::vector<int> class_ids;
            std::vector<float> class_scores;
            std::vector<cv::Rect> boxes;

            // Figure out the bbox, class_id and class_score
            for (int i = 0; i < output_buffer.rows; i++) {
                cv::Mat classes_scores = output_buffer.row(i).colRange(4, 10);
                cv::Point class_id;
                double maxClassScore;
                cv::minMaxLoc(classes_scores, 0, &maxClassScore, 0, &class_id);

                if (maxClassScore > score_threshold) {
                    class_scores.push_back(maxClassScore);
                    class_ids.push_back(class_id.x);
                    float cx = output_buffer.at<float>(i, 0);
                    float cy = output_buffer.at<float>(i, 1);
                    float w  = output_buffer.at<float>(i, 2);
                    float h  = output_buffer.at<float>(i, 3);

                    // Scale the bbox to the original image dimensions
                    int left   = int((cx - 0.5 * w) * scale);
                    int top    = int((cy - 0.5 * h) * scale);
                    int width  = int(w * scale);
                    int height = int(h * scale);
                    boxes.push_back(cv::Rect(left, top, width, height));
                }
            }

            // NMS to remove overlapping boxes
            std::vector<int> indices;
            cv::dnn::NMSBoxes(boxes, class_scores, score_threshold, nms_threshold, indices);

            // -------- Emit Detections --------
            const Eigen::Isometry3d& Hwc = img.Hcw.inverse();

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

            auto bounding_boxes       = std::make_unique<BoundingBoxes>();
            bounding_boxes->timestamp = img.timestamp;
            bounding_boxes->id        = img.id;
            bounding_boxes->Hcw       = img.Hcw;

            for (size_t i = 0; i < indices.size(); i++) {
                int index    = indices[i];
                int class_id = class_ids[index];
                rectangle(img_cv, boxes[index], colours[class_id % 6], 2, 8);
                std::string class_name = class_names[class_id];
                double confidence      = class_scores[index];
                std::string label      = class_name + ":" + std::to_string(confidence).substr(0, 4);
                cv::Size textSize      = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, 0);
                cv::Rect textBox(boxes[index].tl().x, boxes[index].tl().y - 15, textSize.width, textSize.height + 5);
                cv::rectangle(img_cv, textBox, colours[class_id % 6], cv::FILLED);
                cv::putText(img_cv,
                            label,
                            cv::Point(boxes[index].tl().x, boxes[index].tl().y - 5),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.5,
                            cv::Scalar(255, 255, 255));

                // Calculate the image width normalized dimensions of the image
                Eigen::Matrix<double, 2, 1> normalized_dim =
                    Eigen::Matrix<double, 2, 1>(img.dimensions.x(), img.dimensions.y()) / img.dimensions.x();

                double box_x       = boxes[index].x;
                double box_y       = boxes[index].y;
                double box_width   = boxes[index].width;
                double box_height  = boxes[index].height;
                double half_width  = box_width / 2.0;
                double half_height = box_height / 2.0;
                double norm_factor = img.dimensions.x();

                // Helper lambda to simplify unprojection calls
                auto unproject_point = [&](double x, double y) {
                    return unproject(Eigen::Matrix<double, 2, 1>(x / norm_factor, y / norm_factor),
                                     img.lens,
                                     normalized_dim);
                };

                // Convert the bounding box points to unit vectors (rays) in the camera {c} space
                Eigen::Matrix<double, 3, 1> top_left_ray     = unproject_point(box_x, box_y);
                Eigen::Matrix<double, 3, 1> top_right_ray    = unproject_point(box_x + box_width, box_y);
                Eigen::Matrix<double, 3, 1> bottom_right_ray = unproject_point(box_x + box_width, box_y + box_height);
                Eigen::Matrix<double, 3, 1> bottom_left_ray  = unproject_point(box_x, box_y + box_height);
                Eigen::Matrix<double, 3, 1> centre_ray       = unproject_point(box_x + half_width, box_y + half_height);
                Eigen::Matrix<double, 3, 1> bottom_centre_ray = unproject_point(box_x + half_width, box_y + box_height);
                Eigen::Matrix<double, 3, 1> top_centre_ray    = unproject_point(box_x + half_width, box_y);

                // Create a bounding box message
                auto bbox        = std::make_unique<BoundingBox>();
                bbox->name       = class_name;
                bbox->confidence = confidence;
                bbox->corners.push_back(top_left_ray);
                bbox->corners.push_back(top_right_ray);
                bbox->corners.push_back(bottom_right_ray);
                bbox->corners.push_back(bottom_left_ray);

                if (class_name == "ball" && confidence > cfg.ball_confidence_threshold) {
                    // Project the centre ray onto the ground plane
                    Eigen::Vector3d uBCw = Hwc.rotation() * centre_ray;
                    Eigen::Vector3d rPWw = uBCw * std::abs(Hwc.translation().z() / uBCw.z()) + Hwc.translation();
                    Eigen::Vector3d rBCc = Hwc.inverse() * rPWw;

                    Ball b;
                    b.uBCc = rBCc.normalized();
                    b.measurements.emplace_back();
                    b.measurements.back().type = Ball::MeasurementType::PROJECTION;
                    b.measurements.back().rBCc = rBCc;
                    // Calculate the angular radius of the ball in camera space
                    b.radius = bottom_centre_ray.dot(bottom_left_ray);
                    b.colour.fill(1.0);
                    balls->balls.push_back(b);

                    bbox->colour =
                        Eigen::Vector4d(colours[0][0] / 255.0, colours[0][1] / 255.0, colours[0][2] / 255.0, 1.0);
                }

                if (class_name == "goal post" && confidence > cfg.goal_confidence_threshold) {
                    // Project the bottom centre ray onto the ground plane
                    Eigen::Vector3d uGbCw = Hwc.rotation() * bottom_centre_ray;
                    Eigen::Vector3d rGbWw = uGbCw * std::abs(Hwc.translation().z() / uGbCw.z()) + Hwc.translation();
                    Eigen::Vector3d rGbCc = Hwc.inverse() * rGbWw;

                    Goal g;
                    g.measurements.emplace_back();
                    g.measurements.back().type  = Goal::MeasurementType::CENTRE;
                    g.measurements.back().srGCc = cartesianToReciprocalSpherical(rGbCc);
                    g.post.top                  = top_centre_ray;
                    g.post.bottom               = bottom_centre_ray;
                    g.post.distance             = rGbCc.norm();
                    g.side                      = Goal::Side::UNKNOWN_SIDE;
                    g.screen_angular            = cartesianToSpherical(g.post.bottom).tail<2>();
                    goals->goals.push_back(std::move(g));
                    bbox->colour =
                        Eigen::Vector4d(colours[1][0] / 255.0, colours[1][1] / 255.0, colours[1][2] / 255.0, 1.0);
                }

                if (class_name == "robot" && confidence > cfg.robot_confidence_threshold) {
                    // Project the centre ray onto the ground plane
                    Eigen::Vector3d uRCw = Hwc.rotation() * bottom_centre_ray;
                    Eigen::Vector3d rRWw = uRCw * std::abs(Hwc.translation().z() / uRCw.z()) + Hwc.translation();
                    Eigen::Vector3d rRCc = Hwc.inverse() * rRWw;

                    Robot r;
                    r.rRCc   = rRCc.normalized();
                    r.radius = 0.3;
                    robots->robots.push_back(r);
                    bbox->colour =
                        Eigen::Vector4d(colours[2][0] / 255.0, colours[2][1] / 255.0, colours[2][2] / 255.0, 1.0);
                }

                if ((class_name == "L-intersection" || class_name == "T-intersection" || class_name == "X-intersection")
                    && (confidence > cfg.intersection_confidence_threshold)) {
                    // Project the centre ray onto the ground plane
                    Eigen::Vector3d uICw = Hwc.rotation() * centre_ray;
                    Eigen::Vector3d rIWw = uICw * std::abs(Hwc.translation().z() / uICw.z()) + Hwc.translation();

                    FieldIntersection i;
                    i.rIWw = rIWw;
                    if (class_name == "L-intersection") {
                        i.type = FieldIntersection::IntersectionType::L_INTERSECTION;
                        bbox->colour =
                            Eigen::Vector4d(colours[3][0] / 255.0, colours[3][1] / 255.0, colours[3][2] / 255.0, 1.0);
                    }
                    else if (class_name == "T-intersection") {
                        i.type = FieldIntersection::IntersectionType::T_INTERSECTION;
                        bbox->colour =
                            Eigen::Vector4d(colours[4][0] / 255.0, colours[4][1] / 255.0, colours[4][2] / 255.0, 1.0);
                    }
                    else if (class_name == "X-intersection") {
                        i.type = FieldIntersection::IntersectionType::X_INTERSECTION;
                        bbox->colour =
                            Eigen::Vector4d(colours[5][0] / 255.0, colours[5][1] / 255.0, colours[5][2] / 255.0, 1.0);
                    }
                    field_intersections->intersections.push_back(std::move(i));
                }

                bounding_boxes->bounding_boxes.push_back(*bbox);
            }

            emit(std::move(balls));
            emit(std::move(robots));
            emit(std::move(goals));
            emit(std::move(field_intersections));
            emit(std::move(bounding_boxes));

            // -------- Debug --------
            if (log_level <= NUClear::DEBUG) {
                // Benchmark inference time
                auto end      = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                log<NUClear::DEBUG>("Yolo took: ", duration.count(), "ms");
                log<NUClear::DEBUG>("FPS: ", 1000.0 / duration.count());

                // Save image to file
                cv::resize(img_cv, img_cv, cv::Size(img_cv.cols, img_cv.rows));
                cv::imwrite("recordings/yolo.jpg", img_cv);
            }
        });
    }

}  // namespace module::vision
