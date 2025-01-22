/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#include "message/vision/GreenHorizon.hpp"
#include "message/vision/Robot.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/Vision.hpp"
#include "utility/vision/fourcc.hpp"
#include "utility/vision/projection.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

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
    using message::vision::GreenHorizon;
    using message::vision::Robot;
    using message::vision::Robots;

    using utility::math::coordinates::cartesianToReciprocalSpherical;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::math::geometry::point_in_convex_hull;
    using utility::support::Expression;
    using utility::vision::unproject;

    Yolo::Yolo(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Yolo.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Yolo.yaml
            log_level                       = config["log_level"].as<NUClear::LogLevel>();
            objects[0].confidence_threshold = config["ball_confidence_threshold"].as<double>();
            objects[1].confidence_threshold = config["goalpost_confidence_threshold"].as<double>();
            objects[2].confidence_threshold = config["robot_confidence_threshold"].as<double>();
            objects[3].confidence_threshold = config["intersection_confidence_threshold"].as<double>();
            objects[4].confidence_threshold = config["intersection_confidence_threshold"].as<double>();
            objects[5].confidence_threshold = config["intersection_confidence_threshold"].as<double>();
            cfg.nms_threshold               = config["nms_threshold"].as<double>();
            cfg.nms_score_threshold         = config["nms_score_threshold"].as<double>();

            // Compile the model and create inference request object
            compiled_model =
                ov::Core().compile_model(config["model_path"].as<std::string>(), config["device"].as<std::string>());
            infer_request = compiled_model.create_infer_request();
        });

        on<Trigger<Image>, Optional<With<GreenHorizon>>, Single>().then(
            "Yolo Main Loop",
            [this](const Image& img, const std::shared_ptr<const GreenHorizon>& horizon) {
                // Start timer for benchmarking
                auto start = std::chrono::high_resolution_clock::now();

                // -------- Convert image to cv::Mat -------
                const int width  = img.dimensions.x();
                const int height = img.dimensions.y();
                cv::Mat img_cv;
                switch (img.format) {
                    case utility::vision::fourcc("BGR3"):  // BGR3 not available in utility::vision::FOURCC.
                        img_cv = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(img.data.data()));
                        break;
                    case utility::vision::FOURCC::RGGB:
                        img_cv = cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(img.data.data()));
                        cv::cvtColor(img_cv, img_cv, cv::COLOR_BayerRG2RGB);
                        break;
                    case utility::vision::fourcc("RGBA"):
                        img_cv = cv::Mat(height, width, CV_8UC4, const_cast<uint8_t*>(img.data.data()));
                        cv::cvtColor(img_cv, img_cv, cv::COLOR_RGBA2BGR);
                        break;
                    default:
                        log<NUClear::WARN>("Image format not supported: ", utility::vision::fourcc(img.format));
                        return;
                }

                // -------- Preprocess the image -------
                int max               = MAX(width, height);
                cv::Mat letterbox_img = cv::Mat::zeros(max, max, CV_8UC3);
                img_cv.copyTo(letterbox_img(cv::Rect(0, 0, width, height)));
                cv::Mat blob =
                    cv::dnn::blobFromImage(letterbox_img, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true);

                // -------- Feed the blob into the input node of the Model -------
                // Get input port for model with one input
                auto input_port = compiled_model.input();
                // Create tensor from external memory
                ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), blob.ptr(0));
                // Set input tensor for model with one input
                infer_request.set_input_tensor(input_tensor);

                // -------- Perform Inference --------
                infer_request.infer();
                auto output = infer_request.get_output_tensor(0);

                // -------- Postprocess the result --------
                float* data = output.data<float>();
                cv::Mat output_buffer(output.get_shape()[1], output.get_shape()[2], CV_32F, data);
                transpose(output_buffer, output_buffer);  //[8400,84]
                std::vector<int> class_ids;
                std::vector<float> class_confidences;
                std::vector<cv::Rect> boxes;
                float scale = letterbox_img.size[0] / 640.0;

                // Figure out the bbox, class_id and class_score
                for (int i = 0; i < output_buffer.rows; i++) {
                    cv::Mat objects_scores = output_buffer.row(i).colRange(4, 10);
                    cv::Point class_id;
                    double confidence;
                    cv::minMaxLoc(objects_scores, 0, &confidence, 0, &class_id);

                    // Filter out the objects with confidence below the class threshold
                    if (confidence > objects[class_id.x].confidence_threshold) {
                        class_confidences.push_back(confidence);
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

                // Perform NMS (non maximum suppression) to remove overlapping boxes
                std::vector<int> indices;
                cv::dnn::NMSBoxes(boxes, class_confidences, cfg.nms_score_threshold, cfg.nms_threshold, indices);

                // -------- Emit Detections --------
                const Eigen::Isometry3d& Hwc = img.Hcw.inverse();

                auto balls               = std::make_unique<Balls>();
                auto robots              = std::make_unique<Robots>();
                auto goals               = std::make_unique<Goals>();
                auto field_intersections = std::make_unique<FieldIntersections>();
                auto bounding_boxes      = std::make_unique<BoundingBoxes>();

                // Common message fields
                balls->id = robots->id = goals->id = field_intersections->id = bounding_boxes->id = img.id;
                balls->timestamp = robots->timestamp = goals->timestamp = field_intersections->timestamp =
                    bounding_boxes->timestamp                           = img.timestamp;
                balls->Hcw = robots->Hcw = goals->Hcw = field_intersections->Hcw = bounding_boxes->Hcw = img.Hcw;

                // Helper function to simplify unprojection calls
                auto pix_to_ray = [&](double x, double y) {
                    // Normalize the pixel coordinates to the image width normalized dimensions
                    Eigen::Vector2d norm_dim =
                        Eigen::Vector2d(img.dimensions.x(), img.dimensions.y()) / img.dimensions.x();
                    return unproject(Eigen::Matrix<double, 2, 1>(x / img.dimensions.x(), y / img.dimensions.x()),
                                     img.lens,
                                     norm_dim);
                };

                // Helper function to simplify projecting rays onto the field plane then transforming into camera space
                auto ray_to_camera_space = [&](const Eigen::Matrix<double, 3, 1>& ray) {
                    Eigen::Vector3d uBCw = Hwc.rotation() * ray;
                    Eigen::Vector3d rPWw = uBCw * std::abs(Hwc.translation().z() / uBCw.z()) + Hwc.translation();
                    return Hwc.inverse() * rPWw;
                };

                for (size_t i = 0; i < indices.size(); i++) {
                    // Get the index of the detected object from list of indices
                    int idx = indices[i];
                    // Get the class id associated with the detected object
                    int class_id = class_ids[idx];

                    // Convert the bounding box points to unit vectors (rays) in the camera {c} space
                    Eigen::Vector3d top_left_ray  = pix_to_ray(boxes[idx].x, boxes[idx].y);
                    Eigen::Vector3d top_right_ray = pix_to_ray(boxes[idx].x + boxes[idx].width, boxes[idx].y);
                    Eigen::Vector3d bottom_right_ray =
                        pix_to_ray(boxes[idx].x + boxes[idx].width, boxes[idx].y + boxes[idx].height);
                    Eigen::Vector3d bottom_left_ray = pix_to_ray(boxes[idx].x, boxes[idx].y + boxes[idx].height);
                    Eigen::Vector3d centre_ray =
                        pix_to_ray(boxes[idx].x + boxes[idx].width / 2.0, boxes[idx].y + boxes[idx].height / 2.0);
                    Eigen::Vector3d bottom_centre_ray =
                        pix_to_ray(boxes[idx].x + boxes[idx].width / 2.0, boxes[idx].y + boxes[idx].height);
                    Eigen::Vector3d top_centre_ray = pix_to_ray(boxes[idx].x + boxes[idx].width / 2.0, boxes[idx].y);

                    auto bbox        = std::make_unique<BoundingBox>();
                    bbox->name       = objects[class_id].name;
                    bbox->confidence = class_confidences[idx];
                    bbox->corners.push_back(top_left_ray);
                    bbox->corners.push_back(top_right_ray);
                    bbox->corners.push_back(bottom_right_ray);
                    bbox->corners.push_back(bottom_left_ray);


                    if (objects[class_id].name == "ball") {
                        // Get the vector in world space to check if it is in the field
                        Eigen::Vector3d rBWw = img.Hcw.inverse() * ray_to_camera_space(centre_ray);
                        // Only consider vision measurements within the green horizon, if it exists
                        if (horizon != nullptr && !point_in_convex_hull(horizon->horizon, rBWw)) {
                            continue;  // skip this run of the loop and continue the for loop
                        }

                        Ball b;
                        b.uBCc = ray_to_camera_space(centre_ray).normalized();
                        b.measurements.emplace_back();
                        b.measurements.back().type = Ball::MeasurementType::PROJECTION;
                        b.measurements.back().rBCc = ray_to_camera_space(centre_ray);
                        // Calculate the angular radius of the ball in camera space
                        b.radius = bottom_centre_ray.dot(bottom_left_ray);
                        b.colour.fill(1.0);
                        balls->balls.push_back(b);
                        bbox->colour = objects[class_id].colour;
                        bounding_boxes->bounding_boxes.push_back(*bbox);
                    }

                    if (objects[class_id].name == "goal post") {
                        Goal g;
                        g.measurements.emplace_back();
                        g.measurements.back().type = Goal::MeasurementType::CENTRE;
                        g.measurements.back().rGCc = ray_to_camera_space(bottom_centre_ray);
                        g.post.top                 = top_centre_ray;
                        g.post.bottom              = bottom_centre_ray;
                        g.post.distance            = ray_to_camera_space(bottom_centre_ray).norm();
                        g.side                     = Goal::Side::UNKNOWN_SIDE;
                        g.screen_angular           = cartesianToSpherical(g.post.bottom).tail<2>();
                        goals->goals.push_back(std::move(g));
                        bbox->colour = objects[class_id].colour;
                        bounding_boxes->bounding_boxes.push_back(*bbox);
                    }

                    if (objects[class_id].name == "robot") {
                        // Get the vector in world space to check if it is in the field
                        Eigen::Vector3d rRWw = img.Hcw.inverse() * ray_to_camera_space(bottom_centre_ray);
                        // Only consider vision measurements within the green horizon, if it exists
                        if (horizon != nullptr && !point_in_convex_hull(horizon->horizon, rRWw)) {
                            continue;  // skip this run of the loop and continue the for loop
                        }

                        Robot r;
                        r.rRCc   = ray_to_camera_space(bottom_centre_ray);
                        r.radius = bottom_centre_ray.dot(bottom_left_ray);
                        robots->robots.push_back(r);
                        bbox->colour = objects[class_id].colour;
                        bounding_boxes->bounding_boxes.push_back(*bbox);
                    }

                    if (objects[class_id].name == "L-intersection" || objects[class_id].name == "T-intersection"
                        || objects[class_id].name == "X-intersection") {
                        FieldIntersection i;
                        // Project the centre ray onto the ground plane in world {w} space
                        Eigen::Vector3d uICw = Hwc.rotation() * centre_ray;
                        Eigen::Vector3d rIWw = uICw * std::abs(Hwc.translation().z() / uICw.z()) + Hwc.translation();

                        // Only consider vision measurements within the green horizon, if it exists
                        if (horizon != nullptr && !point_in_convex_hull(horizon->horizon, rIWw)) {
                            continue;  // skip this run of the loop and continue the for loop
                        }

                        i.rIWw = rIWw;
                        if (objects[class_id].name == "L-intersection") {
                            i.type       = FieldIntersection::IntersectionType::L_INTERSECTION;
                            bbox->colour = objects[class_id].colour;
                        }
                        else if (objects[class_id].name == "T-intersection") {
                            i.type       = FieldIntersection::IntersectionType::T_INTERSECTION;
                            bbox->colour = objects[class_id].colour;
                        }
                        else if (objects[class_id].name == "X-intersection") {
                            i.type       = FieldIntersection::IntersectionType::X_INTERSECTION;
                            bbox->colour = objects[class_id].colour;
                        }
                        field_intersections->intersections.push_back(std::move(i));
                        bounding_boxes->bounding_boxes.push_back(*bbox);
                    }
                }

                emit(std::move(balls));
                emit(std::move(robots));
                emit(std::move(goals));
                emit(std::move(field_intersections));
                emit(std::move(bounding_boxes));

                // -------- Benchmark --------
                if (log_level <= DEBUG) {
                    auto end      = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                    log<DEBUG>("Yolo took: ", duration, "ms");
                    log<DEBUG>("FPS: ", 1000.0 / duration);
                }
            });
    }

}  // namespace module::vision
