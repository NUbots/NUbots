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
#include "RFDETR.hpp"

#include <chrono>
#include <cmath>
#include <limits>
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

    RFDETR::RFDETR(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("RFDETR.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RFDETR.yaml
            log_level                       = config["log_level"].as<NUClear::LogLevel>();
            objects[0].confidence_threshold = config["ball_confidence_threshold"].as<double>();
            objects[1].confidence_threshold = config["goalpost_confidence_threshold"].as<double>();
            objects[2].confidence_threshold = config["robot_confidence_threshold"].as<double>();
            objects[3].confidence_threshold = config["intersection_confidence_threshold"].as<double>();
            objects[4].confidence_threshold = config["intersection_confidence_threshold"].as<double>();
            objects[5].confidence_threshold = config["intersection_confidence_threshold"].as<double>();
            cfg.score_threshold             = config["score_threshold"].as<double>();
            cfg.class_offset                = config["class_offset"].as<int>();

            // Compile the model and create inference request object
            try {
                std::string model_path = config["model_path"].as<std::string>();
                std::string device     = config["device"].as<std::string>();

                log<INFO>("Loading RF-DETR model from: ", model_path);
                log<INFO>("Using device: ", device);

                ov::Core core{};

                // Try to fallback to CPU if GPU fails
                try {
                    compiled_model = core.compile_model(model_path, device);
                }
                catch (const std::exception& e) {
                    if (device == "GPU") {
                        log<WARN>("Failed to compile model on GPU, falling back to CPU: ", e.what());
                        compiled_model = core.compile_model(model_path, "CPU");
                    }
                    else {
                        throw;
                    }
                }

                infer_request = compiled_model.create_infer_request();
                log<INFO>("Model loaded successfully");
            }
            catch (const std::exception& e) {
                log<ERROR>("Failed to load RF-DETR model: ", e.what());
                throw;
            }
        });

        on<Trigger<Image>, Optional<With<GreenHorizon>>, Single>().then(
            "RFDETR Main Loop",
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
                    default: log<WARN>("Image format not supported: ", utility::vision::fourcc(img.format)); return;
                }

                // -------- Preprocess the image -------
                // RF-DETR uses a plain square resize (no letterbox); normalized box outputs are
                // aspect-invariant so they map straight back to the original image dimensions.
                auto input_port      = compiled_model.input();
                auto input_shape     = input_port.get_shape();
                int model_input_size = static_cast<int>(input_shape[2]);  // NCHW

                // Scale to [0, 1] and convert BGR -> RGB (swapRB) to match training.
                cv::Mat blob = cv::dnn::blobFromImage(img_cv,
                                                      1.0 / 255.0,
                                                      cv::Size(model_input_size, model_input_size),
                                                      cv::Scalar(),
                                                      true,
                                                      false);

                // Apply ImageNet mean/std per channel (RGB order). blobFromImage cannot divide by a
                // per-channel std, so do it here in-place on the NCHW blob.
                for (int c = 0; c < 3; c++) {
                    cv::Mat channel(model_input_size, model_input_size, CV_32F, blob.ptr<float>(0, c));
                    channel -= imagenet_mean[c];
                    channel /= imagenet_std[c];
                }

                // -------- Feed the blob into the input node of the Model -------
                ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape());
                std::memcpy(input_tensor.data<float>(), blob.ptr<float>(), input_tensor.get_byte_size());
                infer_request.set_input_tensor(input_tensor);

                // -------- Perform Inference --------
                try {
                    infer_request.infer();
                }
                catch (const std::exception& e) {
                    log<ERROR>("Inference failed: ", e.what());
                    return;
                }

                // -------- Identify outputs --------
                // RF-DETR emits two tensors: boxes [1, Q, 4] (normalized cxcywh) and logits [1, Q, C].
                // Identify them by the size of the last dimension (== 4 -> boxes).
                ov::Tensor boxes_tensor;
                ov::Tensor logits_tensor;
                bool have_boxes  = false;
                bool have_logits = false;
                for (size_t i = 0; i < compiled_model.outputs().size(); i++) {
                    ov::Tensor t  = infer_request.get_output_tensor(i);
                    ov::Shape shp = t.get_shape();
                    if (shp.size() == 3 && shp[2] == 4) {
                        boxes_tensor = t;
                        have_boxes   = true;
                    }
                    else if (shp.size() == 3) {
                        logits_tensor = t;
                        have_logits   = true;
                    }
                }
                if (!have_boxes || !have_logits) {
                    log<ERROR>("Could not identify RF-DETR box/logit output tensors");
                    return;
                }

                // -------- Postprocess the result (DETR: sigmoid + threshold, no NMS) --------
                const size_t num_queries  = boxes_tensor.get_shape()[1];
                const size_t num_channels = logits_tensor.get_shape()[2];
                // Determine the class-channel offset. RF-DETR exports an extra background channel
                // (num_channels == objects + 1); channel 0 is unused so the offset is 1.
                const int offset =
                    cfg.class_offset >= 0 ? cfg.class_offset : (num_channels == objects.size() + 1 ? 1 : 0);

                const float* box_data   = boxes_tensor.data<float>();
                const float* logit_data = logits_tensor.data<float>();

                std::vector<int> class_ids;
                std::vector<float> class_confidences;
                std::vector<cv::Rect> boxes;

                for (size_t q = 0; q < num_queries; q++) {
                    const float* logits = logit_data + q * num_channels;

                    // Argmax over the class channels (sigmoid is monotonic, so argmax of logits is fine)
                    int best_channel = -1;
                    float best_logit = -std::numeric_limits<float>::infinity();
                    for (size_t c = offset; c < num_channels; c++) {
                        if (logits[c] > best_logit) {
                            best_logit   = logits[c];
                            best_channel = static_cast<int>(c);
                        }
                    }
                    int class_id = best_channel - offset;
                    if (class_id < 0 || class_id >= static_cast<int>(objects.size())) {
                        continue;
                    }

                    float score = 1.0f / (1.0f + std::exp(-best_logit));
                    if (score < cfg.score_threshold || score < objects[class_id].confidence_threshold) {
                        continue;
                    }

                    // Boxes are normalized cxcywh; scale straight to the original image dimensions.
                    const float* b = box_data + q * 4;
                    float cx       = b[0];
                    float cy       = b[1];
                    float bw       = b[2];
                    float bh       = b[3];
                    int left       = static_cast<int>((cx - 0.5f * bw) * width);
                    int top        = static_cast<int>((cy - 0.5f * bh) * height);
                    int box_width  = static_cast<int>(bw * width);
                    int box_height = static_cast<int>(bh * height);

                    boxes.emplace_back(left, top, box_width, box_height);
                    class_ids.push_back(class_id);
                    class_confidences.push_back(score);
                }

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

                for (size_t idx = 0; idx < boxes.size(); idx++) {
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

                        Ball b{};
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
                    log<DEBUG>("RFDETR took: ", duration, "ms");
                    log<DEBUG>("FPS: ", 1000.0 / duration);
                }
            });
    }

}  // namespace module::vision
