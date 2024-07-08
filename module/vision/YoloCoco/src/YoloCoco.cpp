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
#include "YoloCoco.hpp"

#include <chrono>
#include <getopt.h>
#include <iostream>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/vision/BoundingBoxes.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/Vision.hpp"
#include "utility/vision/fourcc.hpp"
#include "utility/vision/projection.hpp"

namespace module::vision {

    using extension::Configuration;

    using message::input::Image;
    using message::vision::BoundingBox;
    using message::vision::BoundingBoxes;

    using utility::support::Expression;
    using utility::vision::unproject;

    YoloCoco::YoloCoco(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("YoloCoco.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Yolo.yaml
            log_level               = config["log_level"].as<NUClear::LogLevel>();
            cfg.nms_threshold       = config["nms_threshold"].as<double>();
            cfg.nms_score_threshold = config["nms_score_threshold"].as<double>();

            // Compile the model and create inference request object
            compiled_model =
                ov::Core().compile_model(config["model_path"].as<std::string>(), config["device"].as<std::string>());
            infer_request = compiled_model.create_infer_request();
        });

        on<Trigger<Image>, Single>().then("Yolo Main Loop", [this](const Image& img) {
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
                default:
                    log<NUClear::WARN>("Image format not supported: ", utility::vision::fourcc(img.format));
                    return;
            }

            // -------- Preprocess the image -------
            int max               = MAX(width, height);
            cv::Mat letterbox_img = cv::Mat::zeros(max, max, CV_8UC3);
            img_cv.copyTo(letterbox_img(cv::Rect(0, 0, width, height)));
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
            auto bounding_boxes       = std::make_unique<BoundingBoxes>();
            bounding_boxes->id        = img.id;
            bounding_boxes->timestamp = img.timestamp;
            bounding_boxes->Hcw       = img.Hcw;

            // Helper function to simplify unprojection calls
            auto pix_to_ray = [&](double x, double y) {
                // Normalize the pixel coordinates to the image width normalized dimensions
                Eigen::Vector2d norm_dim = Eigen::Vector2d(img.dimensions.x(), img.dimensions.y()) / img.dimensions.x();
                return unproject(Eigen::Matrix<double, 2, 1>(x / img.dimensions.x(), y / img.dimensions.x()),
                                 img.lens,
                                 norm_dim);
            };

            for (size_t i = 0; i < indices.size(); i++) {
                // Get the index of the detected object from list of indices
                int idx          = indices[i];
                auto bbox        = std::make_unique<BoundingBox>();
                bbox->name       = objects[class_ids[idx]].name;
                bbox->confidence = class_confidences[idx];
                bbox->colour     = objects[class_ids[idx]].colour;
                // Convert the bounding box points to unit vectors (rays) in the camera {c} space
                bbox->corners.push_back(pix_to_ray(boxes[idx].x, boxes[idx].y));
                bbox->corners.push_back(pix_to_ray(boxes[idx].x + boxes[idx].width, boxes[idx].y));
                bbox->corners.push_back(pix_to_ray(boxes[idx].x + boxes[idx].width, boxes[idx].y + boxes[idx].height));
                bbox->corners.push_back(pix_to_ray(boxes[idx].x, boxes[idx].y + boxes[idx].height));
                bounding_boxes->bounding_boxes.push_back(*bbox);
            }

            emit(std::move(bounding_boxes));

            // -------- Benchmark --------
            if (log_level <= NUClear::DEBUG) {
                auto end      = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                log<NUClear::DEBUG>("Detected ", indices.size(), " objects");
                log<NUClear::DEBUG>("Yolo took: ", duration, "ms");
                log<NUClear::DEBUG>("FPS: ", 1000.0 / duration);
            }
        });
    }

}  // namespace module::vision
