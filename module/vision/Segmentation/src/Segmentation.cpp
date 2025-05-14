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
#include "Segmentation.hpp"

#include <chrono>
#include <iostream>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/vision/FieldLines.hpp"
#include "message/vision/GreenHorizon.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/Vision.hpp"
#include "utility/vision/fourcc.hpp"
#include "utility/vision/projection.hpp"

namespace module::vision {

    using extension::Configuration;

    using message::input::Image;
    using message::vision::FieldLines;
    using message::vision::GreenHorizon;

    Segmentation::Segmentation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Segmentation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Segmentation.yaml
            log_level       = config["log_level"].as<NUClear::LogLevel>();
            cfg.num_classes = config["num_classes"].as<int>();

            // New parameters
            cfg.filter_by_distance      = config["filter_by_distance"].as<bool>();
            cfg.max_field_line_distance = config["max_field_line_distance"].as<float>();
            cfg.min_cluster_size        = config["min_cluster_size"].as<int>();

            // Compile the model and create inference request object
            compiled_model =
                ov::Core().compile_model(config["model_path"].as<std::string>(), config["device"].as<std::string>());
            infer_request = compiled_model.create_infer_request();
        });

        on<Trigger<Image>, Single>().then("Segmentation Main Loop", [this](const Image& img) {
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
                    // OpenCV expects BGRA images, so the following is actually converting to RGB...
                    cv::cvtColor(img_cv, img_cv, cv::COLOR_RGBA2BGR);
                    break;
                default: log<WARN>("Image format not supported: ", utility::vision::fourcc(img.format)); return;
            }

            // -------- Preprocess the image -------
            cv::Mat resized_img;
            cv::resize(img_cv, resized_img, cv::Size(IMAGE_SIZE, IMAGE_SIZE));

            // Convert to float and normalize using ImageNet mean and std
            cv::Mat normalized_img;
            resized_img.convertTo(normalized_img, CV_32F, 1.0 / 255.0);

            // Apply normalization with same values as in Python code
            // Python uses: mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
            std::vector<cv::Mat> channels(3);
            cv::split(normalized_img, channels);

            // Normalize each channel
            channels[0] = (channels[0] - 0.485) / 0.229;  // R channel
            channels[1] = (channels[1] - 0.456) / 0.224;  // G channel
            channels[2] = (channels[2] - 0.406) / 0.225;  // B channel

            cv::merge(channels, normalized_img);

            // Convert to blob format
            cv::Mat blob = cv::dnn::blobFromImage(normalized_img,
                                                  1.0,  // scale factor
                                                  cv::Size(IMAGE_SIZE, IMAGE_SIZE),
                                                  cv::Scalar(0, 0, 0),  // no mean subtraction (already done)
                                                  false,                // don't swap R and B channels
                                                  CV_32F);              // output depth

            // -------- Feed the blob into the input node of the Model -------
            // Get input port for model with one input
            auto input_port = compiled_model.input();

            // Create tensor with explicit static dimensions instead of using dynamic shape
            // For image segmentation, the shape is typically [batch_size, channels, height, width]
            // Set batch_size=1, channels=3, and use our IMAGE_SIZE for height and width
            ov::Shape static_shape = {1, 3, IMAGE_SIZE, IMAGE_SIZE};
            ov::Tensor input_tensor(input_port.get_element_type(), static_shape, blob.ptr(0));

            // Set input tensor for model with one input
            infer_request.set_input_tensor(input_tensor);

            // -------- Perform Inference --------
            infer_request.infer();
            auto output = infer_request.get_output_tensor(0);

            // -------- Postprocess the result --------
            // The output is a [1, num_classes, height, width] tensor
            cv::Mat segmentation_result(IMAGE_SIZE, IMAGE_SIZE, CV_8UC1);

            // Get raw output data and tensor information
            float* data = output.data<float>();

            // For dynamic shape models, we need to get the actual shape after inference
            const auto& output_shape = output.get_shape();
            if (output_shape.size() != 4) {
                log<WARN>("Unexpected output shape from segmentation model");
                return;
            }

            // We know the model should output [batch_size, num_classes, height, width]
            // For most image models, batch_size is 1, height/width should match our IMAGE_SIZE
            int batch_size   = output_shape[0];
            int out_channels = output_shape[1];  // Should match num_classes
            int out_height   = output_shape[2];  // Should match IMAGE_SIZE
            int out_width    = output_shape[3];  // Should match IMAGE_SIZE

            log<DEBUG>("Output shape: [", batch_size, ", ", out_channels, ", ", out_height, ", ", out_width, "]");

            // Verify we have the expected dimensions
            if (out_height != IMAGE_SIZE || out_width != IMAGE_SIZE) {
                log<WARN>("Output dimensions don't match expected size: ",
                          out_height,
                          "x",
                          out_width,
                          " (expected ",
                          IMAGE_SIZE,
                          "x",
                          IMAGE_SIZE,
                          ")");
            }

            // Find the class with highest probability for each pixel
            for (int y = 0; y < out_height; y++) {
                for (int x = 0; x < out_width; x++) {
                    // Find the class with maximum probability for this pixel
                    int max_class  = 0;
                    float max_prob = -std::numeric_limits<float>::max();

                    for (int c = 0; c < out_channels; c++) {
                        // Calculate the index in the flattened tensor: [b,c,y,x]
                        // For batch_size=1: index = c*H*W + y*W + x
                        float prob = data[c * out_height * out_width + y * out_width + x];
                        if (prob > max_prob) {
                            max_prob  = prob;
                            max_class = c;
                        }
                    }

                    // Make sure we're not writing outside our segmentation result
                    if (y < IMAGE_SIZE && x < IMAGE_SIZE) {
                        segmentation_result.at<uchar>(y, x) = static_cast<uchar>(max_class);
                    }
                }
            }

            // If output dimensions don't match IMAGE_SIZE, resize the segmentation result
            if (out_height != IMAGE_SIZE || out_width != IMAGE_SIZE) {
                cv::resize(segmentation_result, segmentation_result, cv::Size(IMAGE_SIZE, IMAGE_SIZE));
            }

            // -------- Create visualization image --------
            cv::Mat visualization(IMAGE_SIZE, IMAGE_SIZE, CV_8UC3);

            // Map each class to its color
            for (int y = 0; y < IMAGE_SIZE; y++) {
                for (int x = 0; x < IMAGE_SIZE; x++) {
                    int class_id = segmentation_result.at<uchar>(y, x);
                    if (static_cast<size_t>(class_id) < classes.size()) {
                        visualization.at<cv::Vec3b>(y, x) = classes[class_id].colour;
                    }
                }
            }

            // Resize back to original dimensions
            cv::resize(visualization, visualization, cv::Size(width, height));
            cv::resize(segmentation_result, segmentation_result, cv::Size(width, height));

            // -------- Create field lines from segmentation result --------
            // Field lines are class index 1 in our segmentation
            const int FIELD_LINE_CLASS = 1;
            const int FIELD_CLASS      = 0;  // Assuming field is class 0

            // Create binary mask for field pixels
            cv::Mat field_mask = (segmentation_result == FIELD_CLASS);
            field_mask.convertTo(field_mask, CV_8UC1);

            // Perform connected components analysis
            cv::Mat labels, stats, centroids;
            int num_labels = cv::connectedComponentsWithStats(field_mask, labels, stats, centroids);

            // Create a mask for valid clusters
            cv::Mat valid_cluster_mask = cv::Mat::zeros(height, width, CV_8UC1);
            for (int i = 1; i < num_labels; i++) {  // Start from 1 to skip background
                if (stats.at<int>(i, cv::CC_STAT_AREA) >= cfg.min_cluster_size) {
                    // Add this cluster to the valid mask
                    valid_cluster_mask |= (labels == i);
                }
            }

            // First, collect all field pixels for convex hull from valid clusters only
            std::vector<cv::Point> field_points;
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    if (valid_cluster_mask.at<uchar>(y, x) > 0) {
                        field_points.push_back(cv::Point(x, y));
                    }
                }
            }

            // Create convex hull if we have field points
            std::vector<cv::Point> hull_points;
            if (!field_points.empty()) {
                cv::convexHull(field_points, hull_points);
            }

            // Helper function to check if a point is inside the convex hull
            auto is_point_in_hull = [&hull_points](const cv::Point& pt) -> bool {
                if (hull_points.empty())
                    return false;
                return cv::pointPolygonTest(hull_points, pt, false) >= 0;
            };

            // Helper function to simplify unprojection calls
            auto pix_to_ray = [&](double x, double y) {
                // Normalize the pixel coordinates to the image width normalized dimensions
                Eigen::Vector2d norm_dim = Eigen::Vector2d(img.dimensions.x(), img.dimensions.y()) / img.dimensions.x();
                return utility::vision::unproject(
                    Eigen::Matrix<double, 2, 1>(x / img.dimensions.x(), y / img.dimensions.x()),
                    img.lens,
                    norm_dim);
            };

            // Helper function to simplify projecting rays onto the field plane then transforming into world space
            auto ray_to_world_space = [&](const Eigen::Matrix<double, 3, 1>& uFCc) {
                const Eigen::Isometry3d& Hwc = img.Hcw.inverse();
                Eigen::Vector3d uFCw         = Hwc.rotation() * uFCc;
                Eigen::Vector3d rFCw         = uFCw * std::abs(Hwc.translation().z() / uFCw.z()) + Hwc.translation();
                return rFCw;
            };

            // Create a field lines message
            auto field_lines       = std::make_unique<FieldLines>();
            field_lines->id        = img.id;
            field_lines->timestamp = img.timestamp;
            field_lines->Hcw       = img.Hcw;

            // Extract field line points from the segmentation result
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    int class_id = segmentation_result.at<uchar>(y, x);
                    if (class_id == FIELD_LINE_CLASS) {
                        // Check if the point is within the field convex hull
                        if (!is_point_in_hull(cv::Point(x, y))) {
                            continue;  // Skip points outside the hull
                        }

                        // Convert pixel to ray in camera space
                        Eigen::Vector3d uFCc = pix_to_ray(x, y);

                        // Project ray onto field plane in world space
                        Eigen::Vector3d rFWw = ray_to_world_space(uFCc);

                        // Check if the point is within the maximum distance
                        bool is_valid = true;
                        if (cfg.filter_by_distance) {
                            // Calculate distance from camera origin to the field point
                            const Eigen::Isometry3d& Hwc = img.Hcw.inverse();
                            double distance              = (rFWw - Hwc.translation()).norm();

                            if (distance > cfg.max_field_line_distance || rFWw.z() != 0) {
                                is_valid = false;
                            }
                        }

                        // Only add the ray if it's valid
                        if (is_valid) {
                            field_lines->points.push_back(uFCc);
                            field_lines->rPWw.push_back(rFWw);
                        }
                    }
                }
            }

            // Only emit field lines if we found any
            if (!field_lines->points.empty()) {
                log<DEBUG>("Found ", field_lines->points.size(), " field line points");
                //  Print first world point rPWw
                log<DEBUG>("First world point rPWw: ", field_lines->rPWw[0]);
                emit(field_lines);
            }
            else {
                log<DEBUG>("No field lines found in segmentation");
            }

            // -------- Create and emit visualization image --------
            auto visualization_image            = std::make_unique<message::input::Image>();
            visualization_image->timestamp      = NUClear::clock::now();
            visualization_image->dimensions.x() = width;
            visualization_image->dimensions.y() = height;
            visualization_image->Hcw            = img.Hcw;  // Keep the same camera transform

            // Create a properly sized data vector first
            visualization_image->data.resize(width * height * 3);

            // Fill the data vector directly without temporary storage
            // This approach minimizes memory operations
            size_t index = 0;
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    cv::Vec3b color                    = visualization.at<cv::Vec3b>(y, x);
                    visualization_image->data[index++] = color[2];  // R (OpenCV uses BGR)
                    visualization_image->data[index++] = color[1];  // G
                    visualization_image->data[index++] = color[0];  // B
                }
            }

            visualization_image->format = utility::vision::fourcc("RGB3");
            visualization_image->name   = "Segmentation";

            // Make sure to use std::move when emitting the unique_ptr
            // emit(std::move(visualization_image));

            if (log_level <= DEBUG) {
                // -------- Benchmark --------
                auto end      = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                log<DEBUG>("Segmentation took: ", duration, "ms");
                log<DEBUG>("FPS: ", 1000.0 / duration);

                // -------- Save images --------
                // Save the colored visualization
                cv::imwrite("recordings/segmentation_visualization.png", visualization);

                // Create a more informative colored version of the class map for debugging
                cv::Mat colored_segmentation_map(height, width, CV_8UC3);
                for (int y = 0; y < height; y++) {
                    for (int x = 0; x < width; x++) {
                        int class_id = segmentation_result.at<uchar>(y, x);
                        if (static_cast<size_t>(class_id) < classes.size()) {
                            colored_segmentation_map.at<cv::Vec3b>(y, x) = classes[class_id].colour;
                        }
                        else {
                            // Use a distinctive color for unexpected class ids
                            colored_segmentation_map.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);  // Red
                        }
                    }
                }
                cv::imwrite("recordings/segmentation_colored_map.png", colored_segmentation_map);

                // Save the raw class indices as well (but scale to make it more visible)
                cv::Mat scaled_class_map;
                // Scale to 0-255 range to make it more visible
                segmentation_result.convertTo(scaled_class_map, CV_8UC1, 255.0 / (cfg.num_classes - 1));
                cv::imwrite("recordings/segmentation_class_indices.png", scaled_class_map);

                // Save a side-by-side comparison of original and segmentation
                cv::Mat comparison(height, width * 2, CV_8UC3);
                // Resize the original image to the output dimensions
                cv::Mat resized_original;
                cv::resize(img_cv, resized_original, cv::Size(width, height));
                // Copy images side by side
                resized_original.copyTo(comparison(cv::Rect(0, 0, width, height)));
                colored_segmentation_map.copyTo(comparison(cv::Rect(width, 0, width, height)));
                cv::imwrite("recordings/segmentation_comparison.png", comparison);

                // Add visualization of the convex hull
                if (!hull_points.empty()) {
                    // Draw the convex hull on the visualization
                    for (size_t i = 0; i < hull_points.size(); i++) {
                        cv::line(visualization,
                                 hull_points[i],
                                 hull_points[(i + 1) % hull_points.size()],
                                 cv::Scalar(0, 255, 0),  // Green color
                                 2);                     // Line thickness
                    }
                }
                cv::imwrite("recordings/segmentation_with_hull.png", visualization);

                // Add visualization of clusters and hull
                cv::Mat cluster_visualization = visualization.clone();

                // Draw different clusters in random colors
                cv::RNG rng(12345);  // Random number generator with seed
                for (int i = 1; i < num_labels; i++) {
                    if (stats.at<int>(i, cv::CC_STAT_AREA) >= cfg.min_cluster_size) {
                        cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
                        cv::Mat cluster_mask = (labels == i);
                        cluster_visualization.setTo(color, cluster_mask);
                    }
                }

                // Draw the convex hull on top
                if (!hull_points.empty()) {
                    for (size_t i = 0; i < hull_points.size(); i++) {
                        cv::line(cluster_visualization,
                                 hull_points[i],
                                 hull_points[(i + 1) % hull_points.size()],
                                 cv::Scalar(0, 255, 0),  // Green color
                                 2);                     // Line thickness
                    }
                }

                cv::imwrite("recordings/segmentation_clusters.png", cluster_visualization);
            }
        });
    }

}  // namespace module::vision
