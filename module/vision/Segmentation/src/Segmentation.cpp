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

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/vision/FieldLines.hpp"
#include "message/vision/GreenHorizon.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/math/geometry/ConvexHull.hpp"
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
            cfg.field_line_grid_size    = config["field_line_grid_size"].as<int>();
            cfg.max_horizon_distance    = config["max_horizon_distance"].as<float>();

            // Compile the model and create inference request object
            ov::Core core;

            // Enable model caching if a cache directory is configured. OpenVINO stores the
            // compiled device kernels there so that after the first run the expensive kernel
            // compilation (several seconds on the GPU) is skipped on subsequent startups.
            const std::string cache_dir = config["cache_dir"].as<std::string>();
            if (!cache_dir.empty()) {
                core.set_property(ov::cache_dir(cache_dir));
            }

            // Optionally run inference in FP16. The segmentation network is memory-bandwidth
            // bound on the integrated GPU (many high-resolution feature maps), so halving the
            // activation precision from FP32 to FP16 reduces the memory traffic and can
            // noticeably lower latency with negligible accuracy impact for this task.
            ov::AnyMap compile_config;
            if (config["inference_precision_fp16"].as<bool>()) {
                compile_config[ov::hint::inference_precision.name()] = ov::element::f16;
            }

            compiled_model = core.compile_model(config["model_path"].as<std::string>(),
                                                config["device"].as<std::string>(),
                                                compile_config);
            infer_request  = compiled_model.create_infer_request();

            // Warm up the model. OpenVINO compiles the device-specific kernels and uploads the
            // weights lazily on the first infer() call, which for the GPU can take several
            // seconds. Run one dummy inference here at startup with a zeroed input so that the
            // first real camera frame is not hit with that one-time cost.
            {
                auto input_port = compiled_model.input();
                ov::Shape warmup_shape{1, 3, IMAGE_SIZE, IMAGE_SIZE};
                std::vector<float> warmup_data(3ul * IMAGE_SIZE * IMAGE_SIZE, 0.0f);
                ov::Tensor warmup_tensor(input_port.get_element_type(), warmup_shape, warmup_data.data());
                infer_request.set_input_tensor(warmup_tensor);
                infer_request.infer();
                log<INFO>("Segmentation model warmed up");
            }
        });

        on<Trigger<Image>, Single>().then("Segmentation Main Loop", [this](const Image& img) {
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
                    // Convert to BGR (not RGB) so every format path produces a consistent BGR mat.
                    // The preprocessing below assumes BGR ordering and blobFromImage(..., swapRB=true)
                    // then converts to the RGB the model expects.
                    cv::cvtColor(img_cv, img_cv, cv::COLOR_BayerRG2BGR);
                    break;
                case utility::vision::fourcc("RGBA"):
                    img_cv = cv::Mat(height, width, CV_8UC4, const_cast<uint8_t*>(img.data.data()));
                    // OpenCV expects BGR images
                    cv::cvtColor(img_cv, img_cv, cv::COLOR_RGBA2BGR);
                    break;
                default: log<WARN>("Image format not supported: ", utility::vision::fourcc(img.format)); return;
            }
            // -------- Preprocess the image -------
            cv::Mat resized_img;
            cv::resize(img_cv, resized_img, cv::Size(IMAGE_SIZE, IMAGE_SIZE));

            // Build the CHW blob in one pass: blobFromImage scales by 1/255, swaps BGR->RGB and
            // lays the data out as [1, 3, H, W]. We then apply the per-channel ImageNet mean/std
            // normalisation directly on each (contiguous) colour plane. This replaces the old
            // convertTo + split + per-channel arithmetic + merge round-trip, which allocated
            // several full-image float temporaries every frame.
            cv::Mat blob = cv::dnn::blobFromImage(resized_img,
                                                  1.0 / 255.0,  // scale factor
                                                  cv::Size(IMAGE_SIZE, IMAGE_SIZE),
                                                  cv::Scalar(),
                                                  true,   // Swap BGR -> RGB
                                                  false,  // no crop
                                                  CV_32F);

            // Planes are in RGB order after swapRB. Apply (x - mean) / std as a single fused
            // affine transform per plane: alpha = 1/std, beta = -mean/std.
            // Python uses: mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225] (RGB order).
            {
                const int plane_area   = IMAGE_SIZE * IMAGE_SIZE;
                float* const blob_data = blob.ptr<float>();
                const std::array<float, 3> mean{0.485f, 0.456f, 0.406f};
                const std::array<float, 3> stdv{0.229f, 0.224f, 0.225f};
                for (int c = 0; c < 3; ++c) {
                    cv::Mat plane(IMAGE_SIZE, IMAGE_SIZE, CV_32F, blob_data + c * plane_area);
                    plane.convertTo(plane, CV_32F, 1.0f / stdv[c], -mean[c] / stdv[c]);
                }
            }
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

            // Argmax across the class planes. The output is laid out as [1, C, H, W] so each
            // class plane is contiguous; walk every pixel once and compare the C planes by a
            // simple pointer offset (data[c * plane_area + i]). Building the result at the
            // network's own output size lets us use a flat pointer loop with no bounds checks.
            cv::Mat segmentation_result(out_height, out_width, CV_8UC1);
            {
                const int plane_area = out_height * out_width;
                uchar* seg           = segmentation_result.ptr<uchar>();
                for (int i = 0; i < plane_area; ++i) {
                    int max_class  = 0;
                    float max_prob = data[i];
                    for (int c = 1; c < out_channels; ++c) {
                        const float prob = data[c * plane_area + i];
                        if (prob > max_prob) {
                            max_prob  = prob;
                            max_class = c;
                        }
                    }
                    seg[i] = static_cast<uchar>(max_class);
                }
            }

            // If output dimensions don't match IMAGE_SIZE, resize the segmentation result.
            // Use nearest-neighbour: this is a class-index map, so interpolating between
            // labels (e.g. field=0 and background=2) would fabricate intermediate classes
            // (field line=1).
            if (out_height != IMAGE_SIZE || out_width != IMAGE_SIZE) {
                cv::resize(segmentation_result,
                           segmentation_result,
                           cv::Size(IMAGE_SIZE, IMAGE_SIZE),
                           0,
                           0,
                           cv::INTER_NEAREST);
            }
            // Run the entire field-line CV pipeline (connected components, convex hull,
            // field-line extraction) at the model's native IMAGE_SIZE x IMAGE_SIZE resolution
            // rather than the full camera resolution. This is roughly (width*height)/
            // (IMAGE_SIZE*IMAGE_SIZE) times cheaper. Pixel coordinates are mapped back to
            // full-resolution image space only when unprojecting rays (see pix_to_ray call).
            const double scale_x = static_cast<double>(width) / IMAGE_SIZE;
            const double scale_y = static_cast<double>(height) / IMAGE_SIZE;

            // -------- Create field lines from segmentation result --------
            // Field lines are class index 1 in our segmentation
            const int FIELD_LINE_CLASS = 1;
            const int FIELD_CLASS      = 0;  // Assuming field is class 0

            // Create binary mask for field pixels
            cv::Mat field_mask = (segmentation_result == FIELD_CLASS);

            // Perform connected components analysis
            cv::Mat labels, stats, centroids;
            int num_labels = cv::connectedComponentsWithStats(field_mask, labels, stats, centroids);

            // Build the valid-cluster mask in a single pass. Precompute which label ids clear
            // the minimum-area threshold, then map the entire label image through that lookup
            // once. (The previous per-label `mask |= (labels == i)` did one full-image pass and
            // allocated a temporary mask for every cluster.)
            cv::Mat valid_cluster_mask(IMAGE_SIZE, IMAGE_SIZE, CV_8UC1);
            {
                std::vector<uchar> label_valid(static_cast<size_t>(num_labels), 0);
                for (int i = 1; i < num_labels; ++i) {  // Start from 1 to skip background
                    if (stats.at<int>(i, cv::CC_STAT_AREA) >= cfg.min_cluster_size) {
                        label_valid[i] = 255;
                    }
                }
                const int* lbl = labels.ptr<int>();
                uchar* vcm     = valid_cluster_mask.ptr<uchar>();
                for (int i = 0; i < IMAGE_SIZE * IMAGE_SIZE; ++i) {
                    vcm[i] = label_valid[lbl[i]];
                }
            }
            // Extract boundary contours from the valid field mask. The GreenHorizon should be
            // derived from the observed field boundary, not from an image-space convex hull of
            // the entire field mask. Starting from the actual contours keeps the world-space hull
            // local to visible field edges and avoids fisheye border artefacts dominating it.
            std::vector<std::vector<cv::Point>> field_contours;
            if (cv::countNonZero(valid_cluster_mask) > 0) {
                cv::Mat contour_mask = valid_cluster_mask.clone();
                cv::findContours(contour_mask, field_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            }

            // Helper function to simplify unprojection calls
            auto pix_to_ray = [&](double x, double y) {
                // Normalize the pixel coordinates to the image width normalized dimensions
                Eigen::Vector2d norm_dim = Eigen::Vector2d(img.dimensions.x(), img.dimensions.y()) / img.dimensions.x();
                return utility::vision::unproject(
                    Eigen::Matrix<double, 2, 1>(x / img.dimensions.x(), y / img.dimensions.x()),
                    img.lens,
                    norm_dim);
            };

            // Precompute the camera-to-world transform once per frame. Hcw.inverse() is a
            // full homogeneous-transform inversion; computing it per field-line point (as was
            // previously done inside the lambdas and the distance filter) cost thousands of
            // inversions per frame.
            const Eigen::Isometry3d Hwc = img.Hcw.inverse();

            // Helper function to simplify projecting rays onto the field plane then transforming into world space
            auto ray_to_world_space = [&](const Eigen::Matrix<double, 3, 1>& uFCc) {
                Eigen::Vector3d uFCw = Hwc.rotation() * uFCc;
                Eigen::Vector3d rFWw = uFCw * std::abs(Hwc.translation().z() / uFCw.z()) + Hwc.translation();
                return rFWw;
            };

            // -------- Build the green horizon --------
            // This module replaces the visual mesh and green horizon detector, so we produce the
            // GreenHorizon ourselves. The horizon is the convex hull of the field region on the
            // ground plane in world space. Downstream consumers (e.g. Yolo) use these hull points
            // to test whether a detection lies within the field, so the mesh/class_map fields are
            // intentionally left empty.
            std::vector<Eigen::Vector3d> green_horizon_hull;
            if (!field_contours.empty()) {
                // Project field-boundary contour points onto the ground plane (z = 0) in world
                // space. Discard rays that point at or above the horizon (they never meet the
                // ground) and points beyond the maximum horizon distance: vertices near the visual
                // horizon otherwise project to enormous ground distances that produce a wildly
                // distorted hull.
                std::vector<Eigen::Vector2d> ground_pts;
                for (const auto& contour : field_contours) {
                    ground_pts.reserve(ground_pts.size() + contour.size());
                    for (const auto& point : contour) {
                        const Eigen::Vector3d uFCc = pix_to_ray(point.x * scale_x, point.y * scale_y);
                        const Eigen::Vector3d uFCw = Hwc.rotation() * uFCc;
                        const double t             = -Hwc.translation().z() / uFCw.z();
                        if (!std::isfinite(t) || t <= 0.0) {
                            continue;
                        }
                        const Eigen::Vector3d rFWw = uFCw * t + Hwc.translation();
                        if ((rFWw - Hwc.translation()).norm() > cfg.max_horizon_distance) {
                            continue;
                        }
                        ground_pts.emplace_back(rFWw.x(), rFWw.y());
                    }
                }

                // The image-space boundary is not convex once projected through the fisheye lens
                // onto the ground, so compute the convex hull in the ground plane to get a
                // correctly ordered, convex world-space polygon (Andrew's monotone chain).
                std::vector<Eigen::Vector2d> world_hull;
                if (ground_pts.size() >= 3) {
                    std::sort(ground_pts.begin(), ground_pts.end(), [](const auto& a, const auto& b) {
                        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
                    });
                    const auto cross =
                        [](const Eigen::Vector2d& o, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
                            return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
                        };
                    const size_t n = ground_pts.size();
                    std::vector<Eigen::Vector2d> hull(2 * n);
                    size_t k = 0;
                    // Lower hull
                    for (size_t i = 0; i < n; ++i) {
                        while (k >= 2 && cross(hull[k - 2], hull[k - 1], ground_pts[i]) <= 0) {
                            k--;
                        }
                        hull[k++] = ground_pts[i];
                    }
                    // Upper hull
                    for (size_t i = n - 1, lower = k + 1; i-- > 0;) {
                        while (k >= lower && cross(hull[k - 2], hull[k - 1], ground_pts[i]) <= 0) {
                            k--;
                        }
                        hull[k++] = ground_pts[i];
                    }
                    hull.resize(k - 1);  // last point == first point
                    world_hull = std::move(hull);
                }

                if (world_hull.size() >= 3) {
                    green_horizon_hull.reserve(world_hull.size());
                    auto green_horizon       = std::make_unique<GreenHorizon>();
                    green_horizon->id        = img.id;
                    green_horizon->timestamp = img.timestamp;
                    green_horizon->Hcw       = img.Hcw;
                    green_horizon->horizon.reserve(world_hull.size());
                    for (const auto& p : world_hull) {
                        green_horizon_hull.emplace_back(p.x(), p.y(), 0.0);
                        green_horizon->horizon.emplace_back(p.x(), p.y(), 0.0);
                    }
                    emit(std::move(green_horizon));
                }
            }

            // Create a field lines message
            auto field_lines       = std::make_unique<FieldLines>();
            field_lines->id        = img.id;
            field_lines->timestamp = img.timestamp;
            field_lines->Hcw       = img.Hcw;

            // Spatial downsampling grid: at most one field line point is kept per grid cell.
            // This keeps the point cloud a manageable size for downstream algorithms while
            // maintaining a roughly uniform density along the lines. A cell size <= 1 keeps
            // every point (downsampling disabled).
            const int grid_size = std::max(1, cfg.field_line_grid_size);
            const int cells_x   = (IMAGE_SIZE + grid_size - 1) / grid_size;
            const int cells_y   = (IMAGE_SIZE + grid_size - 1) / grid_size;
            std::vector<bool> cell_occupied(static_cast<size_t>(cells_x) * cells_y, false);

            // Extract field line points from the segmentation result. Filter them by the emitted
            // GreenHorizon polygon in world space rather than by an image-space hull or a simple
            // radial distance threshold.
            for (int y = 0; y < IMAGE_SIZE; y++) {
                const uchar* seg_row = segmentation_result.ptr<uchar>(y);
                for (int x = 0; x < IMAGE_SIZE; x++) {
                    if (seg_row[x] != FIELD_LINE_CLASS) {
                        continue;
                    }

                    const size_t cell_index = static_cast<size_t>(y / grid_size) * cells_x + (x / grid_size);
                    if (cell_occupied[cell_index]) {
                        continue;
                    }

                    // Convert pixel to ray in camera space. Map the 512x512 pixel back to
                    // full-resolution image coordinates so the lens unprojection is correct.
                    Eigen::Vector3d uFCc = pix_to_ray(x * scale_x, y * scale_y);
                    Eigen::Vector3d rFWw = ray_to_world_space(uFCc);

                    // Reject rays that do not intersect the ground plane in front of the camera.
                    if (std::abs(rFWw.z()) > 1e-6) {
                        continue;
                    }

                    if (green_horizon_hull.size() >= 3
                        && !utility::math::geometry::point_in_convex_hull(green_horizon_hull, rFWw)) {
                        continue;
                    }

                    field_lines->points.push_back(uFCc);
                    field_lines->rPWw.push_back(rFWw);
                    cell_occupied[cell_index] = true;
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
        });
    }

}  // namespace module::vision
