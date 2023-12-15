/*
 * MIT License
 *
 * Copyright (c) 2018 NUbots
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
#include "VisualMesh.hpp"

#include <Eigen/Geometry>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/vision/VisualMesh.hpp"

namespace module::vision {

    using extension::Configuration;

    using message::input::Image;

    VisualMesh::VisualMesh(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("VisualMesh.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            std::lock_guard<std::mutex> lock(engines_mutex);
            // Delete all the old engines
            engines.clear();

            for (const auto& camera : config["cameras"].config) {
                auto name            = camera.first.as<std::string>();
                auto engine_type     = camera.second["engine"].as<std::string>();
                int concurrent       = camera.second["concurrent"].as<int>();
                auto network         = camera.second["network"].as<std::string>();
                auto cache_directory = camera.second["cache_directory"].as<std::string>();

                auto min_height   = camera.second["classifier"]["height"][0].as<double>();
                auto max_height   = camera.second["classifier"]["height"][1].as<double>();
                auto max_distance = camera.second["classifier"]["max_distance"].as<double>();
                auto tolerance    = camera.second["classifier"]["intersection_tolerance"].as<double>();

                // Create a network runner for each concurrent system
                auto ctx = std::make_shared<EngineContext>();
                for (int i = 0; i < concurrent; ++i) {
                    ctx->runners.emplace_back(std::make_unique<std::mutex>(),
                                              visualmesh::VisualMeshRunner{engine_type,
                                                                           min_height,
                                                                           max_height,
                                                                           max_distance,
                                                                           tolerance,
                                                                           network,
                                                                           cache_directory});
                }
                engines[name] = ctx;
            }
        });

        on<Trigger<Image>>().then([this](const Image& image) {
            // Find the engine for this camera
            std::shared_ptr<EngineContext> ctx;

            /* Mutex Scope */ {
                std::lock_guard<std::mutex> lock(engines_mutex);
                auto it = engines.find(image.name);
                if (it != engines.end()) {
                    ctx = it->second;
                }
            }

            if (ctx != nullptr) {

                // Look through our runners and try to find the first free one
                for (auto& ctx : ctx->runners) {
                    // Attempt to acquire a lock on the mutex, if this succeeds then the context wasn't being used
                    std::unique_lock lock(*ctx.mutex, std::try_to_lock);
                    if (lock) {

                        auto& runner = ctx.runner;

                        // Extract the camera position now that we need it
                        Eigen::Isometry3d Hcw(image.Hcw);

                        // Run the inference
                        auto result = runner(image, Hcw.cast<float>());

                        if (result.indices.empty()) {
                            // The ground is not visible, hence the mesh cannot be drawn
                            log<NUClear::TRACE>("Hcw resulted in no mesh points being on-screen.");
                        }
                        else {
                            // Convert rays to world space
                            Eigen::Matrix<float, 3, Eigen::Dynamic> rPWw(3, result.rays.cols());
                            Eigen::Isometry3f Hwc = Hcw.inverse().cast<float>();

                            // Compute these values once outside the loop
                            Eigen::Vector3f Hwc_translation = Hwc.translation();

                            // Use Eigen's operations to perform the calculations on the entire matrix at once
                            rPWw = (result.rays.array() * Hwc_translation.z()
                                    / result.rays.row(2).replicate(1, result.rays.cols()).array())
                                       .matrix()
                                   + Hwc_translation.replicate(1, result.rays.cols());

                            // Move stuff into the emit message
                            auto msg             = std::make_unique<message::vision::VisualMesh>();
                            msg->timestamp       = image.timestamp;
                            msg->id              = image.id;
                            msg->name            = image.name;
                            msg->Hcw             = image.Hcw;
                            msg->rPWw            = rPWw;
                            msg->uPCw            = result.rays;
                            msg->coordinates     = result.coordinates;
                            msg->neighbourhood   = std::move(result.neighbourhood);
                            msg->indices         = std::move(result.indices);
                            msg->classifications = std::move(result.classifications);
                            msg->class_map       = runner.class_map;

                            if (image.vision_ground_truth.exists) {
                                msg->vision_ground_truth = image.vision_ground_truth;
                            }

                            // Emit the inference
                            emit(msg);

                            // Successful processing
                            ++processed;
                            return;
                        }
                    }
                }
            }
            else {
                log<NUClear::TRACE>("There is no network loaded for", image.name);
            }

            // We failed to process this image
            ++dropped;
        });

        on<Every<1, std::chrono::seconds>>().then("Stats", [this] {
            log<NUClear::DEBUG>(fmt::format("Receiving {}/s, Processing {}/s,  Dropping {}/s ({}%)",
                                            processed + dropped,
                                            processed,
                                            dropped,
                                            100 * double(processed) / double(processed + dropped)));
            processed = 0;
            dropped   = 0;
        });
    }
}  // namespace module::vision
