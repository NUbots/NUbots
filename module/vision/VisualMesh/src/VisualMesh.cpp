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
                auto& runners = engines[name];
                for (int i = 0; i < concurrent; ++i) {
                    runners.emplace_back(engine_type,
                                         min_height,
                                         max_height,
                                         max_distance,
                                         tolerance,
                                         network,
                                         cache_directory);
                }
            }
        });

        on<Trigger<Image>>().then([this](const Image& image) {
            // Find the engine for this camera
            std::shared_ptr<EngineContext> ctx;

            /* Mutex Scope */ {
                std::lock_guard<std::mutex> lock(engines_mutex);
                auto it = engines.find(image.id);
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

                        // Extract the camera position now that we need it
                        Eigen::Isometry3d Hcw(image.Hcw);

                        // Run the inference
                        auto result = ctx->runner(image, Hcw.cast<float>());

                        if (result.indices.empty()) {
                            log<NUClear::WARN>("Hcw resulted in no mesh points being on-screen.");
                        }
                        else {
                            // Move stuff into the emit message
                            auto msg             = std::make_unique<message::vision::VisualMesh>();
                            msg->timestamp       = image.timestamp;
                            msg->id              = image.id;
                            msg->name            = image.name;
                            msg->Hcw             = image.Hcw;
                            msg->rays            = result.rays;
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
