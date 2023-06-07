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
            // Check we have a network for this camera
            auto it = engines.find(image.name);
            if (it != engines.end()) {
                auto& engine = it->second;

                // Look through our engines and try to find the first free one
                for (auto& runner : engine) {
                    // We swap in true to the atomic and if we got false back then it wasn't active previously
                    if (!runner.active->exchange(true)) {

                        // Extract the camera position now that we need it
                        Eigen::Isometry3d Hcw(image.Hcw);

                        std::exception_ptr eptr;
                        try {

                            // Run the inference
                            auto result = runner(image, Hcw.cast<float>());

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
                                msg->rays            = result.rays.cast<double>();
                                msg->coordinates     = result.coordinates.cast<double>();
                                msg->neighbourhood   = std::move(result.neighbourhood);
                                msg->indices         = std::move(result.indices);
                                msg->classifications = std::move(result.classifications.cast<double>());
                                msg->class_map       = runner.class_map;

                                if (image.vision_ground_truth.exists) {
                                    msg->vision_ground_truth = image.vision_ground_truth;
                                }

                                // Emit the inference
                                emit(msg);
                            }
                        }
                        catch (...) {
                            eptr = std::current_exception();
                        }

                        // This sets the atomic integer back to false so another thread can use this engine
                        runner.active->store(false);

                        if (eptr) {
                            // Exception :(
                            std::rethrow_exception(eptr);
                        }

                        // Finish processing
                        break;
                    }
                }
            }
            else {
                log<NUClear::TRACE>("There is no network loaded for", image.name);
            }
        });
    }
}  // namespace module::vision
