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
            // clang-format off
        std::string lvl = config["log_level"].as<std::string>();
        if (lvl == "TRACE")      { this->log_level = NUClear::TRACE; }
        else if (lvl == "DEBUG") { this->log_level = NUClear::DEBUG; }
        else if (lvl == "INFO")  { this->log_level = NUClear::INFO;  }
        else if (lvl == "WARN")  { this->log_level = NUClear::WARN;  }
        else if (lvl == "ERROR") { this->log_level = NUClear::ERROR; }
        else if (lvl == "FATAL") { this->log_level = NUClear::FATAL; }
            // clang-format on

            // Delete all the old engines
            engines.clear();

            for (const auto& camera : config["cameras"].config) {
                std::string name        = camera.first.as<std::string>();
                std::string engine_type = camera["engine"].as<std::string>();
                int concurrent          = camera["concurrent"].as<int>();
                std::string network     = camera["network"].as<std::string>();

                double min_height   = camera["classifier"]["height"][0].as<double>();
                double max_height   = camera["classifier"]["height"][1].as<double>();
                double max_distance = camera["classifier"]["max_distance"].as<double>();
                double tolerance    = camera["classifier"]["intersection_tolerance"].as<double>();

                // Create a network runner for each concurrent system
                auto& runners = engines[name];
                for (int i = 0; i < concurrent; ++i) {
                    runners.emplace_back(engine_type, min_height, max_height, max_distance, tolerance, network);
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
                        Eigen::Affine3d Hcw(image.Hcw);

                        std::exception_ptr eptr;
                        try {

                            // Run the inference
                            auto result = runner(image, Hcw.cast<float>());

                            // Move stuff into the emit message
                            auto msg             = std::make_unique<message::vision::VisualMesh>();
                            msg->timestamp       = image.timestamp;
                            msg->camera_id       = image.id;
                            msg->name            = image.name;
                            msg->Hcw             = image.Hcw;
                            msg->rays            = std::move(result.rays);
                            msg->coordinates     = std::move(result.coordinates);
                            msg->neighbourhood   = std::move(result.neighbourhood);
                            msg->indices         = std::move(result.indices);
                            msg->classifications = std::move(result.classifications);

                            // Emit the inference
                            emit(msg);
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
