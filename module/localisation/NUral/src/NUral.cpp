#include "NUral.hpp"

#include "extension/Configuration.hpp"

namespace module::localisation {

    using extension::Configuration;
    using message::platform::RawSensors;

    NUral::NUral(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("NUral.yaml").then([this](const Configuration& config) {
            // Use configuration here from file NUral.yaml
            this->log_level  = config["log_level"].as<NUClear::LogLevel>();
            this->model_path = config["model_path"].as<std::string>();

            // Compile the model and create inference request object
            try {
                compiled_model = core.compile_model(model_path, "CPU");
                infer_request  = compiled_model.create_infer_request();
                log<INFO>("Model loaded successfully");
            }
            catch (const std::exception& e) {
                log<ERROR>("Failed to load NUral model: ", e.what());
                throw;
            }
        });
    }

}  // namespace module::localisation
