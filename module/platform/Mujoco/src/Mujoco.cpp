#include "Mujoco.hpp"

#include "extension/Configuration.hpp"

namespace module::platform {

    using extension::Configuration;

    Mujoco::Mujoco(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Mujoco.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Mujoco.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.world_path  = config["world_path"].as<std::string>();
        });

        on<Startup>().then("Start Mujoco", [this] {
            // Start Mujoco world
            log<NUClear::INFO>("Mujoco started");

            // Load the model
            char error[1000] = "Could not load binary model";
            m                = mj_loadXML(cfg.world_path.c_str(), 0, error, 1000);
            if (!m) {
                mju_error_s("Load model error: %s", error);
            }

            // Make data
            d = mj_makeData(m);

            // Initialize MuJoCo visualization
            // init GLFW
            if (!glfwInit()) {
                mju_error("Could not initialize GLFW");
            }
        });

        // on<Trigger<Sensors>>().then("Update Mujoco", [this](const Sensors& sensors) {
        //     // TODO: Update Mujoco world with sensor data, object positions, etc.
        //     log<NUClear::INFO>("Mujoco updated");
        // });
    }

}  // namespace module::platform
