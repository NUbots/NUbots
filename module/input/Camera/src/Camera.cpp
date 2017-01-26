#include "Camera.h"

namespace module {
namespace input {

    using extension::Configuration;

    Camera::Camera(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("Camera.yaml").then("Camera system configuration", [this] (const Configuration& config) {
            // Use configuration here from file Camera.yaml
        });

        on<Configuration>("Cameras").then("Camera driver loader", [this] (const Configuration& config) {
        	// Monitor camera config directory for files.
        	// Each file MUST define a "driver", we use this driver to load the appropriate handler for the camera.
        	auto driver = config["driver"].as<std::string>();

        	if (dirver == "V4L2")
        	{
		    	initiateV4L2Camera(config);
        	}

        	else if (driver == "Spinnaker")
        	{
		    	initiateSpinnakerCamera(config);
        	}

        	else
        	{
        		log<NUClear::FATAL>("Unsupported camera driver:", driver);
        	}
        });
    }
}
}
