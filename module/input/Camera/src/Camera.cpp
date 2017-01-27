#include "Camera.h"

namespace module {
namespace input {

    uint Camera::cameraCount = 0;
    
    using extension::Configuration;

    Camera::Camera(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , V4L2FrameRateHandle()
    , V4L2SettingsHandle()
    , V4L2Cameras()
    , SpinnakerSystem()
    , SpinnakerCamList()
    , SpinnakerCameras() {

        on<Configuration>("Camera.yaml").then("Camera system configuration", [this] (const Configuration& /*config*/) {
            // Use configuration here from file Camera.yaml
        });

        on<Configuration>("Cameras").then("Camera driver loader", [this] (const Configuration& config) {
        	// Monitor camera config directory for files.
        	// Each file MUST define a "driver", we use this driver to load the appropriate handler for the camera.
        	auto driver = config["driver"].as<std::string>();

        	if (driver == "V4L2")
        	{
                auto cam = V4L2Cameras.find(config["deviceID"].as<std::string>());

                if (cam != V4L2Cameras.end())
                {
                    V4L2Cameras.insert(std::make_pair(config["deviceID"].as<std::string>(), std::move(initiateV4L2Camera(config))));
                    cameraCount++;
                }

                else
                {
                    cam->second.setConfig(config);
                }
        	}

        	else if (driver == "Spinnaker")
        	{
                auto cam = SpinnakerCameras.find(config["deviceID"].as<std::string>());

                if (cam != SpinnakerCameras.end())
                {
                    initiateSpinnakerCamera(config);
                    cameraCount++;
                }

                else
                {
                    resetSpinnakerCamera(cam, config);
                }
        	}

        	else
        	{
        		log<NUClear::FATAL>("Unsupported camera driver:", driver);
        	}
        });

        // When we shutdown, we must tell our camera class to close (stop streaming)
        on<Shutdown>().then([this] {
            ShutdownV4L2Camera();
            
            ShutdownSpinnakerCamera();
        });

    }
}
}
