#include "SpinnakerCamera.h"

#include "extension/Configuration.h"

namespace module {
namespace input {

    using extension::Configuration;

    SpinnakerCamera::SpinnakerCamera(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)), system(Spinnaker::System::GetInstance()), cameras() {

        on<Configuration>("Cameras/").then([this] (const Configuration& config)
        {
            Spinnaker::CameraList camList = system->GetCameras();
            // Use configuration here from file SpinnakerCamera.yaml
        });
    }
}
}
