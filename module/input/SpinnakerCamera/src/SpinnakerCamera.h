#ifndef MODULE_INPUT_SPINNAKERCAMERA_H
#define MODULE_INPUT_SPINNAKERCAMERA_H

#include <nuclear>
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>


namespace module {
namespace input {

    class SpinnakerCamera : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the SpinnakerCamera reactor.
        explicit SpinnakerCamera(std::unique_ptr<NUClear::Environment> environment);

        ~SpinnakerCamera()
        {
            if (system)
            {
                system->ReleaseInstance();
            }
        }

    private:
        Spinnaker::SystemPtr system;
        std::vector<Spinnaker::CameraPtr> cameras;
    };

}
}

#endif  // MODULE_INPUT_SPINNAKERCAMERA_H
