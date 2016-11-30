#ifndef MODULE_INPUT_SPINNAKERCAMERA_H
#define MODULE_INPUT_SPINNAKERCAMERA_H

#include <nuclear>
#include <string>
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include "message/input/Image.h"


namespace module {
namespace input {

    struct ImageEvent : public Spinnaker::ImageEvent {
        ImageEvent(const std::string& serialNumber, Spinnaker::CameraPtr&& camera, NUClear::Reactor& reactor) 
            : serialNumber(serialNumber), camera(std::move(camera)), reactor(reactor) {}
        ~ImageEvent()
        {
            camera->EndAcquisition();
            camera->UnregisterEvent(*this);
            camera->DeInit();
        }

        std::string serialNumber;
        Spinnaker::CameraPtr camera;
        NUClear::Reactor& reactor;

        void OnImageEvent(Spinnaker::ImagePtr image)
        {
            NUClear::log("Recevied image for camera ", serialNumber);

            // Check image retrieval status
            if (!image->IsIncomplete())
            {
                auto timestamp = NUClear::clock::time_point(std::chrono::nanoseconds(image->GetTimeStamp()));
                std::vector<uint8_t> data((uint8_t*)image->GetData(), (uint8_t*)image->GetData() + image->GetBufferSize());

                reactor.emit(std::make_unique<message::input::Image>(serialNumber, image->GetWidth(), image->GetHeight(), timestamp, std::move(data)));
                NUClear::log("Emitting image from camera ", serialNumber);
            }
        }
    };

    class SpinnakerCamera : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the SpinnakerCamera reactor.
        explicit SpinnakerCamera(std::unique_ptr<NUClear::Environment> environment);

        ~SpinnakerCamera()
        {
            cameras.clear();

            if (system)
            {
                system->ReleaseInstance();
            }
        }

    private:
        Spinnaker::SystemPtr system;
        std::map<std::string, std::unique_ptr<ImageEvent>> cameras;
    };

}
}

#endif  // MODULE_INPUT_SPINNAKERCAMERA_H
