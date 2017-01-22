#ifndef MODULE_INPUT_SPINNAKERCAMERA_H
#define MODULE_INPUT_SPINNAKERCAMERA_H

#include <nuclear>
#include <string>
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include "message/input/Image.h"

#include "utility/vision/fourcc.h"


namespace module {
namespace input {

    struct ImageEvent : public Spinnaker::ImageEvent {
        ImageEvent(const std::string& name, const std::string& serialNumber, Spinnaker::CameraPtr&& camera, NUClear::Reactor& reactor, const utility::vision::FOURCC& fourcc) 
            : name(name), serialNumber(serialNumber), camera(std::move(camera)), reactor(reactor), fourcc(fourcc) {}
        ~ImageEvent()
        {
            if (camera)
            {
                camera->EndAcquisition();
                camera->UnregisterEvent(*this);
                camera->DeInit();
            }
        }

        std::string name;
        std::string serialNumber;
        Spinnaker::CameraPtr camera;
        NUClear::Reactor& reactor;
        utility::vision::FOURCC fourcc;

        void OnImageEvent(Spinnaker::ImagePtr image)
        {
            // We have a complete image, emit it.
            if (!image->IsIncomplete())
            {
                auto msg          = std::make_unique<message::input::Image>();
                msg->timestamp    = NUClear::clock::time_point(std::chrono::nanoseconds(image->GetTimeStamp()));
                msg->format       = static_cast<uint32_t>(fourcc);
                msg->dimensions   << image->GetWidth(), image->GetHeight();
                msg->serialNumber = serialNumber;
                msg->camera_id    = 0;
                msg->data.insert(msg->data.end(), static_cast<uint8_t*>(image->GetData()), static_cast<uint8_t*>(image->GetData()) + image->GetBufferSize());

                //NUClear::log("Received complete image with status. Width:", image->GetWidth(), "Height:", image->GetHeight(), "ID:", image->GetID(), "FrameID:", image->GetFrameID());
                //NUClear::log("Pixel Format:", image->GetPixelFormatName(), "Timestamp:", image->GetTimeStamp());
                //NUClear::log("Status:", image->GetImageStatusDescription(image->GetImageStatus()));

                reactor.emit(msg);
            }
        }
    };

    class SpinnakerCamera : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the SpinnakerCamera reactor.
        explicit SpinnakerCamera(std::unique_ptr<NUClear::Environment> environment);

    private:
        Spinnaker::SystemPtr system;
        Spinnaker::CameraList camList;
        std::map<std::string, std::unique_ptr<ImageEvent>> cameras;
    };

}
}

#endif  // MODULE_INPUT_SPINNAKERCAMERA_H
