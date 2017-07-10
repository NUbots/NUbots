#ifndef MODULE_INPUT_CAMERA_SPINNAKERCAMERA_H
#define MODULE_INPUT_CAMERA_SPINNAKERCAMERA_H

#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>
#include <string>

#include "message/input/Image.h"

#include "utility/vision/fourcc.h"

namespace module {
namespace input {
    struct SpinnakerImageEvent : public Spinnaker::ImageEvent {
        SpinnakerImageEvent(const std::string& name,
                            const std::string& serialNumber,
                            Spinnaker::CameraPtr&& camera,
                            NUClear::Reactor& reactor,
                            const utility::vision::FOURCC& fourcc,
                            int cameraID)
            : name(name)
            , serialNumber(serialNumber)
            , camera(std::move(camera))
            , reactor(reactor)
            , fourcc(fourcc)
            , cameraID(cameraID) {}
        ~SpinnakerImageEvent() {
            if (camera) {
                if (camera->IsStreaming()) {
                    camera->EndAcquisition();
                }

                camera->UnregisterEvent(*this);
                camera->DeInit();
            }
        }

        std::string name;
        std::string serialNumber;
        Spinnaker::CameraPtr camera;
        NUClear::Reactor& reactor;
        utility::vision::FOURCC fourcc;
        int cameraID;

        void OnImageEvent(Spinnaker::ImagePtr image) {
            // We have a complete image, emit it.
            if (!image->IsIncomplete()) {
                auto msg       = std::make_unique<message::input::Image>();
                msg->timestamp = NUClear::clock::time_point(std::chrono::nanoseconds(image->GetTimeStamp()));
                msg->format    = static_cast<uint32_t>(fourcc);
                msg->dimensions << image->GetWidth(), image->GetHeight();
                msg->serialNumber = serialNumber;
                msg->camera_id    = cameraID;
                msg->data.insert(msg->data.end(),
                                 static_cast<uint8_t*>(image->GetData()),
                                 static_cast<uint8_t*>(image->GetData()) + image->GetBufferSize());

                reactor.emit(msg);
            }

            image->Release();
        }
    };
}
}

#endif  // MODULE_INPUT_CMAERA_SPINNAKERCAMERA_H
