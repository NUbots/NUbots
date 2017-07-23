#ifndef MODULE_INPUT_CAMERA_SPINNAKERCAMERA_H
#define MODULE_INPUT_CAMERA_SPINNAKERCAMERA_H

#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <nuclear>
#include <string>

#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/motion/KinematicsModels.h"

#include "utility/input/ServoID.h"
#include "utility/vision/fourcc.h"

namespace module {
namespace input {
    struct SpinnakerImageEvent : public Spinnaker::ImageEvent {
        SpinnakerImageEvent(const std::string& name,
                            const std::string& serialNumber,
                            Spinnaker::CameraPtr&& camera,
                            NUClear::Reactor& reactor,
                            const utility::vision::FOURCC& fourcc,
                            int cameraID,
                            bool isLeft)
            : name(name)
            , serialNumber(serialNumber)
            , camera(std::move(camera))
            , reactor(reactor)
            , fourcc(fourcc)
            , cameraID(cameraID)
            , isLeft(isLeft) {}
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
        bool isLeft;
        Eigen::Matrix4d Hcw;
        double ipd;

        void OnImageEvent(Spinnaker::ImagePtr image) {
            // We have a complete image, emit it.
            if (!image->IsIncomplete()) {
                auto msg          = std::make_unique<message::input::Image>();
                msg->timestamp    = NUClear::clock::time_point(std::chrono::nanoseconds(image->GetTimeStamp()));
                msg->format       = static_cast<uint32_t>(fourcc);
                msg->serialNumber = serialNumber;
                msg->camera_id    = cameraID;
                msg->dimensions << image->GetWidth(), image->GetHeight();
                msg->data.insert(msg->data.end(),
                                 static_cast<uint8_t*>(image->GetData()),
                                 static_cast<uint8_t*>(image->GetData()) + image->GetBufferSize());

                // Calculate the world to camera transform
                std::shared_ptr<const message::motion::KinematicsModel> model =
                    NUClear::dsl::store::DataStore<message::motion::KinematicsModel>::get();

                if (model) {
                    ipd = model->head.INTERPUPILLARY_DISTANCE * 0.5f * ((isLeft) ? 1.0f : -1.0f);
                }

                else {
                    ipd = 0.0;
                }

                std::shared_ptr<const message::input::Sensors> sensors =
                    NUClear::dsl::store::DataStore<message::input::Sensors>::get();

                if (sensors) {
                    // Get the transformation from the torso to the camera (or the middle of the face between the eyes
                    // if there
                    // are 2 cameras)
                    auto Htc = sensors->forwardKinematics[utility::input::ServoID::HEAD_PITCH];

                    // Add half the interpupillary distance to the y-coordinate of the translation.
                    Hcw(1, 3) += ipd;

                    // Calculate the world to camera transformation.
                    // Hcw = Hct.inverse() * Htw
                    // Hct is the transform from torso to camera
                    // Htw is the transform from world to torso
                    Hcw = Htc.inverse() * sensors->world;
                }

                else {
                    Hcw.setIdentity();
                }

                msg->Hcw = Hcw;

                reactor.emit(msg);
            }
        }
    };
}  // namespace input
}  // namespace module

#endif  // MODULE_INPUT_CMAERA_SPINNAKERCAMERA_H
