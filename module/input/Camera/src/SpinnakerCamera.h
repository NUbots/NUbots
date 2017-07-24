#ifndef MODULE_INPUT_CAMERA_SPINNAKERCAMERA_H
#define MODULE_INPUT_CAMERA_SPINNAKERCAMERA_H

#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <nuclear>
#include <string>

#include "ImageData.h"

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
                            bool isLeft);

        ~SpinnakerImageEvent();

        void OnImageEvent(Spinnaker::ImagePtr image);

        std::string name;
        std::string serialNumber;
        Spinnaker::CameraPtr camera;
        NUClear::Reactor& reactor;
        utility::vision::FOURCC fourcc;
        int cameraID;
        bool isLeft;
    };

}  // namespace input
}  // namespace module

#endif  // MODULE_INPUT_CMAERA_SPINNAKERCAMERA_H
