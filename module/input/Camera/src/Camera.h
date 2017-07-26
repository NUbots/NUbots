#ifndef MODULE_INPUT_CAMERA_H
#define MODULE_INPUT_CAMERA_H

#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>

#include <fcntl.h>
#include <jpeglib.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>

#include <nuclear>

#include "extension/Configuration.h"

#include "message/input/CameraParameters.h"
#include "message/input/Image.h"

#include "utility/support/eigen_armadillo.h"

#include "utility/vision/fourcc.h"

#include "SpinnakerCamera.h"
#include "V4L2Camera.h"

namespace module {
namespace input {

    class Camera : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Camera reactor.
        explicit Camera(std::unique_ptr<NUClear::Environment> environment);

    private:
        // V4L2 Camera details
        V4L2Camera initiateV4L2Camera(const ::extension::Configuration& config);
        void ShutdownV4L2Camera();

        ReactionHandle V4L2FrameRateHandle;
        ReactionHandle V4L2SettingsHandle;

        std::map<std::string, V4L2Camera> V4L2Cameras;


        // Spinnaker Camera details
        void initiateSpinnakerCamera(const ::extension::Configuration& config);
        void resetSpinnakerCamera(std::map<std::string, std::unique_ptr<SpinnakerImageEvent>>::iterator& camera,
                                  const ::extension::Configuration& config);
        void ShutdownSpinnakerCamera();

        Spinnaker::SystemPtr SpinnakerSystem;
        Spinnaker::CameraList SpinnakerCamList;
        module::input::SpinnakerLogCallback SpinnakerLoggingCallback;
        std::map<std::string, std::unique_ptr<SpinnakerImageEvent>> SpinnakerCameras;

        static uint cameraCount;
    };
}  // namespace input
}  // namespace module


#endif  // MODULE_INPUT_CAMERA_H
