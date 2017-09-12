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

#include <nuclear>

// clang-format off
#include <arvconfig.h>
#include <arv.h>
// clang-format on

#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>

#include "extension/Configuration.h"

#include "message/input/CameraParameters.h"
#include "message/input/Image.h"

#include "utility/support/eigen_armadillo.h"

#include "utility/vision/Vision.h"

#include "SpinnakerCamera.h"
#include "V4L2Camera.h"

namespace module {
namespace input {

    typedef struct {
        uint32_t fourcc;
        std::string deviceID;
        uint cameraID;
        bool isLeft;
        std::tuple<uint, std::unique_ptr<ArvCamera>, std::unique_ptr<ArvStream>>& camera;
    } ImageContext;

    class Camera : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Camera reactor.
        explicit Camera(std::unique_ptr<NUClear::Environment> environment);

    private:
        void setExposure(Spinnaker::GenApi::INodeMap& nodeMap, double exposure);
        void setGain(Spinnaker::GenApi::INodeMap& nodeMap, double gain);
        void resetParameters(Spinnaker::GenApi::INodeMap& nodeMap);
        bool resetUSBDevice(int bus, int device);

        bool dumpImages;

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


        // Aravis Camera details
        void initiateAravisCamera(const ::extension::Configuration& config);
        void EmitAravisImage(ArvStream* stream, ImageContext* context);
        void resetAravisCamera(
            std::map<std::string, std::tuple<uint, std::unique_ptr<ArvCamera>, std::unique_ptr<ArvStream>>>::iterator&
                camera,
            const ::extension::Configuration& config);
        void ShutdownAravisCamera();

        std::map<std::string, std::tuple<uint, std::unique_ptr<ArvCamera>, std::unique_ptr<ArvStream>>> AravisCameras;

        static uint cameraCount;
    };
}  // namespace input
}  // namespace module


#endif  // MODULE_INPUT_CAMERA_H
