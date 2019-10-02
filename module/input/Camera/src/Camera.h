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
extern "C" {
#include <aravis-0.6/arv.h>
}
// clang-format on

#include "extension/Configuration.h"

#include "message/input/Image.h"

#include "utility/input/ServoID.h"

#include "utility/support/eigen_armadillo.h"

#include "utility/vision/Vision.h"

#include "V4L2Camera.h"

namespace module {
namespace input {

    // Camera contextual information for Aravis new-buffer callback function.
    typedef struct {
        uint32_t fourcc;
        std::string deviceID;
        uint cameraID;
        bool isLeft;
        message::input::Image::Lens lens;
        ArvCamera* camera;
        ArvStream* stream;
        NUClear::Reactor& reactor;
    } CameraContext;

    class Camera : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Camera reactor.
        explicit Camera(std::unique_ptr<NUClear::Environment> environment);

    private:
        bool dumpImages;

        // V4L2 Camera details
        V4L2Camera initiateV4L2Camera(const ::extension::Configuration& config);
        void ShutdownV4L2Camera();

        ReactionHandle V4L2FrameRateHandle;
        ReactionHandle V4L2SettingsHandle;

        std::map<std::string, V4L2Camera> V4L2Cameras;


        // Aravis Camera details
        void initiateAravisCamera(const ::extension::Configuration& config);
        static void EmitAravisImage(ArvStream* stream, CameraContext* context);
        void resetAravisCamera(std::map<std::string, CameraContext>::iterator&,
                               const ::extension::Configuration& config);
        void ShutdownAravisCamera();

        std::map<std::string, CameraContext> AravisCameras;

        // Static count of all cameras in the system.
        static uint cameraCount;
    };
}  // namespace input
}  // namespace module


#endif  // MODULE_INPUT_CAMERA_H
