#ifndef MODULE_MODULE_TOOLS_HDRCAPTURE_H
#define MODULE_MODULE_TOOLS_HDRCAPTURE_H

#include <nuclear>
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

// clang-format off
extern "C" {
#include <arvconfig.h>
#include <aravis-0.6/arv.h>
}
// clang-format on

#include "extension/Configuration.h"
#include "message/input/Image.h"
#include "utility/input/ServoID.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/vision/Vision.h"

namespace module {
namespace tools {


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

    class HDRCapture : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the HDRCapture reactor.
        explicit HDRCapture(std::unique_ptr<NUClear::Environment> environment);

    private:
        // Aravis Camera details
        void initiateAravisCamera(const ::extension::Configuration& config);
        static void emitAravisImage(ArvStream* stream, CameraContext* context);
        void resetAravisCamera(std::map<std::string, CameraContext>::iterator&,
                               const ::extension::Configuration& config);
        void shutdownAravisCamera();

        std::map<std::string, CameraContext> AravisCameras;

        // Static count of all cameras in the system.
        static uint cameraCount;

        int exposure_min;
        int exposure_max;
        int gain_min;
        int gain_max;

        int exp_brackets;
        int gain_brackets;

        int exposure;
        float gain;
    };

}  // namespace tools
}  // namespace module

#endif  // MODULE_MODULE_TOOLS_HDRCAPTURE_H