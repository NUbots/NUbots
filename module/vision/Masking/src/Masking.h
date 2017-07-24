#ifndef MODULE_VISION_MASKING_H
#define MODULE_VISION_MASKING_H

#include <nuclear>
#include "message/input/CameraParameters.h"
#include "message/vision/ImageMask.h"

namespace module {
namespace vision {

    class Masking : public NUClear::Reactor {
        message::vision::ImageMask createMaskForCamera(const message::input::CameraParameters& cam, bool& success);
        std::string image_extension;

    public:
        /// @brief Called by the powerplant to build and setup the ImageMask reactor.
        explicit Masking(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_MASKING_H
