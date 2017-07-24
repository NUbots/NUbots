#ifndef MODULE_VISION_IMAGEMASK_H
#define MODULE_VISION_IMAGEMASK_H

#include <nuclear>
#include "message/input/CameraParameters.h"
#include "message/vision/ImageMask.h"

namespace module {
namespace vision {

    class ImageMask : public NUClear::Reactor {
        const message::vision::ImageMask& createMaskFromFile(const message::input::CameraParameters& cam);
        std::string image_extension;

    public:
        /// @brief Called by the powerplant to build and setup the ImageMask reactor.
        explicit ImageMask(std::unique_ptr<NUClear::Environment> environment, bool& success);
    };

}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_IMAGEMASK_H
