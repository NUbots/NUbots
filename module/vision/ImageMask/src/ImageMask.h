#ifndef MODULE_VISION_IMAGEMASK_H
#define MODULE_VISION_IMAGEMASK_H

#include <nuclear>

namespace module {
namespace vision {

    class ImageMask : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the ImageMask reactor.
        explicit ImageMask(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_VISION_IMAGEMASK_H
