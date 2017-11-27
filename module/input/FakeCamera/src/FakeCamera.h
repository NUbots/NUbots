#ifndef MODULE_INPUT_FAKECAMERA_H
#define MODULE_INPUT_FAKECAMERA_H

#include <string>

#include <nuclear>

#include "utility/vision/Vision.h"

namespace module {
namespace input {

    class FakeCamera : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the FakeCamera reactor.
        explicit FakeCamera(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::string image_folder, image_prefix, image_ext;
        size_t num_images, count;
        utility::vision::FOURCC image_format;
    };

}  // namespace input
}  // namespace module

#endif  // MODULE_INPUT_FAKECAMERA_H
