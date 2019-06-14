#ifndef MODULE_SUPPORT_LOGGING_IMAGECOMPRESSOR_H
#define MODULE_SUPPORT_LOGGING_IMAGECOMPRESSOR_H

#include <turbojpeg.h>
#include <cstdint>
#include <nuclear>
#include "message/input/Image.h"

namespace module {
namespace output {

    class ImageCompressor : public NUClear::Reactor {
    private:
        int quality = 75;

        void compress(const message::input::Image& image, const TJPF& format);

    public:
        /// @brief Called by the powerplant to build and setup the ImageCompressor reactor.
        explicit ImageCompressor(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace output
}  // namespace module

#endif  // MODULE_SUPPORT_LOGGING_IMAGECOMPRESSOR_H
