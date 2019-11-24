#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_H
#define MODULE_OUTPUT_IMAGECOMPRESSOR_H

#include <turbojpeg.h>

#include <cstdint>
#include <nuclear>

#include "bayer.h"
#include "message/input/Image.h"

namespace module {
namespace output {

    class ImageCompressor : public NUClear::Reactor {
    private:
        // Quality of compressed image
        int quality;
        // Method of which to perform debayering
        DEBAYER_METHOD method;

        void compress(const message::input::Image& image, const TJPF& format);

    public:
        /// @brief Called by the powerplant to build and setup the ImageCompressor reactor.
        explicit ImageCompressor(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace output
}  // namespace module

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_H
