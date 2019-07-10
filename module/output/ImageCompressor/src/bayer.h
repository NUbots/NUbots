#ifndef MODULE_OUTPUT_BAYER_H
#define MODULE_OUTPUT_BAYER_H

#include "message/input/Image.h"
namespace module {
namespace output {
    enum DEBAYER_METHOD { SIMPLE = 0, BILINEAR = 1, HQLINEAR = 2, EDGESENSE = 3 };

    message::input::Image debayer(const message::input::Image& image, DEBAYER_METHOD method);
}  // namespace output
}  // namespace module

#endif  // MODULE_OUTPUT_BAYER_H
