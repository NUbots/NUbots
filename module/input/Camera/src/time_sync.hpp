#ifndef MODULE_INPUT_CAMERA_TIMESYNC_HPP
#define MODULE_INPUT_CAMERA_TIMESYNC_HPP

extern "C" {
#include <aravis-0.8/arv.h>
}

#include "CameraContext.hpp"

namespace module::input {
    CameraContext::TimeCorrection sync_clocks(ArvDevice* device);
}  // namespace module::input

#endif  // MODULE_INPUT_CAMERA_TIMESYNC_HPP
