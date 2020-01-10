#ifndef MODULE_INPUT_CAMERA_TIMESYNC_H
#define MODULE_INPUT_CAMERA_TIMESYNC_H

extern "C" {
#include <aravis-0.8/arv.h>
}

#include "CameraContext.h"

namespace module {
namespace input {
    CameraContext::TimeCorrection sync_clocks(ArvDevice* device);
}  // namespace input
}  // namespace module

#endif  // MODULE_INPUT_CAMERA_TIMESYNC_H
