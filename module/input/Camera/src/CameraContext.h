#ifndef MODULE_INPUT_CAMERA_CONTEXT_H
#define MODULE_INPUT_CAMERA_CONTEXT_H

#include <cstdint>
#include <memory>
#include <nuclear>

extern "C" {
#include <aravis-0.6/arv.h>
}

#include "message/input/Image.h"

namespace module {
namespace input {

    // Camera contextual information for Aravis new-buffer callback function.
    struct CameraContext {
        std::string name;
        uint32_t fourcc;
        uint32_t camera_id;
        message::input::Image::Lens lens;
        std::shared_ptr<ArvCamera> camera;
        std::shared_ptr<ArvStream> stream;
        NUClear::Reactor& reactor;

        // Values used to correct the time between the clock on the device and the local clock
        struct TimeCorrection {
            // The amount to add to camera timestamps
            std::int64_t offset;
            union {
                struct {
                    /// The time on the camera at calibration time
                    std::int64_t cam_at_calibration;
                    /// The time on the local machine at calibration time
                    std::int64_t local_at_calibration;
                    /// The largest amount of clock drift that we will allow before forcing a recalibration
                    std::int64_t max_clock_drift;
                    /// Counts how many frames are over recently. +1 for every frame over -1 for every frame under.
                    /// If this exceeds a threshold we assume the clock is out of sync
                    std::int64_t over_time_count;
                } drift;
                struct {
                    // Estimation error covariance
                    double p;
                    // Measurement noise covariance
                    double r;
                    // Process noise covariance
                    double q;
                } kf;
            };
            bool live;
        } time;
    };

}  // namespace input
}  // namespace module

#endif  // MODULE_INPUT_CAMERA_CONTEXT_H
