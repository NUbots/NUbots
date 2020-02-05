#ifndef MODULE_INPUT_CAMERA_CONTEXT_H
#define MODULE_INPUT_CAMERA_CONTEXT_H

#include <Eigen/Geometry>
#include <cstdint>
#include <memory>
#include <nuclear>
#include <string>

#include "message/input/Image.h"

extern "C" {
#include <aravis-0.8/arv.h>
}

namespace module {
namespace input {
    // Forward declare camera
    class Camera;

    // Camera contextual information for Aravis new-buffer callback function.
    struct CameraContext {
        Camera& reactor;
        std::string name;
        uint32_t fourcc;
        uint32_t camera_id;
        message::input::Image::Lens lens;
        // Homogenous transform from platform (p) to camera where platform is the rigid body the camera is attached to
        Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> Hpc;
        std::shared_ptr<ArvCamera> camera;
        std::shared_ptr<ArvStream> stream;

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
