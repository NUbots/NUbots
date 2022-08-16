#ifndef MODULE_INPUT_CAMERA_CONTEXT_HPP
#define MODULE_INPUT_CAMERA_CONTEXT_HPP

#include <Eigen/Geometry>
#include <aravis-0.8/arv.h>
#include <atomic>
#include <cstdint>
#include <memory>
#include <nuclear>
#include <string>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"


namespace module::input {
    // Forward declare camera
    class Camera;

    // Camera contextual information for Aravis new-buffer callback function.
    struct CameraContext {
        Camera& reactor;
        ::extension::Configuration config;
        std::string name;
        uint32_t fourcc;
        uint32_t id;
        message::input::Image::Lens lens;
        // Homogenous transform from platform (p) to camera where platform is the rigid body the camera is attached to
        Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> Hpc;
        std::shared_ptr<ArvCamera> camera;
        std::shared_ptr<ArvStream> stream;
        std::shared_ptr<ArvDevice> device;

        // Values used to correct the time between the clock on the device and the local clock
        struct TimeCorrection {
            // The type of method we are using to sync with the camera
            // The amount to add to camera timestamps
            std::int64_t offset;
            union {
                struct {  // NOLINT(clang-diagnostic-nested-anon-types)
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
                struct {  // NOLINT(clang-diagnostic-nested-anon-types)
                    // Estimation error covariance
                    double p;
                    // Measurement noise covariance
                    double r;
                    // Process noise covariance
                    double q;
                } kf;
            };
            enum { LIVE, LATCHED, PTP } type;
        } time;
    };

}  // namespace module::input

#endif  // MODULE_INPUT_CAMERA_CONTEXT_HPP
