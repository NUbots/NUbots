/*
 * MIT License
 *
 * Copyright (c) 2020 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef MODULE_INPUT_CAMERA_CONTEXT_HPP
#define MODULE_INPUT_CAMERA_CONTEXT_HPP

#include <Eigen/Geometry>
#include <cstdint>
#include <memory>
#include <nuclear>
#include <string>

#include "message/input/Image.hpp"
#include "message/input/Lens.hpp"

extern "C" {
#include <aravis-0.8/arv.h>
}

namespace module::input {
    // Forward declare camera
    class Camera;

    // Camera contextual information for Aravis new-buffer callback function.
    struct CameraContext {
        Camera& reactor;
        std::string name;
        uint32_t fourcc;
        uint32_t id;
        message::input::Lens lens;
        // Homogenous transform from platform (p) to camera where platform is the rigid body the camera is attached
        // to
        Eigen::Isometry3d Hpc;
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
}  // namespace module::input

#endif  // MODULE_INPUT_CAMERA_CONTEXT_HPP
