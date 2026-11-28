/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 */
#ifndef MODULE_SUPPORT_DYNAMICSRECORDER_HPP
#define MODULE_SUPPORT_DYNAMICSRECORDER_HPP

#include <Eigen/Geometry>
#include <deque>
#include <nuclear>

namespace module::support {

    /// DEBUG(dynamics alignment): logs a DYNREC line for every joint command sent to the robot, with the sensor
    /// sample it followed and NUSim's true torso and ball at that sample's time.
    class DynamicsRecorder : public NUClear::Reactor {
    public:
        explicit DynamicsRecorder(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Truth {
            NUClear::clock::time_point time;
            Eigen::Isometry3d Hst;
            Eigen::Vector3d rBSs;
            Eigen::Vector3d vBs;
        };
        std::deque<Truth> truth;
    };

}  // namespace module::support

#endif  // MODULE_SUPPORT_DYNAMICSRECORDER_HPP
