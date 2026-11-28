/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 */
#ifndef MODULE_SUPPORT_PERCEPTIONRECORDER_HPP
#define MODULE_SUPPORT_PERCEPTIONRECORDER_HPP

#include <Eigen/Geometry>
#include <deque>
#include <nuclear>

namespace module::support {

    /// DEBUG(perception model): logs one PAMREC line per Yolo frame with the detection (as skill::Walk's ball
    /// skills would see it), NUSim's true ball in the same frame at the image time, and the robot's motion.
    class PerceptionRecorder : public NUClear::Reactor {
    public:
        explicit PerceptionRecorder(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Truth {
            NUClear::clock::time_point time;
            Eigen::Isometry3d Hst;
            Eigen::Vector3d rBSs;
            Eigen::Vector3d vBs;
        };
        struct Motion {
            NUClear::clock::time_point time;
            Eigen::Isometry3d Htw;
            Eigen::Isometry3d Hcw;
            Eigen::Vector3d gyro;
            double head_yaw_vel   = 0.0;
            double head_pitch_vel = 0.0;
            double head_yaw       = 0.0;
            double head_pitch     = 0.0;
        };
        std::deque<Truth> truth;
        std::deque<Motion> motion;
        Eigen::Isometry3d Hpc = Eigen::Isometry3d::Identity();
        Eigen::Vector3d offset{0.06, 0.0, 0.07};
        double ball_radius = 0.0785;
    };

}  // namespace module::support

#endif  // MODULE_SUPPORT_PERCEPTIONRECORDER_HPP
