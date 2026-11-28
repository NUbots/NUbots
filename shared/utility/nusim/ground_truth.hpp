/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 */
#ifndef UTILITY_NUSIM_GROUND_TRUTH_HPP
#define UTILITY_NUSIM_GROUND_TRUTH_HPP

#include <Eigen/Geometry>
#include <nuclear>

namespace utility::nusim {

    /// NUSim's true robot torso and ball, in the simulator's world frame {s}. Emitted in-process by
    /// input::NUSimGroundTruth whenever both feeds are fresh, whether or not ground truth is used for
    /// localisation. For tests and benchmarks that score the robot, never for the robot's own decisions.
    struct GroundTruthState {
        Eigen::Isometry3d Hst = Eigen::Isometry3d::Identity();
        Eigen::Vector3d rBSs  = Eigen::Vector3d::Zero();
        Eigen::Vector3d vBs   = Eigen::Vector3d::Zero();
        NUClear::clock::time_point time{};
    };

}  // namespace utility::nusim

#endif  // UTILITY_NUSIM_GROUND_TRUTH_HPP
