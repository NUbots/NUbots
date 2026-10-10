/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#ifndef MODULE_PLANNING_MODELLEDMPCWALKPATH_HPP
#define MODULE_PLANNING_MODELLEDMPCWALKPATH_HPP

#include <Eigen/Core>
#include <memory>
#include <nuclear>

#include "ModelledWalkMPC.hpp"

#include "extension/Behaviour.hpp"

#include "message/behaviour/state/Stability.hpp"

// PlanWalkPath's control law and dead zone are the fallback. Included from PlanWalkPath rather than copied, so the
// planners can't drift apart.
#include "../../PlanWalkPath/src/walk_path_control.hpp"

namespace module::planning {

    class ModelledMPCWalkPath : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The MPC's limits, tuning and model
            modelled_walk_mpc::Config mpc{};
            /// @brief Seconds. A solve that takes longer, or fails, sends the fallback's command instead
            double time_budget = 0.05;
            /// @brief Seconds without a WalkTo after which the next solve starts from standing still
            double stale_time = 0.5;
            /// @brief The fallback: PlanWalkPath's WalkTo control law
            walk_path::WalkToParams fallback{};
            /// @brief The fallback's dead zone (PlanWalkPath's): minimum effective command per axis
            Eigen::Vector3d fallback_min_velocity = Eigen::Vector3d::Zero();
            /// @brief The fallback's dead zone: commands below this per-axis magnitude are snapped to zero
            Eigen::Vector3d fallback_zero_tolerance = Eigen::Vector3d::Zero();
            /// @brief The fallback's dead zone: forward velocity given to a turn with no translation
            double fallback_rotate_velocity_x = 0.0;
        } cfg;

        /// @brief The MPC, created on the first configuration
        std::unique_ptr<modelled_walk_mpc::ModelledWalkMPC> mpc{};
        /// @brief When the last WalkTo was planned, to tell a fresh request from a continuing one
        NUClear::clock::time_point last_plan_time{};
        /// @brief The last command sent to the walk
        Eigen::Vector3d last_walk_command = Eigen::Vector3d::Zero();
        /// @brief Solves that fell back to PlanWalkPath's control law since the last log
        int fallbacks = 0;
        /// @brief Current stability of the robot
        message::behaviour::state::Stability stability{};

    public:
        /// @brief Called by the powerplant to build and setup the ModelledMPCWalkPath reactor.
        explicit ModelledMPCWalkPath(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_MODELLEDMPCWALKPATH_HPP
