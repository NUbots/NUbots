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
#ifndef MODULE_PLANNING_INVERTEDMPCWALKPATH_HPP
#define MODULE_PLANNING_INVERTEDMPCWALKPATH_HPP

#include <Eigen/Core>
#include <memory>
#include <nuclear>
#include <optional>

#include "InvertedWalkMPC.hpp"
#include "ResponseTable.hpp"

#include "extension/Behaviour.hpp"

#include "message/behaviour/state/Stability.hpp"

// PlanWalkPath's control law and dead zone are the fallback. Included from PlanWalkPath rather than copied, so the
// planners can't drift apart.
#include "../../PlanWalkPath/src/walk_path_control.hpp"

namespace module::planning {

    class InvertedMPCWalkPath : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The MPC's tuning and envelope. Its velocity limits are the ones below, within what the response
            /// table delivers.
            inverted_walk_mpc::Config mpc{};
            /// @brief Maximum planned velocity per axis [vx (m/s), vy (m/s), vtheta (rad/s)]. It bounds the plan, not
            /// the command that delivers it.
            Eigen::Vector3d max_velocity = Eigen::Vector3d::Zero();
            /// @brief Maximum backward planned velocity (m/s)
            double max_backward_velocity = 0.0;
            /// @brief Planned velocities below this per-axis magnitude are taken as zero before finding the command
            Eigen::Vector3d zero_tolerance = Eigen::Vector3d::Zero();
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

        /// @brief The walk policy's measured response, once InvertedMPCWalkPathResponse.yaml has been read
        std::optional<inverted_walk_mpc::ResponseTable> response{};
        /// @brief Whether InvertedMPCWalkPath.yaml has been read
        bool have_config = false;
        /// @brief The MPC, created once both configuration files have been read
        std::unique_ptr<inverted_walk_mpc::InvertedWalkMPC> mpc{};
        /// @brief When the last WalkTo was planned, to tell a fresh request from a continuing one
        NUClear::clock::time_point last_plan_time{};
        /// @brief The last command sent to the walk
        Eigen::Vector3d last_walk_command = Eigen::Vector3d::Zero();
        /// @brief Solves that fell back to PlanWalkPath's control law since the last log
        int fallbacks = 0;
        /// @brief Current stability of the robot
        message::behaviour::state::Stability stability{};

        /// @brief Creates or reconfigures the MPC, once both configuration files have been read
        void configure_mpc();

    public:
        /// @brief Called by the powerplant to build and setup the InvertedMPCWalkPath reactor.
        explicit InvertedMPCWalkPath(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_INVERTEDMPCWALKPATH_HPP
