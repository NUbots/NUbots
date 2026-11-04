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
#ifndef MODULE_PLANNING_DYNAMICINVERTEDMPCWALKPATH_DYNAMICINVERSE_HPP
#define MODULE_PLANNING_DYNAMICINVERTEDMPCWALKPATH_DYNAMICINVERSE_HPP

#include <Eigen/Core>
#include <array>
#include <vector>

#include "utility/planning/HammersteinModel.hpp"

namespace module::planning::dynamic_inverted_walk_mpc {

    /// @brief The one-step dynamic inverse of the Hammerstein model: the command that, held from the model's estimate
    /// of the robot's lag states now, delivers a velocity one planner step later. The estimate is open loop, as
    /// ModelledMPCWalkPath's is: the lag states are run from the commands sent, advanced by the time since the last
    /// solve before each solve.
    ///
    /// The search is ResponseTable's: translation first, from every (vx, vy) command step apart with no turn, then the
    /// turn given that forward command; then, around the closest, a finer search with the model itself, as the
    /// model's dead zones end in steep edges. Of the commands within TIE_TOLERANCE of the closest, the one nearest
    /// zero. Commands stay within range, so a velocity the lag can't reach in one step gets the command that comes
    /// closest, at the edge of the range.
    class DynamicInverse {
    public:
        using HammersteinModel = utility::planning::HammersteinModel;

        /// @brief How much worse than the best match (m/s or rad/s) a command can be and still be chosen for being
        /// smaller, as the response is flat where the policy stands still
        static constexpr double TIE_TOLERANCE = 0.005;
        /// @brief Steps either side of the closest grid command, a twentieth of a grid step apart, that the inverse
        /// searches with the model itself
        static constexpr int REFINEMENT = 20;

        /// @param model The Hammerstein model
        /// @param range The command range per axis [vx, vy, ω] either side of zero (the model's identified range)
        /// @param step The step of the command grid the inverse searches
        /// @param horizon How far ahead the velocity is to be delivered (s): the planner's step, rounded to the
        ///                model's samples
        DynamicInverse(const HammersteinModel& model, const Eigen::Vector3d& range, double step, double horizon);

        /// @brief Forgets the estimate: the robot is standing, as every run the model was fitted on started
        void reset();

        /// @brief Runs the estimate forward by elapsed seconds, with the last command recorded held
        void advance(double elapsed);

        /// @brief Records the command sent, which the estimate holds until the next advance
        void record(const Eigen::Vector3d& command);

        /// @brief The estimate of the velocity the robot is delivering now
        [[nodiscard]] Eigen::Vector3d delivered() const;

        /// @brief The velocity the robot delivers one horizon from now with the command held
        [[nodiscard]] Eigen::Vector3d after_step(const Eigen::Vector3d& command) const;

        /// @brief The smallest command that delivers the velocity [vx, vy, ω] one horizon from now
        [[nodiscard]] Eigen::Vector3d inverse(const Eigen::Vector3d& velocity) const;

        /// @brief The smallest and largest velocity the model settles to on each axis [vx, vy, ω], for commands in
        /// range: what the planner can plan
        [[nodiscard]] const Eigen::Vector3d& lower() const {
            return lo;
        }
        [[nodiscard]] const Eigen::Vector3d& upper() const {
            return hi;
        }

        /// @brief Model samples in the horizon
        [[nodiscard]] int samples() const {
            return n;
        }

    private:
        HammersteinModel model;
        Eigen::Vector3d range;
        double step;
        int n;
        /// @brief The commands searched on each axis [vx, vy, ω]
        std::array<std::vector<double>, 3> axes{};
        /// @brief Over the horizon from standing, what each translation path's lag state rises to for each command on
        /// its axis: vx←vx, vx←vy, vy←vx, vy←vy
        std::array<std::vector<double>, 4> rise{};
        Eigen::Vector3d lo = Eigen::Vector3d::Zero();
        Eigen::Vector3d hi = Eigen::Vector3d::Zero();

        HammersteinModel::Lags lags = HammersteinModel::Lags::Zero();
        /// @brief Time advanced that is less than a model sample, carried to the next advance (s)
        double leftover = 0.0;
        /// @brief The command recorded last
        Eigen::Vector3d u_prev = Eigen::Vector3d::Zero();
    };

}  // namespace module::planning::dynamic_inverted_walk_mpc

#endif  // MODULE_PLANNING_DYNAMICINVERTEDMPCWALKPATH_DYNAMICINVERSE_HPP
