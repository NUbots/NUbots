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
#ifndef MODULE_PURPOSE_PROFILEWALK_HPP
#define MODULE_PURPOSE_PROFILEWALK_HPP

#include <Eigen/Core>
#include <nuclear>
#include <string>
#include <vector>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class ProfileWalk : public ::extension::behaviour::BehaviourReactor {
    public:
        /// @brief Called by the powerplant to build and setup the ProfileWalk reactor
        explicit ProfileWalk(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Rate at which the profile is stepped and the walk command is emitted
        static constexpr size_t UPDATE_RATE = 50;

    private:
        /// @brief A single walk command in the profile, which is ramped up to, held, then ramped back down
        struct Segment {
            /// @brief Human readable name, used for logging
            std::string name = "";
            /// @brief Velocity target (vx, vy, wz) in m/s and rad/s
            Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        };

        /// @brief A group of segments that are run back to back. A fall abandons the whole stage.
        struct Stage {
            /// @brief Human readable name, used for logging
            std::string name = "";
            /// @brief Segments run in order
            std::vector<Segment> segments{};
        };

        struct Config {
            /// @brief Time to wait after configuration before the profile starts, in seconds
            double start_delay = 0.0;
            /// @brief Time taken to linearly ramp between velocity targets, in seconds
            double ramp_time = 0.0;
            /// @brief Time each velocity target is held for once the ramp is finished, in seconds
            double hold_time = 0.0;
            /// @brief Time spent standing still between segments and stages, in seconds
            double rest_time = 0.0;
            /// @brief Time spent standing still after getting up before the next stage starts, in seconds
            double recovery_time = 0.0;
            /// @brief Whether to ramp down and rest between the segments within a stage
            bool rest_between_segments = true;
            /// @brief Whether to restart the profile once the last stage finishes
            bool loop = false;
            /// @brief Head yaw held for the duration of the profile, in radians
            double neck_yaw = 0.0;
            /// @brief Head pitch held for the duration of the profile, in radians
            double head_pitch = 0.0;
            /// @brief The stages of the profile, run in order
            std::vector<Stage> profile{};
        } cfg;

        /// @brief What the profile is currently doing
        enum class ProfilePhase {
            /// @brief Standing still, waiting for the start delay to elapse
            WAITING,
            /// @brief Ramping from the previous velocity towards the current segment's velocity
            RAMP_UP,
            /// @brief Holding the current segment's velocity
            HOLD,
            /// @brief Ramping the current segment's velocity back down to zero
            RAMP_DOWN,
            /// @brief Standing still between segments/stages
            REST,
            /// @brief The robot is on the ground or getting up, the profile is paused
            FALLEN,
            /// @brief Every stage has been run, standing still
            FINISHED
        };

        /// @brief The phase the profile is currently in
        ProfilePhase phase = ProfilePhase::WAITING;

        /// @brief The phase the profile was in when the robot fell, so we know how to resume
        ProfilePhase phase_before_fall = ProfilePhase::WAITING;

        /// @brief Whether the current rest is the settle after getting up rather than the usual rest
        bool rest_after_recovery = false;

        /// @brief Index of the stage currently being run
        size_t stage_index = 0;

        /// @brief Index of the segment within the current stage
        size_t segment_index = 0;

        /// @brief The time the current phase started
        NUClear::clock::time_point phase_start_time{};

        /// @brief The velocity the current ramp starts from
        Eigen::Vector3d ramp_from = Eigen::Vector3d::Zero();

        /// @brief The velocity the current ramp finishes at
        Eigen::Vector3d ramp_to = Eigen::Vector3d::Zero();

        /// @brief The velocity currently being commanded
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

        /// @brief Time in seconds since the current phase started
        double phase_elapsed() const;

        /// @brief Moves the profile into the given phase and restarts the phase timer
        /// @param new_phase The phase to move into
        void set_phase(const ProfilePhase& new_phase);

        /// @brief Resets the profile back to the very start, standing still for the start delay
        void reset_profile();

        /// @brief Steps the profile state machine and updates the commanded velocity
        void update_profile();

        /// @brief Starts ramping towards the current segment's velocity
        /// @param from The velocity to ramp from
        void start_segment(const Eigen::Vector3d& from);

        /// @brief Moves to the next segment, rolling over into the next stage when the current one is done
        void next_segment();

        /// @brief Abandons the rest of the current stage and moves to the start of the next one
        void skip_stage();

        /// @brief Starts the current segment, or finishes the profile if there are no stages left
        void start_or_finish();

        /// @brief Whether every stage in the profile has been run
        bool profile_complete() const;

        /// @brief The segment currently being run
        const Segment& current_segment() const;

        /// @brief Name of the current stage and segment, for logging
        std::string current_name() const;
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_PROFILEWALK_HPP
