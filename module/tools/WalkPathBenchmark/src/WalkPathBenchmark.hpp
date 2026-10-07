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
#ifndef MODULE_TOOLS_WALKPATHBENCHMARK_HPP
#define MODULE_TOOLS_WALKPATHBENCHMARK_HPP

#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <nuclear>
#include <vector>

#include "extension/Behaviour.hpp"

namespace module::tools {

    class WalkPathBenchmark : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Seconds to wait after startup before the first trial
            double start_delay = 0.0;
            /// @brief Target poses [x, y, theta] in the field frame, walked to in order
            std::vector<Eigen::Vector3d> targets{};
            /// @brief Whether the targets are relative to the robot's pose when the trials start
            bool relative_targets = false;
            /// @brief NUSim world position [x, y] to move the ball to before starting, empty to leave it
            std::vector<double> park_ball{};
            /// @brief Walk commands [vx, vy, wz] held open loop in turn before the trials, to measure the velocity
            /// the walk delivers for each
            std::vector<Eigen::Vector3d> sweep_commands{};
            /// @brief Seconds each sweep command is held
            double sweep_duration = 0.0;
            /// @brief Seconds at the end of each hold the delivered velocity is averaged over
            double sweep_measure = 0.0;
            /// @brief Seconds before a trial is abandoned as a failure
            double trial_timeout = 0.0;
            /// @brief Position error (m) within which the robot counts as at the target
            double reach_position_error = 0.0;
            /// @brief Heading error (rad) within which the robot counts as at the target
            double reach_heading_error = 0.0;
            /// @brief Seconds the robot must stay at the target for the trial to finish
            double settle_time = 0.0;
            /// @brief Torso speed (m/s) below which the robot is at rest
            double rest_speed = 0.0;
            /// @brief Torso turn rate (rad/s) below which the robot is at rest
            double rest_turn_rate = 0.0;
            /// @brief Torso height (m) below which the robot counts as fallen
            double fall_height = 0.0;
            /// @brief Emit the Field from NUSim ground truth (field frame = NUSim world), replacing localisation
            bool ground_truth_field = false;
            /// @brief Priority of the FallRecovery task
            int fall_recovery_priority = 0;
            /// @brief Shut down the powerplant once every trial has run
            bool shutdown_when_done = false;
            /// @brief Directory for the per-tick trajectory CSVs, one per run named by its local start time, empty
            /// to disable
            std::filesystem::path csv_directory{};
        } cfg;

        /// @brief When the reactor started, for the start delay
        NUClear::clock::time_point startup_time{};

        /// @brief Benchmark progress
        enum class State { WAITING, SWEEP, RUNNING, DONE } state = State::WAITING;

        /// @brief Measurement of the sweep command in progress
        struct Sweep {
            NUClear::clock::time_point start{};
            /// @brief Sum of the body-frame velocity samples [vx, vy, wz]
            Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
            int samples              = 0;
            int falls                = 0;
            bool fallen              = false;
        } sweep{};

        /// @brief Index of the sweep command in progress
        size_t sweep_index = 0;

        /// @brief Metrics of the trial in progress
        struct Trial {
            NUClear::clock::time_point start{};
            Eigen::Vector3d start_pose    = Eigen::Vector3d::Zero();
            Eigen::Vector2d last_position = Eigen::Vector2d::Zero();
            double path_length            = 0.0;
            /// @brief Seconds until the robot first reached the target, negative until it does
            double reach_time = -1.0;
            /// @brief Seconds at which the robot entered its final stay at the target, negative while outside it
            double settle_start = -1.0;
            /// @brief Seconds at which the robot came to rest, negative while moving
            double rest_start = -1.0;
            /// @brief Largest position error after first reaching the target (overshoot / drift)
            double max_error_after_reach = 0.0;
            int falls                    = 0;
            bool fallen                  = false;
        } trial{};

        /// @brief Targets resolved in the field frame when the benchmark starts
        std::vector<Eigen::Vector3d> targets{};

        /// @brief Index of the trial in progress
        size_t trial_index = 0;

        /// @brief Totals over every trial
        double total_time       = 0.0;
        double total_pos_error  = 0.0;
        double total_head_error = 0.0;
        int total_falls         = 0;
        int failures            = 0;

        /// @brief Latest commanded walk velocity (post smoothing and dead zone)
        Eigen::Vector3d walk_command = Eigen::Vector3d::Zero();

        /// @brief Trajectory output
        std::ofstream csv{};

        /// @brief Begins holding the sweep command at sweep_index
        void start_sweep();
        /// @brief Resolves the targets from the robot's current pose and begins the first trial
        void begin_trials(const Eigen::Vector3d& pose);
        /// @brief Begins the trial at trial_index from the robot's current pose
        void start_trial(const Eigen::Vector3d& pose);
        /// @brief How a trial ended: settled at the target, came to rest outside it, or ran out of time
        enum class Outcome { OK, SHORT, TIMEOUT };
        /// @brief Logs and accumulates the result of the trial in progress, then moves on
        void finish_trial(const Eigen::Vector3d& pose, Outcome outcome);

    public:
        /// @brief Called by the powerplant to build and setup the WalkPathBenchmark reactor.
        explicit WalkPathBenchmark(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_WALKPATHBENCHMARK_HPP
