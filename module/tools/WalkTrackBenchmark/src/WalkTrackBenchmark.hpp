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
#ifndef MODULE_TOOLS_WALKTRACKBENCHMARK_HPP
#define MODULE_TOOLS_WALKTRACKBENCHMARK_HPP

#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <nuclear>
#include <string>
#include <vector>

#include "Track.hpp"

#include "extension/Behaviour.hpp"

namespace module::tools {

    class WalkTrackBenchmark : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief A configured track, before it is placed in the field
        struct TrackConfig {
            std::string name{};
            /// @brief Where the target starts [x, y] and its initial travel direction (rad)
            Eigen::Vector3d start = Eigen::Vector3d::Zero();
            /// @brief Heading the robot should hold relative to the travel direction (rad)
            double heading_offset = 0.0;
            std::vector<walk_track::Segment> segments{};
        };

        /// @brief Stores configuration values
        struct Config {
            /// @brief Seconds to wait after startup before the first track
            double start_delay = 0.0;
            /// @brief Whether the tracks are relative to the robot's pose when the benchmark starts
            bool relative_targets = false;
            /// @brief NUSim world position [x, y] to move the ball to before starting, empty to leave it
            std::vector<double> park_ball{};
            /// @brief The tracks, run in order once per speed scale
            std::vector<TrackConfig> tracks{};
            /// @brief Each pass over the tracks runs them this many times faster (same paths)
            std::vector<double> speed_scales{};
            /// @brief Before each track the target waits at its start until the robot has stayed within these errors
            /// (m, rad) for approach_settle seconds, or approach_timeout seconds have passed
            double approach_position_error = 0.0;
            double approach_heading_error  = 0.0;
            double approach_settle         = 0.0;
            double approach_timeout        = 0.0;
            /// @brief Position error (m) beyond which the robot counts as having lost the target
            double lost_distance = 0.0;
            /// @brief Torso height (m) below which the robot counts as fallen
            double fall_height = 0.0;
            /// @brief Emit the Field from NUSim ground truth (field frame = NUSim world), replacing localisation
            bool ground_truth_field = false;
            /// @brief Priority of the FallRecovery task
            int fall_recovery_priority = 0;
            /// @brief Shut down the powerplant once every track has run
            bool shutdown_when_done = false;
            /// @brief Directory for the per-tick trajectory CSVs, one per run named by its local start time, empty
            /// to disable
            std::filesystem::path csv_directory{};
        } cfg;

        /// @brief When the reactor started, for the start delay
        NUClear::clock::time_point startup_time{};

        /// @brief Benchmark progress: waiting to start, the target waiting for the robot at a track's start, the
        /// target moving, or finished
        enum class State { WAITING, APPROACH, TRACKING, DONE } state = State::WAITING;

        /// @brief A track placed in the field, with its speed scale
        struct Run {
            std::string name{};
            double speed_scale = 1.0;
            walk_track::Track track;
        };
        std::vector<Run> runs{};

        /// @brief Index of the run in progress
        size_t run_index = 0;

        /// @brief Progress and metrics of the run in progress
        struct Progress {
            /// @brief When the current phase (approach or tracking) began
            NUClear::clock::time_point phase_start{};
            /// @brief Seconds into the approach at which the robot entered its current stay at the start, negative
            /// while outside it
            double settle_start = -1.0;
            /// @brief Seconds the approach took, and whether it ended by the robot reaching the start
            double approach_time = 0.0;
            bool approached      = false;
            /// @brief Per-tick errors while tracking: position (m), heading (rad), along the travel direction (m,
            /// positive behind the target) and across it (m, unsigned)
            std::vector<double> position_errors{};
            std::vector<double> heading_errors{};
            std::vector<double> along_errors{};
            std::vector<double> cross_errors{};
            int falls   = 0;
            bool fallen = false;
        } progress{};

        /// @brief Every tracking tick's errors over all runs, for the summary
        std::vector<double> all_position_errors{};
        std::vector<double> all_heading_errors{};
        int total_falls = 0;

        /// @brief Latest commanded walk velocity (post smoothing and dead zone)
        Eigen::Vector3d walk_command = Eigen::Vector3d::Zero();

        /// @brief Trajectory output
        std::ofstream csv{};

        /// @brief Places the tracks in the field from the robot's current pose and begins the first approach
        void begin(const Eigen::Vector3d& pose);
        /// @brief Begins the approach of the run at run_index
        void start_run();
        /// @brief Logs and accumulates the run in progress, then moves on
        void finish_run();

    public:
        /// @brief Called by the powerplant to build and setup the WalkTrackBenchmark reactor.
        explicit WalkTrackBenchmark(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_WALKTRACKBENCHMARK_HPP
