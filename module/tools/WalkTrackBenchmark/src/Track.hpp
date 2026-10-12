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
#ifndef MODULE_TOOLS_WALKTRACKBENCHMARK_TRACK_HPP
#define MODULE_TOOLS_WALKTRACKBENCHMARK_TRACK_HPP

#include <Eigen/Core>
#include <vector>

namespace module::tools::walk_track {

    /// @brief One piece of a moving target's path. The target moves along its travel direction at a speed that changes
    /// linearly (stopping at zero), while the travel direction turns in proportion to the distance covered.
    struct Segment {
        /// @brief Speed (m/s) at the start of the segment
        double speed = 0.0;
        /// @brief Rate (m/s²) the speed changes at through the segment. A negative one stops the target, which then
        /// stays put until the segment ends (a rolling ball coming to rest).
        double acceleration = 0.0;
        /// @brief Turn of the travel direction per metre travelled (rad/m, positive to the left): 1/radius of an arc
        double curvature = 0.0;
        /// @brief Turn of the travel direction (rad) at the start of the segment, e.g. π to double back
        double turn = 0.0;
        /// @brief Seconds the segment lasts
        double duration = 0.0;
    };

    /// @brief Where a moving target is and how it is moving at one time
    struct TargetState {
        /// @brief Target pose [x, y, theta]: its position, and the heading the robot should hold there (the travel
        /// direction plus the track's heading offset)
        Eigen::Vector3d pose = Eigen::Vector3d::Zero();
        /// @brief Direction of travel (rad), defined while stationary too
        double direction = 0.0;
        /// @brief Velocity [vx, vy] (m/s)
        Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    };

    /// @brief A target moving along a path of segments, in closed form
    class Track {
    public:
        /// @param start          the target's position [x, y] and initial travel direction (rad) when it starts moving
        /// @param heading_offset heading the robot should hold relative to the travel direction (rad), e.g. π/2 to
        ///                       walk alongside it sideways
        /// @param segments       the path, in order
        /// @param speed_scale    runs the same path this many times faster: speeds scale by it, accelerations by its
        ///                       square and durations by its inverse, so the target covers the same ground
        Track(const Eigen::Vector3d& start,
              double heading_offset,
              const std::vector<Segment>& segments,
              double speed_scale = 1.0);

        /// @brief The target at t seconds after it starts moving. Before 0 it waits at the start, and after the end it
        /// stays where the last segment left it, stationary.
        [[nodiscard]] TargetState at(double t) const;

        /// @brief Seconds from starting to move to the end of the last segment
        [[nodiscard]] double duration() const {
            return total_duration;
        }

    private:
        /// @brief A segment, scaled, with where the target is when it starts
        struct Piece {
            Segment segment{};
            /// @brief Seconds from the start of the track to the start of this piece
            double start_time = 0.0;
            /// @brief Position and travel direction at the start of the piece, after its turn
            Eigen::Vector2d position = Eigen::Vector2d::Zero();
            double direction         = 0.0;
        };

        /// @brief Distance (m) and speed (m/s) a segment has covered after tau seconds
        static std::pair<double, double> travelled(const Segment& segment, double tau);

        /// @brief The target's state tau seconds into a piece
        [[nodiscard]] TargetState state(const Piece& piece, double tau) const;

        std::vector<Piece> pieces{};
        double heading_offset = 0.0;
        double total_duration = 0.0;
        /// @brief Where the target starts and where it ends up, for times outside the segments
        TargetState first{};
        TargetState last{};
    };

}  // namespace module::tools::walk_track

#endif  // MODULE_TOOLS_WALKTRACKBENCHMARK_TRACK_HPP
