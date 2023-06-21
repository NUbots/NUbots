#ifndef UTILITY_MOTION_SPLINES_TRAJECTORY_HPP
#define UTILITY_MOTION_SPLINES_TRAJECTORY_HPP

#include <array>
#include <iostream>
#include <vector>

#include "QuinticSpline.hpp"

using utility::motion::splines::QuinticSpline;

namespace utility::motion::splines {

    enum TrajectoryDimension { X, Y, Z, ROLL, PITCH, YAW };

    template <typename Scalar>
    class Trajectory {
    public:
        // Add waypoint for the specified dimension
        void add_waypoint(TrajectoryDimension dimension,
                          Scalar timepoint,
                          const Scalar position,
                          const Scalar velocity     = 0,
                          const Scalar acceleration = 0) {
            waypoints[dimension].push_back(Eigen::Matrix<Scalar, 3, 1>(position, velocity, acceleration));
            timepoints[dimension].push_back(timepoint);
            build_splines_for_dimension(dimension);
        }

        /// @brief Get position (x,y,z) at a given time
        Eigen::Matrix<Scalar, 3, 1> position(Scalar time) const {
            return Eigen::Matrix<Scalar, 3, 1>(eval(X, time), eval(Y, time), eval(Z, time));
        }

        /// @brief Get orientation (roll,pitch,yaw) at a given time
        Eigen::Matrix<Scalar, 3, 1> orientation(Scalar time) const {
            return Eigen::Matrix<Scalar, 3, 1>(eval(ROLL, time), eval(PITCH, time), eval(YAW, time));
        }

        /// @brief Get pose at a given time
        Eigen::Transform<Scalar, 3, Eigen::Isometry> pose(Scalar time) const {
            Eigen::Transform<Scalar, 3, Eigen::Isometry> pose;
            pose.translation() = position(time);
            pose.linear()      = Eigen::AngleAxis<Scalar>(orientation(time)(2), Eigen::Matrix<Scalar, 3, 1>::UnitZ())
                            * Eigen::AngleAxis<Scalar>(orientation(time)(1), Eigen::Matrix<Scalar, 3, 1>::UnitY())
                            * Eigen::AngleAxis<Scalar>(orientation(time)(0), Eigen::Matrix<Scalar, 3, 1>::UnitX())
                                  .toRotationMatrix();
            return pose;
        }

        /// @brief Clear all splines, waypoints, and timepoints for all dimensions
        void clear() {
            for (auto& spline : splines) {
                spline.clear();
            }
            for (auto& waypoint : waypoints) {
                waypoint.clear();
            }
            for (auto& timepoint : timepoints) {
                timepoint.clear();
            }
        }

    private:
        /// @brief Build splines between waypoints for the specified dimension
        void build_splines_for_dimension(TrajectoryDimension dimension) {
            auto& dimensions_splines    = splines[dimension];
            auto& dimensions_waypoints  = waypoints[dimension];
            auto& dimensions_timepoints = timepoints[dimension];

            // If there are less than 2 waypoints, skip building the splines
            if (dimensions_waypoints.size() < 2) {
                return;
            }

            dimensions_splines.clear();
            for (size_t i = 0; i < dimensions_waypoints.size() - 1; ++i) {
                Scalar segment_duration = dimensions_timepoints[i + 1] - dimensions_timepoints[i];
                dimensions_splines.emplace_back(dimensions_waypoints[i], dimensions_waypoints[i + 1], segment_duration);
            }
        }

        /// @brief Evaluate a specific dimension at the given time
        Scalar eval(TrajectoryDimension dimension, Scalar time) const {
            // Determine which segment the time falls into
            size_t segment_idx = 0;
            while (segment_idx < timepoints[dimension].size() && time > timepoints[dimension][segment_idx + 1]) {
                segment_idx++;
            }

            if (segment_idx < timepoints[dimension].size()) {
                Scalar segment_start_time = timepoints[dimension][segment_idx];
                return splines[dimension][segment_idx].position(time - segment_start_time);
            }
            else {
                // If time is beyond the end of the trajectory, return the end value.
                return splines[dimension].back().position(timepoints[dimension].back());
            }
        }

        /// @brief Array of spline vectors for x, y, z, roll, pitch, and yaw.
        std::array<std::vector<QuinticSpline<Scalar>>, 6> splines;

        /// @brief Array of waypoint vectors for x, y, z, roll, pitch, and yaw.
        std::array<std::vector<Eigen::Matrix<Scalar, 3, 1>>, 6> waypoints;

        /// @brief Array of timepoints for x, y, z, roll, pitch, and yaw.
        std::array<std::vector<Scalar>, 6> timepoints;
    };

}  // namespace utility::motion::splines

#endif  // MOTION_TRAJECTORY_HPP
