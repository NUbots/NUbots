#ifndef MODULE_MOTION_KICK_HPP
#define MODULE_MOTION_KICK_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <nuclear>

#include "utility/input/LimbID.hpp"
#include "utility/math/euler.hpp"
#include "utility/motion/splines/Trajectory.hpp"

namespace utility::skill {

    using utility::input::LimbID;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::motion::splines::Trajectory;
    using utility::motion::splines::TrajectoryDimension::PITCH;
    using utility::motion::splines::TrajectoryDimension::ROLL;
    using utility::motion::splines::TrajectoryDimension::X;
    using utility::motion::splines::TrajectoryDimension::Y;
    using utility::motion::splines::TrajectoryDimension::YAW;
    using utility::motion::splines::TrajectoryDimension::Z;


    /// @brief Motion generation options.
    template <typename Scalar>
    struct KickOptions {
        /// @brief Foot waypoints
        std::vector<Eigen::Matrix<Scalar, 4, 1>> foot_waypoints{};

        /// @brief Torso waypoints
        std::vector<Eigen::Matrix<Scalar, 4, 1>> torso_waypoints{};

        /// @brief Torso pitch (in radians)
        Scalar torso_pitch = 0.0;

        /// @brief Whether the planted foot should be the left foot or the right foot
        bool left_foot_is_planted = false;
    };

    template <typename Scalar>
    class KickGenerator {
    public:
        /**
         * @brief Configure kick options.
         * @param options Kick options.
         */
        void configure(const KickOptions<Scalar>& options) {
            // Configure the torso trajectory
            foot_waypoints  = options.foot_waypoints;
            torso_waypoints = options.torso_waypoints;
            torso_pitch     = options.torso_pitch;

            // Assert that the first timepoint of both trajectories is 0
            if (foot_waypoints.front()(3) != 0.0 || torso_waypoints.front()(3) != 0.0) {
                throw std::runtime_error("The first waypoint of the foot and torso trajectories must be at time 0.");
            }

            // Assert that the length of the foot waypoints is the same as the length of the torso waypoints
            if (foot_waypoints.back()(3) != torso_waypoints.back()(3)) {
                throw std::runtime_error(
                    "The last waypoint of the foot and torso trajectories must have the same time.");
            }

            // Get the total duration of the kick
            kick_duration = foot_waypoints.back()(3);
        }

        /**
         * @brief Get the swing foot pose at the current time.
         * @return Trajectory of torso.
         */
        Eigen::Transform<Scalar, 3, Eigen::Isometry> get_swing_foot_pose() const {
            return swingfoot_trajectory.pose(t);
        }

        /**
         * @brief Get the swing foot pose at the given time.
         * @param t Time.
         * @return Swing foot pose at time t.
         */
        Eigen::Transform<Scalar, 3, Eigen::Isometry> get_swing_foot_pose(Scalar t) const {
            return swingfoot_trajectory.pose(t);
        }

        /**
         * @brief Get the torso pose object at the current time.
         * @return Pose of torso.
         */
        Eigen::Transform<Scalar, 3, Eigen::Isometry> get_torso_pose() const {
            return torso_trajectory.pose(t);
        }

        /**
         * @brief Get the torso pose object at the given time.
         * @param t Time.
         * @return Pose of torso at time t.
         */
        Eigen::Transform<Scalar, 3, Eigen::Isometry> get_torso_pose(Scalar t) const {
            return torso_trajectory.pose(t);
        }

        /**
         * @brief Get the left or right foot pose at the given time in the torso {t} frame.
         * @param t Time.
         * @param limb Limb ID of foot to get pose of.
         * @return Swing foot pose at time t.
         */
        Eigen::Transform<Scalar, 3, Eigen::Isometry> get_foot_pose(const LimbID& limb) const {
            // Assign the value based on the foot planted
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Htl =
                left_foot_is_planted ? get_torso_pose().inverse() : get_torso_pose().inverse() * get_swing_foot_pose();
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Htr =
                left_foot_is_planted ? get_torso_pose().inverse() * get_swing_foot_pose() : get_torso_pose().inverse();

            // Return the desired pose of the specified foot
            if (limb == LimbID::LEFT_LEG)
                return Htl;
            if (limb == LimbID::RIGHT_LEG)
                return Htr;
            throw std::runtime_error("Invalid Limb ID");
        }

        /**
         * @brief Reset kick generator.
         */
        void reset() {
            // Generate swing foot and torso trajectories
            generate_trajectories();

            // Reset time
            t = 0;
        }

        /**
         * @brief Run an update of the walk engine, updating the time and engine state and trajectories.
         * @param dt Time step.
         * @return Engine state.
         */
        void update(const Scalar& dt) {
            NUClear::log<NUClear::DEBUG>("dt: ", dt);
            // Check for negative time step
            if (dt <= 0.0f) {
                NUClear::log<NUClear::WARN>("dt <= 0.0f");
                return;
            }

            // Check for too long dt
            if (dt > kick_duration) {
                NUClear::log<NUClear::WARN>("dt > kick_duration", dt, kick_duration);
                return;
            }

            // Update the phase
            t += dt;

            // Reset time after it reaches the end of the kick
            if (t >= kick_duration) {
                t = 0;
            }
        }


        /**
         * @brief Getter for the current time.
         * @return Current time.
         */
        Scalar get_time() const {
            return t;
        }

        /**
         * @brief Getter for the duration of the kick.
         * @return Duration of the kick.
         */
        Scalar get_duration() const {
            return kick_duration;
        }

        /**
         * @brief Get whether or not the left foot is planted.
         * @return True if left foot is planted, false otherwise.
         */
        bool is_left_foot_planted() const {
            return left_foot_is_planted;
        }

    private:
        // ******************************** Options ********************************

        /// @brief Foot waypoints
        std::vector<Eigen::Matrix<Scalar, 4, 1>> foot_waypoints{};

        /// @brief Torso waypoints
        std::vector<Eigen::Matrix<Scalar, 4, 1>> torso_waypoints{};

        /// @brief Torso pitch (in radians)
        Scalar torso_pitch = 0.0;

        // ******************************** State ********************************

        // Duration of complete kick
        Scalar kick_duration = 0.0;

        /// @brief Whether the left foot is planted.
        bool left_foot_is_planted = false;

        /// @brief Current time in the step cycle
        Scalar t = 0.0;

        // ******************************** Trajectories ********************************

        // 6D piecewise polynomial trajectory for swing foot.
        Trajectory<Scalar> swingfoot_trajectory{};

        // 6D piecewise polynomial trajectory for torso.
        Trajectory<Scalar> torso_trajectory{};

        /**
         * @brief Generate swing foot trajectory.
         * @return Trajectory of swing foot to kick.
         */
        void generate_trajectories() {
            // Clear current trajectories
            swingfoot_trajectory.clear();
            torso_trajectory.clear();

            int sign = left_foot_is_planted ? -1 : 1;

            // Add waypoints to trajectories
            for (const auto& waypoint : foot_waypoints) {
                swingfoot_trajectory.add_waypoint(X, waypoint.w(), waypoint.x(), 0);
                swingfoot_trajectory.add_waypoint(Y, waypoint.w(), sign * waypoint.y(), 0);
                swingfoot_trajectory.add_waypoint(Z, waypoint.w(), waypoint.z(), 0);
                swingfoot_trajectory.add_waypoint(ROLL, waypoint.w(), 0, 0);
                swingfoot_trajectory.add_waypoint(PITCH, waypoint.w(), 0, 0);
                swingfoot_trajectory.add_waypoint(YAW, waypoint.w(), 0, 0);
            }

            for (const auto& waypoint : torso_waypoints) {
                torso_trajectory.add_waypoint(X, waypoint.w(), waypoint.x(), 0);
                torso_trajectory.add_waypoint(Y, waypoint.w(), sign * waypoint.y(), 0);
                torso_trajectory.add_waypoint(Z, waypoint.w(), waypoint.z(), 0);
                torso_trajectory.add_waypoint(ROLL, waypoint.w(), 0, 0);
                torso_trajectory.add_waypoint(PITCH, waypoint.w(), torso_pitch, 0);
                torso_trajectory.add_waypoint(YAW, waypoint.w(), 0, 0);
            }
        }
    };
}  // namespace utility::skill
#endif
