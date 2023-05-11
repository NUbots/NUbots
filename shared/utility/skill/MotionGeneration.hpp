/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef MODULE_MOTION_MOTIONGENERATION_HPP
#define MODULE_MOTION_MOTIONGENERATION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "utility/motion/splines/Trajectory.hpp"
using utility::motion::splines::Trajectory;

using utility::motion::splines::TrajectoryDimension::PITCH;
using utility::motion::splines::TrajectoryDimension::ROLL;
using utility::motion::splines::TrajectoryDimension::X;
using utility::motion::splines::TrajectoryDimension::Y;
using utility::motion::splines::TrajectoryDimension::YAW;
using utility::motion::splines::TrajectoryDimension::Z;

namespace utility::skill {


    /// @brief Motion generation options.
    template <typename Scalar>
    struct MotionGenerationOptions {
        /// @brief Maximum step limits in x, y, and theta.
        Eigen::Matrix<Scalar, 3, 1> step_limits = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /// @brief Step period (in seconds). Time for one complete step.
        Scalar step_period = 0.0;

        /// @brief Step height.
        Scalar step_height = 0.0;

        /// @brief Lateral distance between feet. (how spread apart the feet should be)
        Scalar step_width = 0.0;

        /// @brief Torso height.
        Scalar torso_height = 0.0;

        /// @brief Torso pitch.
        Scalar torso_pitch = 0.0;
    };

    /// @brief Motion generation
    enum class MotionGenerationState { WALK, STOP };

    template <typename Scalar>
    class MotionGeneration {

    public:
        /**
         * @brief Reset foot placement.
         */
        void reset() {
            // Assume left foot is planted foot to start
            left_foot_is_planted = true;

            // Initialize planted foot placement
            Hps_start.setIdentity();
            Hps_start.translate(Eigen::Matrix<Scalar, 3, 1>(0.0, -step_width, 0.0));

            // Initialize torso placement
            Hpt_start.setIdentity();
            Hpt_start.translate(Eigen::Matrix<Scalar, 3, 1>(0.0, -step_width / 2, torso_height));
            Hpt_start.rotate(Eigen::AngleAxis<Scalar>(torso_pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()));
        }

        /**
         * @brief Configure motion generation options.
         * @param options Motion generation options.
         */
        void configure(const MotionGenerationOptions<Scalar>& options) {
            step_limits      = options.step_limits;
            step_height      = options.step_height;
            step_period      = options.step_period;
            half_step_period = step_period / 2.0;
            step_width       = options.step_width;
            torso_height     = options.torso_height;
            torso_pitch      = options.torso_pitch;

            // Initialize foot and torso placement
            reset();
        }

        /// @brief Get the lateral distance between feet in current planted foot frame.
        /// @return Lateral distance between feet.
        Scalar get_foot_width_offset() const {
            return left_foot_is_planted ? -step_width : step_width;
        }

        /**
         * @brief Generate swing foot trajectory.
         * @param walk_command Walk command (dx, dy, dtheta).
         * @param Hps_end Next foot placement.
         * @return Trajectory of swing foot to follow to reach next foot placement.
         */
        void generate_swingfoot_trajectory(const Eigen::Matrix<Scalar, 3, 1>& walk_command) {
            // Clear current trajectory
            swingfoot_trajectory.clear();

            // X position trajectory
            swingfoot_trajectory.add_waypoint(X, 0, Hps_start.translation().x(), 0);
            swingfoot_trajectory.add_waypoint(X, half_step_period, 0, walk_command.x());
            swingfoot_trajectory.add_waypoint(X, step_period, walk_command.x() * step_period, 0);

            // Y position trajectory
            swingfoot_trajectory.add_waypoint(Y, 0, Hps_start.translation().y(), 0);
            swingfoot_trajectory.add_waypoint(Y, half_step_period, get_foot_width_offset(), walk_command.y());
            swingfoot_trajectory.add_waypoint(Y,
                                              step_period,
                                              get_foot_width_offset() + walk_command.y() * step_period,
                                              0);

            // Z position trajectory
            swingfoot_trajectory.add_waypoint(Z, 0, Hps_start.translation().z(), 0);
            swingfoot_trajectory.add_waypoint(Z, half_step_period, step_height, 0);
            swingfoot_trajectory.add_waypoint(Z, step_period, 0, 0);

            // Roll trajectory
            swingfoot_trajectory.add_waypoint(ROLL, 0, 0, 0);
            swingfoot_trajectory.add_waypoint(ROLL, half_step_period, 0, 0);
            swingfoot_trajectory.add_waypoint(ROLL, step_period, 0, 0);

            // Pitch trajectory
            swingfoot_trajectory.add_waypoint(PITCH, 0, 0, 0);
            swingfoot_trajectory.add_waypoint(PITCH, half_step_period, 0, 0);
            swingfoot_trajectory.add_waypoint(PITCH, step_period, 0, 0);

            // Yaw trajectory
            swingfoot_trajectory.add_waypoint(YAW, 0, 0, 0);
            swingfoot_trajectory.add_waypoint(YAW, half_step_period, walk_command.z() * half_step_period, 0);
            swingfoot_trajectory.add_waypoint(YAW, step_period, walk_command.z() * step_period, 0);
        }

        /**
         * @brief Generate torso trajectory.
         * @param walk_command Walk command (dx, dy, dtheta).
         * @param Hps_end Next foot placement.
         * @return Trajectory of torso to follow to reach next torso placement.
         */
        void generate_torso_trajectory(const Eigen::Matrix<Scalar, 3, 1>& walk_command) {
            // Clear current trajectory
            torso_trajectory.clear();

            // X position trajectory
            torso_trajectory.add_waypoint(X, 0, Hpt_start.translation().x(), 0);
            torso_trajectory.add_waypoint(X, half_step_period, 0, walk_command.x());
            torso_trajectory.add_waypoint(X, step_period, walk_command.x() * half_step_period, 0);

            // Y position trajectory
            torso_trajectory.add_waypoint(Y, 0, Hpt_start.translation().y(), 0);
            torso_trajectory.add_waypoint(Y, half_step_period, 0, walk_command.y());
            torso_trajectory.add_waypoint(Y,
                                          step_period,
                                          get_foot_width_offset() / 2 + walk_command.y() * half_step_period,
                                          0);

            // Z position trajectory
            torso_trajectory.add_waypoint(Z, 0, Hpt_start.translation().z(), 0);
            torso_trajectory.add_waypoint(Z, half_step_period, torso_height, 0);
            torso_trajectory.add_waypoint(Z, step_period, torso_height, 0);

            // Roll trajectory
            torso_trajectory.add_waypoint(ROLL, 0, 0, 0);
            torso_trajectory.add_waypoint(ROLL, half_step_period, 0, 0);
            torso_trajectory.add_waypoint(ROLL, step_period, 0, 0);

            // Pitch trajectory
            torso_trajectory.add_waypoint(PITCH, 0, torso_pitch, 0);
            torso_trajectory.add_waypoint(PITCH, half_step_period, torso_pitch, 0);
            torso_trajectory.add_waypoint(PITCH, step_period, torso_pitch, 0);

            // Yaw trajectory
            torso_trajectory.add_waypoint(YAW, 0, Hpt_start.rotation().eulerAngles(0, 1, 2).z(), 0);
            torso_trajectory.add_waypoint(YAW, half_step_period, walk_command.z() * half_step_period, 0);
            torso_trajectory.add_waypoint(YAW, step_period, walk_command.z() * step_period, 0);
        }

        /**
         * @brief Get the current swing foot trajectory.
         * @return Trajectory of swing foot.
         */
        Trajectory<Scalar> get_swingfoot_trajectory() const {
            return swingfoot_trajectory;
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
         * @brief Get the torso trajectory
         * @return Trajectory of torso.
         */
        Trajectory<Scalar> get_torso_trajectory() const {
            return torso_trajectory;
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
         * @brief Get whether the left foot is planted.
         * @return True if left foot is planted, false otherwise.
         */
        bool is_left_foot_planted() const {
            return left_foot_is_planted;
        }

        /**
         * @brief Switch planted foot.
         * @details This function switches the planted foot and updates the start torso and start swing foot poses.
         */
        void switch_planted_foot() {
            // Transform planted foot into swing foot frame at next foot placement
            Hps_start = get_swing_foot_pose(step_period).inverse();

            // Transform torso into end torso frame at end foot placement
            Hpt_start = Hps_start * get_torso_pose(step_period);

            // Switch planted foot indicator
            left_foot_is_planted = !left_foot_is_planted;

            // Reset time
            t = 0;
        }

        /**
         * @brief Update the time.
         * @param dt Time step.
         */
        void update_time(const float& dt) {
            // Check for negative time step
            if (dt <= 0.0f) {
                NUClear::log<NUClear::WARN>("dt <= 0.0f");
                return;
            }
            // Check for too long dt
            if (dt > step_period) {
                NUClear::log<NUClear::WARN>("dt > params.step_period");
                return;
            }

            // Update the phase
            t += dt;

            // Clamp time to step period
            if (t >= step_period) {
                t = step_period;
            }
        }

        /**
         * @brief Update the state of the walk engine.
         * @param dt Time step.
         * @param walk_command Walk command (dx, dy, dtheta).
         */
        void update_state(const Scalar& dt, const Eigen::Matrix<Scalar, 3, 1>& walk_command) {
            const bool walk_command_zero = walk_command.isZero();

            if (walk_command_zero) {
                engine_state = MotionGenerationState::STOP;
            }
            else {
                engine_state = MotionGenerationState::WALK;
            }

            switch (engine_state) {
                case MotionGenerationState::WALK:
                    update_time(dt);
                    // If we are at the end of the step, switch the planted foot and reset time
                    if (t >= step_period) {
                        switch_planted_foot();
                    }
                    generate_swingfoot_trajectory(walk_command);
                    generate_torso_trajectory(walk_command);
                    break;
                case MotionGenerationState::STOP:
                    update_time(dt);
                    generate_swingfoot_trajectory(walk_command);
                    generate_torso_trajectory(walk_command);
                    break;
                default: NUClear::log<NUClear::WARN>("Unknown state"); break;
            }
        }

    private:
        // ******************************** Options ********************************

        /// @brief Maximum step limits in x, y, and theta.
        Eigen::Matrix<Scalar, 3, 1> step_limits = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /// @brief Step height.
        Scalar step_height = 0.0;

        /// @brief Step period (in seconds). Time for one complete step.
        Scalar step_period = 0.0;

        /// @brief Half of step period (in seconds).
        Scalar half_step_period = 0.0;

        /// @brief Lateral distance between feet. (how spread apart the feet should be)
        Scalar step_width = 0.0;

        /// @brief Torso height.
        Scalar torso_height = 0.0;

        /// @brief Torso pitch.
        Scalar torso_pitch = 0.0;

        // ******************************** State ********************************

        /// @brief Current engine state.
        MotionGenerationState engine_state{};

        /// @brief Transform from planted {p} foot to swing {s} foot current placement at start of step.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hps_start =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Transform from planted {p} foot to swing {s} foot current placement at midpoint of step.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hps_midpoint =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Transform from planted {p} foot to swing {s} foot current placement at end of step.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hps_end = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Transform from planted {p} foot to the torso {t} at start of step.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hpt_start =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Transform from planted {p} foot to the torso {t} at midpoint of step.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hpt_midpoint =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Transform from planted {p} foot to the torso {t} at end of step.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hpt_end = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Whether the left foot is planted.
        bool left_foot_is_planted = true;

        /// @brief Current time in the step cycle [0, step_period]
        Scalar t = 0.0;

        // ******************************** Trajectories ********************************

        // 6D piecewise polynomial trajectory for swing foot.
        Trajectory<Scalar> swingfoot_trajectory;

        // 6D piecewise polynomial trajectory for torso.
        Trajectory<Scalar> torso_trajectory;

        // 6D piecewise polynomial trajectory for center of mass.
        Trajectory<Scalar> com_trajectory;
    };
}  // namespace utility::skill
#endif
