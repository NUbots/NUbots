#ifndef MODULE_MOTION_MOTIONGENERATION_HPP
#define MODULE_MOTION_MOTIONGENERATION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "message/behaviour/state/WalkState.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/math/euler.hpp"
#include "utility/motion/splines/Trajectory.hpp"

namespace utility::skill {

    using message::behaviour::state::WalkState;
    using utility::input::LimbID;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::motion::splines::Trajectory;
    using utility::motion::splines::Waypoint;
    using utility::motion::splines::TrajectoryDimension::PITCH;
    using utility::motion::splines::TrajectoryDimension::ROLL;
    using utility::motion::splines::TrajectoryDimension::X;
    using utility::motion::splines::TrajectoryDimension::Y;
    using utility::motion::splines::TrajectoryDimension::YAW;
    using utility::motion::splines::TrajectoryDimension::Z;

    /// @brief Motion generation options.
    template <typename Scalar>
    struct MotionGenerationOptions {
        /// @brief Maximum step limits in x, y, and theta.
        Eigen::Matrix<Scalar, 3, 1> step_limits = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /// @brief Time for one complete step (in seconds).
        Scalar step_period = 0.0;

        /// @brief Step height (in meters)
        Scalar step_height = 0.0;

        /// @brief Lateral distance in meters between feet (how spread apart the feet should be)
        Scalar step_width = 0.0;

        /// @brief Ratio of the step_period where the foot should be at its highest point, between [0 1]
        Scalar step_apex_ratio = 0.0;

        /// @brief Torso height (in meters)
        Scalar torso_height = 0.0;

        /// @brief Torso pitch (in radians)
        Scalar torso_pitch = 0.0;

        /// @brief Ratio of the step_period where the torso should be at its maximum sway, between [0 1]
        Scalar torso_sway_ratio = 0.0;

        /// @brief Torso offset from the planted foot [x,y,z] (in meters), at time = torso_sway_ratio*step_period
        Eigen::Matrix<Scalar, 3, 1> torso_sway_offset = Eigen::Matrix<Scalar, 3, 1>::Zero();
    };

    template <typename Scalar>
    class MotionGeneration {
    public:
        /**
         * @brief Configure motion generation options.
         * @param options Motion generation options.
         */
        void configure(const MotionGenerationOptions<Scalar>& options) {
            step_limits       = options.step_limits;
            step_height       = options.step_height;
            step_period       = options.step_period;
            half_step_period  = step_period / 2.0;
            step_width        = options.step_width;
            step_apex_ratio   = options.step_apex_ratio;
            torso_height      = options.torso_height;
            torso_pitch       = options.torso_pitch;
            torso_sway_offset = options.torso_sway_offset;
            torso_sway_ratio  = options.torso_sway_ratio;
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
         * @brief Get whether or not the left foot is planted.
         * @return True if left foot is planted, false otherwise.
         */
        bool is_left_foot_planted() const {
            return left_foot_is_planted;
        }

        /**
         * @brief Reset walk engine.
         */
        void reset() {
            // Initialize swing foot pose
            Hps_start.translation() = Eigen::Matrix<Scalar, 3, 1>(0.0, get_foot_width_offset(), 0.0);
            Hps_start.linear().setIdentity();

            // Initialize torso pose
            Hpt_start.translation() = Eigen::Matrix<Scalar, 3, 1>(0.0, get_foot_width_offset() / 2, torso_height);
            Hpt_start.linear() =
                Eigen::AngleAxis<Scalar>(torso_pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()).toRotationMatrix();

            // Initialize swing foot and torso trajectories
            generate_swingfoot_trajectory(Eigen::Matrix<Scalar, 3, 1>::Zero());
            generate_torso_trajectory(Eigen::Matrix<Scalar, 3, 1>::Zero());

            // Start time at end of step period to avoid taking a step when starting.
            t = step_period;

            // Set engine state to stopped
            engine_state = WalkState::State::STOPPED;
        }

        /**
         * @brief Run an update of the walk engine, updating the time and engine state and trajectories.
         * @param dt Time step.
         * @param velocity_target Requested velocity target (dx, dy, dtheta).
         * @return Engine state.
         */
        WalkState::State update(const Scalar& dt, const Eigen::Matrix<Scalar, 3, 1>& velocity_target) {
            if (velocity_target.isZero() && t < step_period) {
                // Requested velocity target is zero and we haven't finished taking a step, continue stopping
                engine_state = WalkState::State::STOPPING;
            }
            else if (velocity_target.isZero() && t >= step_period) {
                // Requested velocity target is zero and we have finished taking a step, remain stopped
                engine_state = WalkState::State::STOPPED;
            }
            else {
                // Requested velocity target is non-zero, walk
                engine_state = WalkState::State::WALKING;
            }

            switch (engine_state.value) {
                case WalkState::State::WALKING:
                    update_time(dt);
                    // If we are at the end of the step, switch the planted foot and reset time
                    if (t >= step_period) {
                        switch_planted_foot();
                    }
                    break;
                case WalkState::State::STOPPING:
                    update_time(dt);
                    // We do not switch the planted foot here because we want to transition to the standing state
                    break;
                case WalkState::State::STOPPED:
                    // We do not update the time here because we want to remain in the standing state
                    break;
                default: NUClear::log<NUClear::WARN>("Unknown state"); break;
            }

            // Generate the torso and swing foot trajectories
            generate_swingfoot_trajectory(velocity_target);
            generate_torso_trajectory(velocity_target);

            return engine_state;
        }

        /**
         * @brief Get the current state of the walk engine.
         * @return Current state of the walk engine.
         */
        WalkState::State get_state() const {
            return engine_state;
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

        /// @brief Ratio of the step_period where the swing foot should be at its maximum height, between [0 1].
        Scalar step_apex_ratio = 0.0;

        /// @brief Lateral distance between feet. (how spread apart the feet should be)
        Scalar step_width = 0.0;

        /// @brief Torso height.
        Scalar torso_height = 0.0;

        /// @brief Torso pitch.
        Scalar torso_pitch = 0.0;

        /// @brief Torso offset at half step period from the planted foot.
        Eigen::Matrix<Scalar, 3, 1> torso_sway_offset = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /// @brief Ratio of the step_period where the torso should be at its maximum sway, between [0 1].
        Scalar torso_sway_ratio = 0.0;

        // ******************************** State ********************************

        /// @brief Current engine state.
        WalkState::State engine_state = WalkState::State::STOPPED;

        /// @brief Transform from planted {p} foot to swing {s} foot current placement at start of step.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hps_start =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Transform from planted {p} foot to the torso {t} at start of step.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hpt_start =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Whether the left foot is planted.
        bool left_foot_is_planted = true;

        /// @brief Current time in the step cycle [0, step_period]
        Scalar t = 0.0;

        // ******************************** Trajectories ********************************

        /// @brief 6D piecewise polynomial trajectory for swing foot.
        Trajectory<Scalar> swingfoot_trajectory;

        /// @brief 6D piecewise polynomial trajectory for torso.
        Trajectory<Scalar> torso_trajectory;

        /// @brief Get the lateral distance between feet in current planted foot frame.
        /// @return Lateral distance between feet.
        Scalar get_foot_width_offset() const {
            return left_foot_is_planted ? -step_width : step_width;
        }

        /**
         * @brief Generate swing foot trajectory.
         * @param velocity_target Requested velocity target (dx, dy, dtheta).
         * @param Hps_end Next foot placement.
         * @return Trajectory of swing foot to follow to reach next foot placement.
         */
        void generate_swingfoot_trajectory(const Eigen::Matrix<Scalar, 3, 1>& velocity_target) {
            // Clear current trajectory
            swingfoot_trajectory.clear();

            // Compute next foot placement and clamp with step limits
            Eigen::Matrix<Scalar, 3, 1> step_placement;
            step_placement.x() =
                std::max(std::min(velocity_target.x() * step_period, step_limits.x()), -step_limits.x());
            step_placement.y() =
                std::max(std::min(velocity_target.y() * step_period, step_limits.y()), -step_limits.y());
            step_placement.z() =
                std::max(std::min(velocity_target.z() * step_period, step_limits.z()), -step_limits.z());

            // Start waypoint: Start from current foot placement
            Waypoint<Scalar> start_waypoint;
            start_waypoint.time_point  = 0;
            start_waypoint.position    = Hps_start.translation();
            start_waypoint.orientation = MatrixToEulerIntrinsic(Hps_start.rotation());
            swingfoot_trajectory.add_waypoint(start_waypoint);

            // Apex waypoint: Raise foot to step height at time = step_period * step_apex_ratio
            Waypoint<Scalar> middle_waypoint;
            middle_waypoint.time_point = step_apex_ratio * step_period;
            middle_waypoint.position   = Eigen::Matrix<Scalar, 3, 1>(0, get_foot_width_offset(), step_height);
            middle_waypoint.velocity   = velocity_target;
            middle_waypoint.orientation =
                Eigen::Matrix<Scalar, 3, 1>(0, 0, velocity_target.z() * step_apex_ratio * step_period);

            swingfoot_trajectory.add_waypoint(middle_waypoint);

            // End waypoint: Place foot at next foot placement on ground at time = step_period
            Waypoint<Scalar> end_waypoint;
            end_waypoint.time_point = step_period;
            end_waypoint.position =
                Eigen::Matrix<Scalar, 3, 1>(step_placement.x(), get_foot_width_offset() + step_placement.y(), 0);
            end_waypoint.orientation = Eigen::Matrix<Scalar, 3, 1>(0, 0, step_placement.z());
            swingfoot_trajectory.add_waypoint(end_waypoint);
        }

        /**
         * @brief Generate torso trajectory.
         * @param velocity_target Requested velocity target (dx, dy, dtheta).
         * @param Hps_end Next foot placement.
         * @return Trajectory of torso to follow to reach next torso placement.
         */
        void generate_torso_trajectory(const Eigen::Matrix<Scalar, 3, 1>& velocity_target) {
            // Clear current trajectory
            torso_trajectory.clear();

            // Start of step waypoint: Start from current torso placement
            Waypoint<Scalar> start_waypoint;
            start_waypoint.time_point  = 0;
            start_waypoint.position    = Hpt_start.translation();
            start_waypoint.orientation = MatrixToEulerIntrinsic(Hpt_start.rotation());
            torso_trajectory.add_waypoint(start_waypoint);

            // Sway apex waypoint: Torso shifts to this point relative to planted foot at torso_sway_ratio * step_period
            Waypoint<Scalar> middle_waypoint;
            middle_waypoint.time_point = torso_sway_ratio * step_period;
            Scalar torso_offset_y      = left_foot_is_planted ? -torso_sway_offset.y() : torso_sway_offset.y();
            middle_waypoint.position   = Eigen::Matrix<Scalar, 3, 1>(torso_sway_offset.x(),
                                                                   torso_offset_y,
                                                                   torso_height + torso_sway_offset.z());
            middle_waypoint.velocity   = velocity_target;
            middle_waypoint.orientation =
                Eigen::Matrix<Scalar, 3, 1>(0, torso_pitch, velocity_target.z() * torso_sway_ratio * step_period);
            torso_trajectory.add_waypoint(middle_waypoint);

            // End waypoint: Torso shifts to this point relative to planted foot at step_period, positioned halfway
            // between the feet
            Waypoint<Scalar> end_waypoint;
            end_waypoint.time_point = step_period;
            end_waypoint.position =
                Eigen::Matrix<Scalar, 3, 1>(velocity_target.x() * half_step_period,
                                            get_foot_width_offset() / 2 + velocity_target.y() * half_step_period,
                                            torso_height);
            end_waypoint.orientation = Eigen::Matrix<Scalar, 3, 1>(0, torso_pitch, velocity_target.z() * step_period);
            torso_trajectory.add_waypoint(end_waypoint);
        }

        /**
         * @brief Switch planted foot.
         * @details This function switches the planted foot and updates the start torso and start swing foot poses.
         */
        void switch_planted_foot() {
            // Transform planted foot to swing foot frame at end of step
            Hps_start = get_swing_foot_pose(step_period).inverse();

            // Transform torso to end torso frame at end of step, in the new planted foot frame
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
        void update_time(const Scalar& dt) {
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
    };
}  // namespace utility::skill
#endif
