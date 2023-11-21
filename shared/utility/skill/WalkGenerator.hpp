#ifndef MODULE_MOTION_WALKGENERATOR_HPP
#define MODULE_MOTION_WALKGENERATOR_HPP

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

    /// @brief Motion generation options.
    template <typename Scalar>
    struct WalkGeneratorOptions {
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

        /// @brief Torso constant position offset [x,y,z] (in meters)
        Eigen::Matrix<Scalar, 3, 1> torso_position_offset = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /// @brief Ratio of the step_period where the torso should be at its maximum sway, between [0 1]
        Scalar torso_sway_ratio = 0.0;

        /// @brief Torso offset from the planted foot [x,y,z] (in meters), at time = torso_sway_ratio*step_period
        Eigen::Matrix<Scalar, 3, 1> torso_sway_offset = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /// @brief Ratio of where to position the torso relative to the next step position [x,y] (in meters)
        Eigen::Matrix<Scalar, 3, 1> torso_final_position_ratio = Eigen::Matrix<Scalar, 3, 1>::Zero();
    };

    template <typename Scalar>
    class WalkGenerator {
    public:
        /**
         * @brief Configure motion generation options.
         * @param options Motion generation options.
         */
        void configure(const WalkGeneratorOptions<Scalar>& options) {
            step_limits                = options.step_limits;
            step_height                = options.step_height;
            step_period                = options.step_period;
            step_width                 = options.step_width;
            step_apex_ratio            = options.step_apex_ratio;
            torso_height               = options.torso_height;
            torso_pitch                = options.torso_pitch;
            torso_position_offset      = options.torso_position_offset;
            torso_sway_ratio           = options.torso_sway_ratio;
            torso_sway_offset          = options.torso_sway_offset;
            torso_final_position_ratio = options.torso_final_position_ratio;
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
         * @brief Get the torso velocity at the current time.
         * @param t Time.
         * @return Pose of torso at time t.
         */
        Eigen::Matrix<Scalar, 6, 1> get_torso_velocity() const {
            return torso_trajectory.velocity(t);
        }

        /**
         * @brief Get the torso velocity at the given time.
         * @param t Time.
         * @return Pose of torso at time t.
         */
        Eigen::Matrix<Scalar, 6, 1> get_torso_velocity(Scalar t) const {
            return torso_trajectory.velocity(t);
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
         * @brief Reset walk engine time and initial poses.
         */
        void reset() {
            // Initialize swing foot pose
            Hps_start.translation() = Eigen::Matrix<Scalar, 3, 1>(0.0, get_foot_width_offset(), 0.0);
            Hps_start.linear().setIdentity();

            // Initialize torso pose
            Hpt_start.translation() = torso_sway_offset + torso_position_offset;
            Hpt_start.linear() =
                Eigen::AngleAxis<Scalar>(torso_pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()).toRotationMatrix();

            // Start time at end of step period to avoid taking a step when starting.
            t = step_period;

            // Generate trajectories for swing foot and torso with zero velocity target
            generate_walking_trajectories(Eigen::Matrix<Scalar, 3, 1>::Zero());

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

            // Generate the torso and swing foot trajectories to reach the next foot step location
            generate_walking_trajectories(velocity_target);

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

        /// @brief Ratio of the step_period where the swing foot should be at its maximum height, between [0 1].
        Scalar step_apex_ratio = 0.0;

        /// @brief Lateral distance between feet. (how spread apart the feet should be)
        Scalar step_width = 0.0;

        /// @brief Torso height.
        Scalar torso_height = 0.0;

        /// @brief Torso pitch.
        Scalar torso_pitch = 0.0;

        /// @brief Torso constant position offset [x,y,z] (in meters)
        Eigen::Matrix<Scalar, 3, 1> torso_position_offset = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /// @brief Torso offset at half step period from the planted foot, at time = step_apex_ratio * step_period
        Eigen::Matrix<Scalar, 3, 1> torso_sway_offset = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /// @brief Ratio of the step_period where the torso should be at its maximum sway, between [0 1].
        Scalar torso_sway_ratio = 0.0;

        /// @brief Ratio of where to position the torso relative to the next step position [x,y] (in meters)
        Eigen::Matrix<Scalar, 3, 1> torso_final_position_ratio = Eigen::Matrix<Scalar, 3, 1>::Zero();

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
        Trajectory<Scalar> swingfoot_trajectory{};

        /// @brief 6D piecewise polynomial trajectory for torso.
        Trajectory<Scalar> torso_trajectory{};

        /**
         * @brief Get the lateral distance between feet in current planted foot frame.
         * @return Lateral distance between feet.
         */
        Scalar get_foot_width_offset() const {
            return left_foot_is_planted ? -step_width : step_width;
        }

        /**
         * @brief Generate walking trajectories for the torso and swing foot.
         * @param velocity_target Requested velocity target (dx, dy, dtheta).
         */
        void generate_walking_trajectories(const Eigen::Matrix<Scalar, 3, 1>& velocity_target) {
            // Compute the next step placement in the planted foot frame based on the requested velocity target
            Eigen::Matrix<Scalar, 3, 1> step = Eigen::Matrix<Scalar, 3, 1>::Zero();
            step.x() = std::max(std::min(velocity_target.x() * step_period, step_limits.x()), -step_limits.x());
            step.y() = std::max(std::min(velocity_target.y() * step_period, step_limits.y()), -step_limits.y());
            step.z() = std::max(std::min(velocity_target.z() * step_period, step_limits.z()), -step_limits.z());

            // Create a waypoint variable to use for all waypoints
            Waypoint<Scalar> waypoint;

            // ******************************** Swing Foot Trajectory ********************************
            swingfoot_trajectory.clear();

            // Start waypoint: Start from the current swing foot position at time = 0
            waypoint.time_point       = 0.0;
            waypoint.position         = Hps_start.translation();
            waypoint.velocity         = Eigen::Matrix<Scalar, 3, 1>::Zero();
            waypoint.orientation      = MatrixToEulerIntrinsic(Hps_start.rotation());
            waypoint.angular_velocity = Eigen::Matrix<Scalar, 3, 1>::Zero();
            swingfoot_trajectory.add_waypoint(waypoint);

            // Middle waypoint: Raise foot to step height at time = step_apex_ratio * step_period
            waypoint.time_point = step_apex_ratio * step_period;
            waypoint.position   = Eigen::Matrix<Scalar, 3, 1>(0, get_foot_width_offset(), step_height);
            waypoint.velocity   = Eigen::Matrix<Scalar, 3, 1>(velocity_target.x(), velocity_target.y(), 0);
            waypoint.orientation =
                Eigen::Matrix<Scalar, 3, 1>(0.0, 0.0, velocity_target.z() * step_apex_ratio * step_period);
            waypoint.angular_velocity = Eigen::Matrix<Scalar, 3, 1>(0.0, 0.0, velocity_target.z());
            swingfoot_trajectory.add_waypoint(waypoint);

            // End waypoint: End at next foot placement on ground at time = step_period
            waypoint.time_point       = step_period;
            waypoint.position         = Eigen::Matrix<Scalar, 3, 1>(step.x(), get_foot_width_offset() + step.y(), 0);
            waypoint.velocity         = Eigen::Matrix<Scalar, 3, 1>::Zero();
            waypoint.orientation      = Eigen::Matrix<Scalar, 3, 1>(0, 0, step.z());
            waypoint.angular_velocity = Eigen::Matrix<Scalar, 3, 1>::Zero();
            swingfoot_trajectory.add_waypoint(waypoint);

            //  ******************************** Torso Trajectory ********************************
            torso_trajectory.clear();

            // Start waypoint: Start from the current torso position at time = 0
            waypoint.time_point       = 0.0;
            waypoint.position         = Hpt_start.translation();
            waypoint.velocity         = Eigen::Matrix<Scalar, 3, 1>::Zero();
            waypoint.orientation      = MatrixToEulerIntrinsic(Hpt_start.rotation());
            waypoint.angular_velocity = Eigen::Matrix<Scalar, 3, 1>::Zero();
            torso_trajectory.add_waypoint(waypoint);

            // Middle waypoint: Shift torso to the torso_sway_offset at time = torso_sway_ratio * step_period
            waypoint.time_point   = torso_sway_ratio * step_period;
            Scalar torso_offset_y = left_foot_is_planted ? -torso_sway_offset.y() : torso_sway_offset.y();
            waypoint.position =
                Eigen::Matrix<Scalar, 3, 1>(torso_sway_offset.x(), torso_offset_y, torso_height + torso_sway_offset.z())
                + torso_position_offset;
            waypoint.velocity = Eigen::Matrix<Scalar, 3, 1>(velocity_target.x(), velocity_target.y(), 0);
            waypoint.orientation =
                Eigen::Matrix<Scalar, 3, 1>(0.0, torso_pitch, velocity_target.z() * torso_sway_ratio * step_period);
            waypoint.angular_velocity = Eigen::Matrix<Scalar, 3, 1>(0.0, 0.0, velocity_target.z());
            torso_trajectory.add_waypoint(waypoint);

            // End waypoint: Shift torso back to the final torso position at time = step_period
            waypoint.time_point = step_period;
            waypoint.position =
                Eigen::Matrix<Scalar, 3, 1>(torso_final_position_ratio.x() * step.x(),
                                            get_foot_width_offset() / 2 + torso_final_position_ratio.y() * step.y(),
                                            torso_height)
                + torso_position_offset;
            waypoint.velocity         = Eigen::Matrix<Scalar, 3, 1>::Zero();
            waypoint.angular_velocity = Eigen::Matrix<Scalar, 3, 1>::Zero();
            waypoint.orientation =
                Eigen::Matrix<Scalar, 3, 1>(0.0, torso_pitch, torso_final_position_ratio.z() * step.z());
            torso_trajectory.add_waypoint(waypoint);
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
