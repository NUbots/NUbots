/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::motion::splines::Trajectory;
    using utility::motion::splines::Waypoint;

    template <typename Scalar>
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

    template <typename Scalar>
    using Iso3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;

    /// @brief Walk generation parameters.
    template <typename Scalar>
    struct WalkParameters {
        /// @brief Maximum step limits in x, y, and theta.
        Vec3<Scalar> step_limits = Vec3<Scalar>::Zero();

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
        Vec3<Scalar> torso_position_offset = Vec3<Scalar>::Zero();

        /// @brief Ratio of the step_period where the torso should be at its maximum sway, between [0 1]
        Scalar torso_sway_ratio = 0.0;

        /// @brief Torso offset from the planted foot [x,y,z] (in meters), at time = torso_sway_ratio*step_period
        Vec3<Scalar> torso_sway_offset = Vec3<Scalar>::Zero();

        /// @brief Ratio of where to position the torso relative to the next step position [x,y] (in meters)
        Vec3<Scalar> torso_final_position_ratio = Vec3<Scalar>::Zero();
    };

    template <typename Scalar>
    class WalkGenerator {
    public:
        /**
         * @brief Configure motion generation options.
         * @param options Motion generation options.
         */
        void set_parameters(const WalkParameters<Scalar>& params) {
            p = params;
        }

        /**
         * @brief Get the WalkParameters.
         * @return WalkParameters.
         */
        WalkParameters<Scalar> get_parameters() const {
            return p;
        }

        /**
         * @brief Get the swing foot (Hps) pose at the current time.
         * @return Trajectory of torso.
         */
        Iso3<Scalar> get_swing_foot_pose() const {
            return swingfoot_trajectory.pose(t);
        }

        /**
         * @brief Get the swing foot (Hps) pose at the given time.
         * @param t Time.
         * @return Swing foot pose at time t.
         */
        Iso3<Scalar> get_swing_foot_pose(Scalar t) const {
            return swingfoot_trajectory.pose(t);
        }

        /**
         * @brief Get the torso pose (Hpt) object at the current time.
         * @return Pose of torso.
         */
        Iso3<Scalar> get_torso_pose() const {
            return torso_trajectory.pose(t);
        }

        /**
         * @brief Get the torso pose (Hpt) object at the given time.
         * @param t Time.
         * @return Pose of torso at time t.
         */
        Iso3<Scalar> get_torso_pose(Scalar t) const {
            return torso_trajectory.pose(t);
        }

        /**
         * @brief Get the left or right foot pose at the current time in the torso {t} frame.
         * @param limb Limb ID of foot to get pose of.
         * @return Swing foot pose at time t.
         */
        Iso3<Scalar> get_foot_pose(const LimbID& limb) const {
            // Return the desired pose of the specified foot
            if (limb == LimbID::LEFT_LEG)
                return left_foot_is_planted ? get_torso_pose().inverse()
                                            : get_torso_pose().inverse() * get_swing_foot_pose();
            if (limb == LimbID::RIGHT_LEG)
                return left_foot_is_planted ? get_torso_pose().inverse() * get_swing_foot_pose()
                                            : get_torso_pose().inverse();
            throw std::runtime_error("Invalid Limb ID");
        }

        /**
         * @brief Get the left or right foot pose at the given time in the torso {t} frame.
         * @param t Time.
         * @param limb Limb ID of foot to get pose of.
         * @return Swing foot pose at time t.
         */
        Iso3<Scalar> get_foot_pose(Scalar t, const LimbID& limb) const {
            // Return the desired pose of the specified foot
            if (limb == LimbID::LEFT_LEG)
                return left_foot_is_planted ? get_torso_pose(t).inverse()
                                            : get_torso_pose(t).inverse() * get_swing_foot_pose(t);
            if (limb == LimbID::RIGHT_LEG)
                return left_foot_is_planted ? get_torso_pose(t).inverse() * get_swing_foot_pose(t)
                                            : get_torso_pose(t).inverse();
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
            Hps_start.translation() = Vec3<Scalar>(0.0, get_foot_width_offset(), 0.0);
            Hps_start.linear().setIdentity();

            // Initialize torso pose
            Hpt_start.translation() = p.torso_sway_offset + p.torso_position_offset;
            Hpt_start.linear()      = Eigen::AngleAxis<Scalar>(p.torso_pitch, Vec3<Scalar>::UnitY()).toRotationMatrix();

            // Start time at end of step period to avoid taking a step when starting.
            t = p.step_period;

            // Generate trajectories for swing foot and torso with zero velocity target
            generate_walking_trajectories(Vec3<Scalar>::Zero());

            // Set engine state to stopped
            engine_state = WalkState::State::STOPPED;
        }

        /**
         * @brief Run an update of the walk engine, updating the time and engine state and trajectories.
         * @param dt Time step.
         * @param velocity_target Requested velocity target (dx, dy, dtheta).
         * @return Engine state.
         */
        WalkState::State update(const Scalar& dt, const Vec3<Scalar>& velocity_target) {
            if (velocity_target.isZero() && t < p.step_period) {
                // Requested velocity target is zero and we haven't finished taking a step, continue stopping
                engine_state = WalkState::State::STOPPING;
            }
            else if (velocity_target.isZero() && t >= p.step_period) {
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
                    if (t >= p.step_period) {
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
        /// @brief Walk engine parameters.
        WalkParameters<Scalar> p;

        // ******************************** State ********************************

        /// @brief Current engine state.
        WalkState::State engine_state = WalkState::State::STOPPED;

        /// @brief Transform from planted {p} foot to swing {s} foot current placement at start of step.
        Iso3<Scalar> Hps_start = Iso3<Scalar>::Identity();

        /// @brief Transform from planted {p} foot to the torso {t} at start of step.
        Iso3<Scalar> Hpt_start = Iso3<Scalar>::Identity();

        /// @brief Whether the left foot is planted.
        bool left_foot_is_planted = true;

        /// @brief Current time in the step cycle [0, p.step_period]
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
            return left_foot_is_planted ? -p.step_width : p.step_width;
        }

        /**
         * @brief Generate walking trajectories for the torso and swing foot.
         * @param velocity_target Requested velocity target (dx, dy, dtheta).
         */
        void generate_walking_trajectories(const Vec3<Scalar>& velocity_target) {
            // Compute the next step placement in the planted foot frame based on the requested velocity target
            Vec3<Scalar> step = Vec3<Scalar>::Zero();
            step.x() = std::max(std::min(velocity_target.x() * p.step_period, p.step_limits.x()), -p.step_limits.x());
            step.y() = std::max(std::min(velocity_target.y() * p.step_period, p.step_limits.y()), -p.step_limits.y());
            step.z() = std::max(std::min(velocity_target.z() * p.step_period, p.step_limits.z()), -p.step_limits.z());

            // Create a waypoint variable to use for all waypoints
            Waypoint<Scalar> wp;

            // ******************************** Swing Foot Trajectory ********************************
            swingfoot_trajectory.clear();

            // Start waypoint: Start from the current swing foot position at time = 0
            wp.time_point       = 0.0;
            wp.position         = Hps_start.translation();
            wp.velocity         = Vec3<Scalar>::Zero();
            wp.orientation      = mat_to_rpy_intrinsic(Hps_start.rotation());
            wp.angular_velocity = Vec3<Scalar>::Zero();
            swingfoot_trajectory.add_waypoint(wp);

            // Middle waypoint: Raise foot to step height at time = p.step_apex_ratio * p.step_period
            wp.time_point       = p.step_apex_ratio * p.step_period;
            wp.position         = Vec3<Scalar>(0, get_foot_width_offset(), p.step_height);
            wp.velocity         = Vec3<Scalar>(velocity_target.x(), velocity_target.y(), 0);
            wp.orientation      = Vec3<Scalar>(0.0, 0.0, velocity_target.z() * p.step_apex_ratio * p.step_period);
            wp.angular_velocity = Vec3<Scalar>(0.0, 0.0, velocity_target.z());
            swingfoot_trajectory.add_waypoint(wp);

            // End waypoint: End at next foot placement on ground at time = p.step_period
            wp.time_point       = p.step_period;
            wp.position         = Vec3<Scalar>(step.x(), get_foot_width_offset() + step.y(), 0);
            wp.velocity         = Vec3<Scalar>::Zero();
            wp.orientation      = Vec3<Scalar>(0, 0, step.z());
            wp.angular_velocity = Vec3<Scalar>::Zero();
            swingfoot_trajectory.add_waypoint(wp);

            //  ******************************** Torso Trajectory ********************************
            torso_trajectory.clear();

            // Start waypoint: Start from the current torso position at time = 0
            wp.time_point       = 0.0;
            wp.position         = Hpt_start.translation();
            wp.velocity         = Vec3<Scalar>::Zero();
            wp.orientation      = mat_to_rpy_intrinsic(Hpt_start.rotation());
            wp.angular_velocity = Vec3<Scalar>::Zero();
            torso_trajectory.add_waypoint(wp);

            // Middle waypoint: Shift torso to the p.torso_sway_offset at time = p.torso_sway_ratio * p.step_period
            wp.time_point         = p.torso_sway_ratio * p.step_period;
            Scalar torso_offset_y = left_foot_is_planted ? -p.torso_sway_offset.y() : p.torso_sway_offset.y();
            wp.position =
                Vec3<Scalar>(p.torso_sway_offset.x(), torso_offset_y, p.torso_height + p.torso_sway_offset.z())
                + p.torso_position_offset;
            wp.velocity    = Vec3<Scalar>(velocity_target.x(), velocity_target.y(), 0);
            wp.orientation = Vec3<Scalar>(0.0, p.torso_pitch, velocity_target.z() * p.torso_sway_ratio * p.step_period);
            wp.angular_velocity = Vec3<Scalar>(0.0, 0.0, velocity_target.z());
            torso_trajectory.add_waypoint(wp);

            // End waypoint: Shift torso back to the final torso position at time = p.step_period
            wp.time_point = p.step_period;
            wp.position   = Vec3<Scalar>(p.torso_final_position_ratio.x() * step.x(),
                                       get_foot_width_offset() / 2 + p.torso_final_position_ratio.y() * step.y(),
                                       p.torso_height)
                          + p.torso_position_offset;
            wp.velocity         = Vec3<Scalar>::Zero();
            wp.angular_velocity = Vec3<Scalar>::Zero();
            wp.orientation      = Vec3<Scalar>(0.0, p.torso_pitch, p.torso_final_position_ratio.z() * step.z());
            torso_trajectory.add_waypoint(wp);
        }

        /**
         * @brief Switch planted foot.
         * @details This function switches the planted foot and updates the start torso and start swing foot poses.
         */
        void switch_planted_foot() {
            // Transform planted foot to swing foot frame at end of step
            Hps_start = get_swing_foot_pose(p.step_period).inverse();

            // Transform torso to end torso frame at end of step, in the new planted foot frame
            Hpt_start = Hps_start * get_torso_pose(p.step_period);

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
            if (dt > p.step_period) {
                NUClear::log<NUClear::WARN>("dt > params.p.step_period");
                return;
            }

            // Update the phase
            t += dt;

            // Clamp time to step period
            if (t >= p.step_period) {
                t = p.step_period;
            }
        }
    };
}  // namespace utility::skill
#endif
