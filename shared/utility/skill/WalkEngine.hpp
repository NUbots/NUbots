/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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

/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef MODULE_MOTION_WALKENGINE_HPP
#define MODULE_MOTION_WALKENGINE_HPP

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

#include "utility/motion/splines/Footstep.hpp"
#include "utility/motion/splines/TrajectoryUtils.hpp"

using utility::motion::splines::Footstep;
using utility::motion::splines::Trajectories;
using utility::motion::splines::TrajectoryTypes;

namespace utility::skill {

    struct WalkingParameter {
        // Full walk cycle frequency
        //(in Hz, > 0)
        float freq = 0.0f;
        // Length of double support phase in half cycle
        //(ratio, [0:1])
        float double_support_ratio = 0.0f;
        // Lateral distance between the feet center
        //(in m, >= 0)
        float foot_distance = 0.0f;
        // Maximum flying foot height
        //(in m, >= 0)
        float foot_rise = 0.0f;
        // Pause of Z movement on highest point
        //(single support cycle ratio, [0,1])
        float foot_z_pause = 0.0f;
        // Let the foot's downward trajectory end above the ground
        // this is helpful if the support leg bends
        //(in m, >= 0)
        float foot_put_down_z_offset = 0.0f;
        // Phase time for moving the foot from Z offset to ground,
        // also used for X and Y since they should not move after contact to the ground
        //(phase between apex and single support end [0:1])
        float foot_put_down_phase = 0.0f;
        // Phase of flying foot apex
        //(single support cycle phase, [0:1])
        float foot_apex_phase = 0.0f;
        // Foot X/Y overshoot in ratio of step length
        //(ratio, >= 0)
        float foot_overshoot_ratio = 0.0f;
        // Foot X/Y overshoot phase
        //(single support cycle phase, [footApexPhase:1]
        float foot_overshoot_phase = 0.0f;
        // Height of the trunk from ground
        //(in m, > 0)
        float trunk_height = 0.0f;
        // Trunk pitch orientation
        //(in rad)
        float trunk_pitch = 0.0f;
        // Phase offset of trunk oscillation
        //(half cycle phase, [0:1])
        float trunk_phase = 0.0f;
        // Trunk forward offset
        //(in m)
        float trunk_x_offset = 0.0f;
        // Trunk lateral offset
        //(in m)
        float trunk_y_offset = 0.0f;
        // Trunk lateral oscillation amplitude ratio
        //(ratio, >= 0)
        float trunk_swing = 0.0f;
        // Trunk swing pause length in phase at apex
        //(half cycle ratio, [0:1])
        float trunk_pause = 0.0f;
        // Trunk forward offset proportional to forward step
        //(in 1)
        float trunk_x_offset_p_coef_forward = 0.0f;
        // Trunk forward offset proportional to rotation step
        //(in m/rad)
        float trunk_x_offset_p_coef_turn = 0.0f;
        // Trunk pitch orientation proportional to forward step
        //(in rad/m)
        float trunk_pitch_p_coef_forward = 0.0f;
        // Trunk pitch orientation proportional to rotation step
        //(in 1)
        float trunk_pitch_p_coef_turn        = 0.0f;
        float trunk_y_only_in_double_support = 0.0f;
        float kick_length                    = 0.0f;
        float kick_phase                     = 0.0f;
        float foot_put_down_roll_offset      = 0.0f;
        float kick_vel                       = 0.0f;
        float pause_duration                 = 0.0f;
        float first_step_swing_factor        = 0.0f;
    };

    enum class WalkEngineState { IDLE, PAUSED, START_MOVEMENT, START_STEP, WALKING, STOP_STEP, STOP_MOVEMENT, KICK };

    /**
     * QuinticWalkEngine
     *
     * Holonomic and open loop walk
     * generator based on footstep control
     * and quintic splines in cartesian space.
     * Expressed all target state in cartesian
     * space with respect to current support foot
     */
    class QuinticWalkEngine {
    public:
        QuinticWalkEngine();

        /**
         * Return current walk phase
         * between 0 and 1
         */
        [[nodiscard]] float get_phase() const {
            return phase;
        }

        /**
         * Return current time between
         * 0 and half period for
         * trajectories evaluation
         */
        [[nodiscard]] float get_trajs_time() const {
            return phase < 0.5f ? phase / params.freq : (phase - 0.5f) / params.freq;
        }

        /**
         * Get the footstep object.
         */
        [[nodiscard]] inline Footstep get_footstep() const {
            return foot_step;
        }

        /**
         * Return if true if left is current support foot
         */
        [[nodiscard]] bool is_left_support() const {
            return foot_step.is_left_support();
        }

        /**
         * Return true if both feet are currently on the ground
         */
        [[nodiscard]] bool is_double_support() const {
            // returns true if the value of the "is_float_support" spline is currently higher than 0.5
            // the spline should only have values of 0 or 1
            return trajs.get(TrajectoryTypes::IS_DOUBLE_SUPPORT).pos(get_trajs_time()) >= 0.5f;
        }


        /**
         * Assign given parameters vector
         */
        constexpr void set_parameters(const WalkingParameter& params_) {
            params      = params_;
            half_period = 1.0f / (2.0f * params.freq);
            foot_step.set_foot_distance(params.foot_distance);
        }

        /**
         * Update the internal walk state
         * (phase, trajectories) from given
         * elapsed time since last update() call
         */
        bool update_state(const float& dt, const Eigen::Vector3f& orders);

        // Return type for the following two functions
        using PositionSupportTuple =
            std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, bool>;
        /**
         * @brief Compute current cartesian target from trajectories
         * @return PositionSupportTuple The computed vectors with the bool indicating the support foot:
         *         {trunk_positionTarget, trunkAxisTarget, footPositionTarget, footAxisTarget, is_left_supportFoot}
         */
        [[nodiscard]] PositionSupportTuple compute_cartesian_position()
            const;  // Gets current time and calls below func
        [[nodiscard]] PositionSupportTuple compute_cartesian_position_at_time(const float& time) const;

        constexpr void request_kick(const bool& left) {
            if (left) {
                left_kick_requested  = true;
                right_kick_requested = false;
            }
            else {
                left_kick_requested  = false;
                right_kick_requested = true;
            }
        }

        constexpr void request_pause() {
            pause_requested = true;
        }

        /**
         * Ends the current step earlier. Useful if foot hits ground to early.
         */
        constexpr void end_step() {
            // ends the step earlier, e.g. when foot has already contact to ground
            phase = phase < 0.5f ? 0.5f : 0.0f;
        }

        /**
         * Completely reset the engine, e.g. when robot fell down
         */
        void reset();

        [[nodiscard]] constexpr WalkEngineState get_state() {
            return engine_state;
        }

    private:
        WalkEngineState engine_state{};

        /**
         * Current footstep support
         * and flying last and next pose
         */
        Footstep foot_step = Footstep(0.14f, true);

        /**
         * Movement phase between 0 and 1
         */
        float phase      = 0.0f;
        float last_phase = 0.0f;

        float time_paused = 0.0f;

        /**
         * Currently used parameters
         */
        WalkingParameter params{};
        float half_period = 0.0f;

        bool left_kick_requested  = false;
        bool right_kick_requested = false;
        bool pause_requested      = false;

        /**
         * Trunk pose and orientation
         * position, velocity and acceleration
         * at half cycle start
         */
        Eigen::Vector3f trunk_pos_at_last      = Eigen::Vector3f::Zero();
        Eigen::Vector3f trunk_vel_at_last      = Eigen::Vector3f::Zero();
        Eigen::Vector3f trunk_acc_at_last      = Eigen::Vector3f::Zero();
        Eigen::Vector3f trunk_axis_pos_at_last = Eigen::Vector3f::Zero();
        Eigen::Vector3f trunk_axis_vel_at_last = Eigen::Vector3f::Zero();
        Eigen::Vector3f trunk_axis_acc_at_last = Eigen::Vector3f::Zero();

        /**
         * Generated half walk
         * cycle trajectory
         */
        Trajectories trajs{};

        void update_phase(const float& dt);

        void build_trajectories(const Eigen::Vector3f& orders,
                                const bool& start_movement,
                                const bool& start_step,
                                const bool& kick_step);

        void build_start_movement_trajectories(const Eigen::Vector3f& orders) {
            build_trajectories(orders, true, false, false);
        }

        void build_start_step_trajectories(const Eigen::Vector3f& orders) {
            build_trajectories(orders, false, true, false);
        }

        void build_kick_trajectories(const Eigen::Vector3f& orders) {
            build_trajectories(orders, false, false, true);
        }

        void build_normal_trajectories(const Eigen::Vector3f& orders) {
            build_trajectories(orders, false, false, false);
        }

        void build_stop_step_trajectories(const Eigen::Vector3f& orders) {
            build_walk_disable_trajectories(orders, false);
        }

        void build_stop_movement_trajectories(const Eigen::Vector3f& orders) {
            build_walk_disable_trajectories(orders, true);
        }


        void build_walk_disable_trajectories(const Eigen::Vector3f& orders, const bool& foot_in_idle_position);

        void save_current_trunk_state();

        void use_current_trunk_state();

        void point(const TrajectoryTypes& spline,
                   const float& t,
                   const float& pos,
                   const float& vel = 0,
                   const float& acc = 0) {
            trajs.get(spline).addPoint(t, pos, vel, acc);
        }

        /**
         * Reset the trunk position and
         * orientation state vectors at last
         * half cycle as stopped pose
         */
        void reset_trunk_last_state();
    };
}  // namespace utility::skill
#endif
