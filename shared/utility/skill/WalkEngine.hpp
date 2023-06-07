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
        double freq = 0.0;
        // Length of double support phase in half cycle
        //(ratio, [0:1])
        double double_support_ratio = 0.0;
        // Lateral distance between the feet center
        //(in m, >= 0)
        double foot_distance = 0.0;
        // Maximum flying foot height
        //(in m, >= 0)
        double foot_rise = 0.0;
        // Pause of Z movement on highest point
        //(single support cycle ratio, [0,1])
        double foot_z_pause = 0.0;
        // Let the foot's downward trajectory end above the ground
        // this is helpful if the support leg bends
        //(in m, >= 0)
        double foot_put_down_z_offset = 0.0;
        // Phase time for moving the foot from Z offset to ground,
        // also used for X and Y since they should not move after contact to the ground
        //(phase between apex and single support end [0:1])
        double foot_put_down_phase = 0.0;
        // Phase of flying foot apex
        //(single support cycle phase, [0:1])
        double foot_apex_phase = 0.0;
        // Foot X/Y overshoot in ratio of step length
        //(ratio, >= 0)
        double foot_overshoot_ratio = 0.0;
        // Foot X/Y overshoot phase
        //(single support cycle phase, [footApexPhase:1]
        double foot_overshoot_phase = 0.0;
        // Height of the trunk from ground
        //(in m, > 0)
        double trunk_height = 0.0;
        // Trunk pitch orientation
        //(in rad)
        double trunk_pitch = 0.0;
        // Phase offset of trunk oscillation
        //(half cycle phase, [0:1])
        double trunk_phase = 0.0;
        // Trunk forward offset
        //(in m)
        double trunk_x_offset = 0.0;
        // Trunk lateral offset
        //(in m)
        double trunk_y_offset = 0.0;
        // Trunk lateral oscillation amplitude ratio
        //(ratio, >= 0)
        double trunk_swing = 0.0;
        // Trunk swing pause length in phase at apex
        //(half cycle ratio, [0:1])
        double trunk_pause = 0.0;
        // Trunk forward offset proportional to forward step
        //(in 1)
        double trunk_x_offset_p_coef_forward = 0.0;
        // Trunk forward offset proportional to rotation step
        //(in m/rad)
        double trunk_x_offset_p_coef_turn = 0.0;
        // Trunk pitch orientation proportional to forward step
        //(in rad/m)
        double trunk_pitch_p_coef_forward = 0.0;
        // Trunk pitch orientation proportional to rotation step
        //(in 1)
        double trunk_pitch_p_coef_turn        = 0.0;
        double trunk_y_only_in_double_support = 0.0;
        double kick_length                    = 0.0;
        double kick_phase                     = 0.0;
        double foot_put_down_roll_offset      = 0.0;
        double kick_vel                       = 0.0;
        double pause_duration                 = 0.0;
        double first_step_swing_factor        = 0.0;
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
        [[nodiscard]] double get_phase() const {
            return phase;
        }

        /**
         * Return current time between
         * 0 and half period for
         * trajectories evaluation
         */
        [[nodiscard]] double get_trajs_time() const {
            return phase < 0.5 ? phase / params.freq : (phase - 0.5) / params.freq;
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
            // returns true if the value of the "is_double_support" spline is currently higher than 0.5
            // the spline should only have values of 0 or 1
            return trajs.get(TrajectoryTypes::IS_DOUBLE_SUPPORT).pos(get_trajs_time()) >= 0.5;
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
        bool update_state(const double& dt, const Eigen::Vector3d& orders);

        // Return type for the following two functions
        using PositionSupportTuple =
            std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, bool>;
        /**
         * @brief Compute current cartesian target from trajectories
         * @return PositionSupportTuple The computed vectors with the bool indicating the support foot:
         *         {trunk_positionTarget, trunkAxisTarget, footPositionTarget, footAxisTarget, is_left_supportFoot}
         */
        [[nodiscard]] PositionSupportTuple compute_cartesian_position()
            const;  // Gets current time and calls below func
        [[nodiscard]] PositionSupportTuple compute_cartesian_position_at_time(const double& time) const;

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
            phase = phase < 0.5 ? 0.5 : 0.0;
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
        Footstep foot_step = Footstep(0.14, true);

        /**
         * Movement phase between 0 and 1
         */
        double phase      = 0.0;
        double last_phase = 0.0;

        double time_paused = 0.0;

        /**
         * Currently used parameters
         */
        WalkingParameter params{};
        double half_period = 0.0;

        bool left_kick_requested  = false;
        bool right_kick_requested = false;
        bool pause_requested      = false;

        /**
         * Trunk pose and orientation
         * position, velocity and acceleration
         * at half cycle start
         */
        Eigen::Vector3d trunk_pos_at_last      = Eigen::Vector3d::Zero();
        Eigen::Vector3d trunk_vel_at_last      = Eigen::Vector3d::Zero();
        Eigen::Vector3d trunk_acc_at_last      = Eigen::Vector3d::Zero();
        Eigen::Vector3d trunk_axis_pos_at_last = Eigen::Vector3d::Zero();
        Eigen::Vector3d trunk_axis_vel_at_last = Eigen::Vector3d::Zero();
        Eigen::Vector3d trunk_axis_acc_at_last = Eigen::Vector3d::Zero();

        /**
         * Generated half walk
         * cycle trajectory
         */
        Trajectories trajs{};

        void update_phase(const double& dt);

        void build_trajectories(const Eigen::Vector3d& orders,
                                const bool& start_movement,
                                const bool& start_step,
                                const bool& kick_step);

        void build_start_movement_trajectories(const Eigen::Vector3d& orders) {
            build_trajectories(orders, true, false, false);
        }

        void build_start_step_trajectories(const Eigen::Vector3d& orders) {
            build_trajectories(orders, false, true, false);
        }

        void build_kick_trajectories(const Eigen::Vector3d& orders) {
            build_trajectories(orders, false, false, true);
        }

        void build_normal_trajectories(const Eigen::Vector3d& orders) {
            build_trajectories(orders, false, false, false);
        }

        void build_stop_step_trajectories(const Eigen::Vector3d& orders) {
            build_walk_disable_trajectories(orders, false);
        }

        void build_stop_movement_trajectories(const Eigen::Vector3d& orders) {
            build_walk_disable_trajectories(orders, true);
        }


        void build_walk_disable_trajectories(const Eigen::Vector3d& orders, const bool& foot_in_idle_position);

        void save_current_trunk_state();

        void use_current_trunk_state();

        void point(const TrajectoryTypes& spline,
                   const double& t,
                   const double& pos,
                   const double& vel = 0,
                   const double& acc = 0) {
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
