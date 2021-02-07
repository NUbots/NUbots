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

namespace module {
namespace motion {

    struct WalkingParameter {
        // Full walk cycle frequency
        //(in Hz, > 0)
        float freq;
        // Length of double support phase in half cycle
        //(ratio, [0:1])
        float double_support_ratio;
        // Lateral distance between the feet center
        //(in m, >= 0)
        float foot_distance;
        // Maximum flying foot height
        //(in m, >= 0)
        float foot_rise;
        // Pause of Z movement on highest point
        //(single support cycle ratio, [0,1])
        float foot_z_pause;
        // Let the foot's downward trajectory end above the ground
        // this is helpful if the support leg bends
        //(in m, >= 0)
        float foot_put_down_z_offset;
        // Phase time for moving the foot from Z offset to ground,
        // also used for X and Y since they should not move after contact to the ground
        //(phase between apex and single support end [0:1])
        float foot_put_down_phase;
        // Phase of flying foot apex
        //(single support cycle phase, [0:1])
        float foot_apex_phase;
        // Foot X/Y overshoot in ratio of step length
        //(ratio, >= 0)
        float foot_overshoot_ratio;
        // Foot X/Y overshoot phase
        //(single support cycle phase, [footApexPhase:1]
        float foot_overshoot_phase;
        // Height of the trunk from ground
        //(in m, > 0)
        float trunk_height;
        // Trunk pitch orientation
        //(in rad)
        float trunk_pitch;
        // Phase offset of trunk oscillation
        //(half cycle phase, [0:1])
        float trunk_phase;
        // Trunk forward offset
        //(in m)
        float trunk_x_offset;
        // Trunk lateral offset
        //(in m)
        float trunk_y_offset;
        // Trunk lateral oscillation amplitude ratio
        //(ratio, >= 0)
        float trunk_swing;
        // Trunk swing pause length in phase at apex
        //(half cycle ratio, [0:1])
        float trunk_pause;
        // Trunk forward offset proportional to forward step
        //(in 1)
        float trunk_x_offset_p_coef_forward;
        // Trunk forward offset proportional to rotation step
        //(in m/rad)
        float trunk_x_offset_p_coef_turn;
        // Trunk pitch orientation proportional to forward step
        //(in rad/m)
        float trunk_pitch_p_coef_forward;
        // Trunk pitch orientation proportional to rotation step
        //(in 1)
        float trunk_pitch_p_coef_turn;
        float trunk_y_only_in_double_support;
        float kick_length;
        float kick_phase;
        float foot_put_down_roll_offset;
        float kick_vel;
        float pause_duration;
        float first_step_swing_factor;
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
        /**
         * Initialization
         */
        QuinticWalkEngine();

        /**
         * Return current walk phase
         * between 0 and 1
         */
        float getPhase() const {
            return phase;
        }

        /**
         * Return current time between
         * 0 and half period for
         * trajectories evaluation
         */
        float getTrajsTime() const {
            return phase < 0.5f ? phase / params.freq : (phase - 0.5f) / params.freq;
        }

        /**
         * Get the footstep object.
         */
        Footstep getFootstep() {
            return foot_step;
        }

        /**
         * Return if true if left is current support foot
         */
        bool isLeftSupport() {
            return foot_step.isLeftSupport();
        }

        /**
         * Return true if both feet are currently on the ground
         */
        bool isDoubleSupport() {
            // returns true if the value of the "is_float_support" spline is currently higher than 0.5
            // the spline should only have values of 0 or 1
            return trajs.get(TrajectoryTypes::IS_DOUBLE_SUPPORT).pos(getTrajsTime()) >= 0.5f;
        }


        /**
         * Assign given parameters vector
         */
        void setParameters(const WalkingParameter& params) {
            this->params      = params;
            this->half_period = 1.0f / (2.0f * this->params.freq);
            foot_step.setFootDistance(this->params.foot_distance);
        }

        /**
         * Update the internal walk state
         * (phase, trajectories) from given
         * elapsed time since last update() call
         */
        bool updateState(const float dt, const Eigen::Vector3f& orders);

        /**
         * Compute current cartesian
         * target from trajectories and assign
         * it to given model through inverse
         * kinematics.
         * Return false is the target is
         * unreachable.
         */
        void computeCartesianPosition(Eigen::Vector3f& trunkPos,
                                      Eigen::Vector3f& trunkAxis,
                                      Eigen::Vector3f& footPos,
                                      Eigen::Vector3f& footAxis,
                                      bool isLeftsupportFoot);

        void computeCartesianPositionAtTime(Eigen::Vector3f& trunkPos,
                                            Eigen::Vector3f& trunkAxis,
                                            Eigen::Vector3f& footPos,
                                            Eigen::Vector3f& footAxis,
                                            bool isLeftsupportFoot,
                                            float time);

        void requestKick(const bool left) {
            if (left) {
                left_kick_requested  = true;
                right_kick_requested = false;
            }
            else {
                left_kick_requested  = false;
                right_kick_requested = true;
            }
        }

        void requestPause() {
            pause_requested = true;
        }

        /**
         * Ends the current step earlier. Useful if foot hits ground to early.
         */
        void endStep() {
            // ends the step earlier, e.g. when foot has already contact to ground
            phase = phase < 0.5f ? 0.5f : 0.0f;
        }

        /**
         * Completely reset the engine, e.g. when robot fell down
         */
        void reset();

        WalkEngineState getState() {
            return engine_state;
        }

    private:
        WalkEngineState engine_state;

        /**
         * Current footstep support
         * and flying last and next pose
         */
        Footstep foot_step;

        /**
         * Movement phase between 0 and 1
         */
        float phase;
        float last_phase;

        float time_paused;

        /**
         * Currently used parameters
         */
        WalkingParameter params;
        float half_period;

        bool left_kick_requested;
        bool right_kick_requested;
        bool pause_requested;

        /**
         * Trunk pose and orientation
         * position, velocity and acceleration
         * at half cycle start
         */
        Eigen::Vector3f trunk_pos_at_last;
        Eigen::Vector3f trunk_vel_at_last;
        Eigen::Vector3f trunk_acc_at_last;
        Eigen::Vector3f trunk_axis_pos_at_last;
        Eigen::Vector3f trunk_axis_vel_at_last;
        Eigen::Vector3f trunk_axis_acc_at_last;

        /**
         * Generated half walk
         * cycle trajectory
         */
        Trajectories trajs;

        void updatePhase(const float dt);

        void buildNormalTrajectories(const Eigen::Vector3f& orders) {
            buildTrajectories(orders, false, false, false);
        }

        void buildKickTrajectories(const Eigen::Vector3f& orders) {
            buildTrajectories(orders, false, false, true);
        }

        void buildStartTrajectories(const Eigen::Vector3f& orders) {
            buildTrajectories(orders, true, false, false);
        }

        void buildStopStepTrajectories(const Eigen::Vector3f& orders) {
            buildWalkDisableTrajectories(orders, false);
        }

        void buildStopMovementTrajectories(const Eigen::Vector3f& orders) {
            buildWalkDisableTrajectories(orders, true);
        }
        void buildTrajectories(const Eigen::Vector3f& orders,
                               const bool startMovement,
                               const bool startStep,
                               const bool kickStep);

        void buildWalkDisableTrajectories(const Eigen::Vector3f& orders, const bool footInIdlePosition);

        void saveCurrentTrunkState();

        void useCurrentTrunkState();

        void point(const TrajectoryTypes& spline,
                   const float t,
                   const float pos,
                   const float vel = 0,
                   const float acc = 0) {
            trajs.get(spline).addPoint(t, pos, vel, acc);
        }

        /**
         * Reset the trunk position and
         * orientation state vectors at last
         * half cycle as stopped pose
         */
        void resetTrunkLastState();
    };

}  // namespace motion
}  // namespace module
#endif
