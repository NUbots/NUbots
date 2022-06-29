/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "WalkEngine.hpp"

#include <cmath>
#include <fmt/format.h>
#include <iostream>
#include <nuclear>

#include "utility/math/angle.hpp"
#include "utility/math/euler.hpp"

namespace module::motion {

    QuinticWalkEngine::QuinticWalkEngine() {
        trajectoriesInit(trajs);
    }

    bool QuinticWalkEngine::updateState(const float& dt, const Eigen::Vector3f& orders) {
        const bool ordersZero = orders.isZero();
        // First check if we are currently in pause state or idle, since we don't want to update the phase in this
        // case
        if (engine_state == WalkEngineState::PAUSED) {
            if (time_paused > params.pause_duration) {
                // our pause is finished, see if we can continue walking
                if (pause_requested) {
                    // not yet, wait another pause duration
                    pause_requested = false;
                    time_paused     = 0.0f;
                    return false;
                }
                // we can continue
                engine_state = WalkEngineState::WALKING;
                time_paused  = 0.0f;
            }
            else {
                time_paused += dt;
                return false;
            }
            // we don't have to update anything more
        }
        else if (engine_state == WalkEngineState::IDLE) {
            if (ordersZero) {
                // we are in idle and don't have orders. current state is fine, just do nothing
                return false;
            }
        }

        // update the current phase
        updatePhase(dt);

        // check if we will finish a half step with this update
        const bool halfStepFinished = (last_phase < 0.5f && phase >= 0.5f) || (last_phase > 0.5f && phase < 0.5f);

        // small state machine
        switch (engine_state) {
            case WalkEngineState::IDLE:
                // state is idle and orders are not zero, we can start walking
                buildStartMovementTrajectories(orders);
                engine_state = WalkEngineState::START_MOVEMENT;
                break;
            case WalkEngineState::START_MOVEMENT:
                // in this state we do a single "step" where we only move the trunk
                if (halfStepFinished) {
                    if (ordersZero) {
                        engine_state = WalkEngineState::STOP_MOVEMENT;
                        buildStopMovementTrajectories(orders);
                    }
                    else {
                        // start step is finished, go to next state
                        buildNormalTrajectories(orders);
                        engine_state = WalkEngineState::START_STEP;
                    }
                }
                break;
            case WalkEngineState::START_STEP:
                if (halfStepFinished) {
                    if (ordersZero) {
                        // we have zero command vel -> we should stop
                        engine_state = WalkEngineState::STOP_STEP;
                        // phase = 0.0f;
                        buildStopStepTrajectories(orders);
                    }
                    else {
                        // start step is finished, go to next state
                        buildNormalTrajectories(orders);
                        engine_state = WalkEngineState::WALKING;
                    }
                }
                break;
            case WalkEngineState::WALKING:
                // check if a half step was finished and we are unstable
                if (halfStepFinished && pause_requested) {
                    // go into pause
                    engine_state    = WalkEngineState::PAUSED;
                    pause_requested = false;
                    return false;
                }
                else if (halfStepFinished
                         && ((left_kick_requested && !foot_step.isLeftSupport())
                             || (right_kick_requested && foot_step.isLeftSupport()))) {
                    // lets do a kick
                    buildKickTrajectories(orders);
                    engine_state         = WalkEngineState::KICK;
                    left_kick_requested  = false;
                    right_kick_requested = false;
                }
                else if (halfStepFinished) {
                    // current step is finished, lets see if we have to change state
                    if (ordersZero) {
                        // we have zero command vel -> we should stop
                        engine_state = WalkEngineState::STOP_STEP;
                        // phase = 0.0f;
                        buildStopStepTrajectories(orders);
                    }
                    else {
                        // we can keep on walking
                        buildNormalTrajectories(orders);
                    }
                }
                break;
            case WalkEngineState::KICK:
                // in this state we do a kick while doing a step
                if (halfStepFinished) {
                    // kick step is finished, go on walking
                    engine_state = WalkEngineState::WALKING;
                    buildNormalTrajectories(orders);
                }
                break;
            case WalkEngineState::STOP_STEP:
                // in this state we do a step back to get feet into idle pose
                if (halfStepFinished) {
                    // stop step is finished, go to stop movement state
                    engine_state = WalkEngineState::STOP_MOVEMENT;
                    buildStopMovementTrajectories(orders);
                }
                break;
            case WalkEngineState::STOP_MOVEMENT:
                // in this state we do a "step" where we move the trunk back to idle position
                if (halfStepFinished) {
                    // stop movement is finished, go to idle state
                    engine_state = WalkEngineState::IDLE;
                    return false;
                }
                break;
            default:
                NUClear::log<NUClear::WARN>("Something is wrong with the walk engine state.");
                engine_state = WalkEngineState::IDLE;
        }

        // Sanity check support foot state
        if ((phase < 0.5f && !foot_step.isLeftSupport()) || (phase >= 0.5f && foot_step.isLeftSupport())) {
            NUClear::log<NUClear::WARN>(
                fmt::format("Invalid state. phase={}, support={}, dt={}", phase, foot_step.isLeftSupport(), dt));
            return false;
        }
        last_phase = phase;

        return true;
    }

    void QuinticWalkEngine::updatePhase(const float& dt) {
        float local_dt = dt;
        // Check for negative time step
        if (local_dt <= 0.0f) {
            if (local_dt == 0.0f) {  // sometimes happens due to rounding
                local_dt = 0.0001f;
            }
            else {
                NUClear::log<NUClear::WARN>(fmt::format("Negative dt. phase={}, dt={}", phase, dt));
                return;
            }
        }
        // Check for too long dt
        if (local_dt > 0.25f / params.freq) {
            NUClear::log<NUClear::WARN>(fmt::format("dt too long. phase={}, dt={}", phase, dt));
            return;
        }

        // Update the phase
        phase += local_dt * params.freq;

        // reset to 0 if step complete
        if (phase > 1.0f) {
            phase = 0.0f;
        }
    }

    void QuinticWalkEngine::reset() {
        engine_state = WalkEngineState::IDLE;
        phase        = 0.0f;
        time_paused  = 0.0f;

        // Initialize the footstep
        foot_step.setFootDistance(params.foot_distance);
        foot_step.reset(false);
        // Reset the trunk saved state
        resetTrunkLastState();
    }

    void QuinticWalkEngine::saveCurrentTrunkState() {
        // Evaluate current trunk state (position, velocity, acceleration) in next support foot frame

        // Compute current point in time to save state by multiplying the half period time with the advancement of
        // period time
        float factor = last_phase;
        if (factor < 0.5f) {
            factor = factor * 2.0f;
        }

        float period_time = half_period * factor;

        Eigen::Vector2f trunkPos(trajs.get(TrajectoryTypes::TRUNK_POS_X).pos(period_time),
                                 trajs.get(TrajectoryTypes::TRUNK_POS_Y).pos(period_time));
        Eigen::Vector2f trunkVel(trajs.get(TrajectoryTypes::TRUNK_POS_X).vel(period_time),
                                 trajs.get(TrajectoryTypes::TRUNK_POS_Y).vel(period_time));
        Eigen::Vector2f trunkAcc(trajs.get(TrajectoryTypes::TRUNK_POS_X).acc(period_time),
                                 trajs.get(TrajectoryTypes::TRUNK_POS_Y).acc(period_time));

        // Convert in next support foot frame
        trunkPos.x() -= foot_step.getNext().x();
        trunkPos.y() -= foot_step.getNext().y();
        trunkPos = Eigen::Rotation2Df(-foot_step.getNext().z()).toRotationMatrix() * trunkPos;
        trunkVel = Eigen::Rotation2Df(-foot_step.getNext().z()).toRotationMatrix() * trunkVel;
        trunkAcc = Eigen::Rotation2Df(-foot_step.getNext().z()).toRotationMatrix() * trunkAcc;

        // Save state
        trunk_pos_at_last.x() = trunkPos.x();
        trunk_pos_at_last.y() = trunkPos.y();
        trunk_vel_at_last.x() = trunkVel.x();
        trunk_vel_at_last.y() = trunkVel.y();
        trunk_acc_at_last.x() = trunkAcc.x();
        trunk_acc_at_last.y() = trunkAcc.y();

        // No transformation for height
        trunk_pos_at_last.z() = trajs.get(TrajectoryTypes::TRUNK_POS_Z).pos(period_time);
        trunk_vel_at_last.z() = trajs.get(TrajectoryTypes::TRUNK_POS_Z).vel(period_time);
        trunk_acc_at_last.z() = trajs.get(TrajectoryTypes::TRUNK_POS_Z).acc(period_time);

        // Evaluate and save trunk orientation in next support foot frame
        Eigen::Vector3f trunkAxis(trajs.get(TrajectoryTypes::TRUNK_AXIS_X).pos(period_time),
                                  trajs.get(TrajectoryTypes::TRUNK_AXIS_Y).pos(period_time),
                                  trajs.get(TrajectoryTypes::TRUNK_AXIS_Z).pos(period_time));

        // Transform to next support foot
        trunkAxis.z() -= foot_step.getNext().z();

        trunk_axis_pos_at_last = trunkAxis;

        // Evaluate trunk orientation velocity and acceleration without frame transformation
        trunk_axis_vel_at_last.x() = trajs.get(TrajectoryTypes::TRUNK_AXIS_X).vel(period_time);
        trunk_axis_vel_at_last.y() = trajs.get(TrajectoryTypes::TRUNK_AXIS_Y).vel(period_time);
        trunk_axis_vel_at_last.z() = trajs.get(TrajectoryTypes::TRUNK_AXIS_Z).vel(period_time);
        trunk_axis_acc_at_last.x() = trajs.get(TrajectoryTypes::TRUNK_AXIS_X).acc(period_time);
        trunk_axis_acc_at_last.y() = trajs.get(TrajectoryTypes::TRUNK_AXIS_Y).acc(period_time);
        trunk_axis_acc_at_last.z() = trajs.get(TrajectoryTypes::TRUNK_AXIS_Z).acc(period_time);
    }

    void QuinticWalkEngine::buildTrajectories(const Eigen::Vector3f& orders,
                                              const bool& startMovement,
                                              const bool& startStep,
                                              const bool& kickStep) {


        // Key time points for trajectories are : [0 -> double_support_length -> single_support_length/2 -> half_period]

        // Time length of double and single support phase during the half cycle
        float double_support_length = params.double_support_ratio * half_period;
        float single_support_length = half_period - double_support_length;

        // Save the current trunk state to use it later and compute next step position
        if (startMovement) {
            trunk_pos_at_last.y() -= foot_step.getNext().y();
            foot_step.stepFromOrders(Eigen::Vector3f::Zero());
            // Only move the trunk on the first half cycle after a walk enable
            double_support_length = half_period;
            single_support_length = 0.0f;
        }
        else {
            // Save the previous trunk state
            saveCurrentTrunkState();
            foot_step.stepFromOrders(orders);
        }

        // Reset the trajectories
        trajectoriesInit(trajs);

        // Set double support phase
        point(TrajectoryTypes::IS_DOUBLE_SUPPORT, 0.0f, 1.0f);
        point(TrajectoryTypes::IS_DOUBLE_SUPPORT, double_support_length, 1.0f);
        point(TrajectoryTypes::IS_DOUBLE_SUPPORT, double_support_length, 0.0f);
        point(TrajectoryTypes::IS_DOUBLE_SUPPORT, half_period, 0.0f);

        // Set support foot
        point(TrajectoryTypes::IS_LEFT_SUPPORT_FOOT, 0.0f, static_cast<float>(foot_step.isLeftSupport()));
        point(TrajectoryTypes::IS_LEFT_SUPPORT_FOOT, half_period, static_cast<float>(foot_step.isLeftSupport()));

        //  ******************************** Flying foot position******************************** //

        // Add points for the foot x position
        point(TrajectoryTypes::FOOT_POS_X, 0.0f, foot_step.getLast().x());
        point(TrajectoryTypes::FOOT_POS_X, double_support_length, foot_step.getLast().x());
        point(TrajectoryTypes::FOOT_POS_X, half_period, foot_step.getNext().x());

        // Add points for the foot y position
        point(TrajectoryTypes::FOOT_POS_Y, 0.0f, foot_step.getLast().y());
        point(TrajectoryTypes::FOOT_POS_Y, double_support_length, foot_step.getLast().y());
        point(TrajectoryTypes::FOOT_POS_Y, half_period, foot_step.getNext().y());

        // Add points for the foot z position
        point(TrajectoryTypes::FOOT_POS_Z, 0.0f, 0.0f);
        point(TrajectoryTypes::FOOT_POS_Z, double_support_length, 0.0f);
        point(TrajectoryTypes::FOOT_POS_Z, double_support_length + 0.5f * single_support_length, params.foot_rise);
        point(TrajectoryTypes::FOOT_POS_Z, half_period, 0.0f);

        //  ******************************** Flying foot orientation ******************************** //
        // Flying foot roll
        point(TrajectoryTypes::FOOT_AXIS_X, 0.0f, 0.0f);
        point(TrajectoryTypes::FOOT_AXIS_X, double_support_length, 0.0f);
        point(TrajectoryTypes::FOOT_AXIS_X, half_period, 0.0f);

        // Flying foot pitch
        point(TrajectoryTypes::FOOT_AXIS_Y, 0.0f, 0.0f);
        point(TrajectoryTypes::FOOT_AXIS_Y, double_support_length, 0.0f);
        point(TrajectoryTypes::FOOT_AXIS_Y, half_period, 0.0f);

        // Flying foot yaw
        point(TrajectoryTypes::FOOT_AXIS_Z, 0.0f, foot_step.getLast().z());
        point(TrajectoryTypes::FOOT_AXIS_Z, double_support_length, foot_step.getLast().z());
        point(TrajectoryTypes::FOOT_AXIS_Z, half_period, foot_step.getNext().z());

        //  ******************************** Trunk position******************************** //

        // Trunk position at next support (half way between the support and swing foot) and apply lateral y swing
        // amplitude ratio
        Eigen::Vector2f trunk_point_next(0.5f * foot_step.getNext().x() + params.trunk_x_offset,
                                         0.5f * foot_step.getNext().y()
                                             + 0.5f * foot_step.getNext().y() * params.trunk_swing
                                             + params.trunk_y_offset);

        // Trunk forward velocity
        const float trunk_vel_next = foot_step.getNext().x() / half_period;

        // Trunk x position
        point(TrajectoryTypes::TRUNK_POS_X, 0.0f, trunk_pos_at_last.x(), trunk_vel_at_last.x(), trunk_acc_at_last.x());
        point(TrajectoryTypes::TRUNK_POS_X, half_period, trunk_point_next.x(), trunk_vel_next);

        // Trunk y position
        point(TrajectoryTypes::TRUNK_POS_Y, 0.0f, trunk_pos_at_last.y(), trunk_vel_at_last.y(), trunk_acc_at_last.y());
        point(TrajectoryTypes::TRUNK_POS_Y, half_period, trunk_point_next.y());

        // Trunk z position
        point(TrajectoryTypes::TRUNK_POS_Z, 0.0f, trunk_pos_at_last.z(), trunk_vel_at_last.z(), trunk_acc_at_last.z());
        point(TrajectoryTypes::TRUNK_POS_Z, half_period, params.trunk_height);

        //  ******************************** Trunk orientation ******************************** //

        // Trunk roll
        point(TrajectoryTypes::TRUNK_AXIS_X,
              0.0f,
              trunk_axis_pos_at_last.x(),
              trunk_axis_vel_at_last.x(),
              trunk_axis_acc_at_last.x());
        point(TrajectoryTypes::TRUNK_AXIS_X, half_period, 0.0f, 0.0f);

        // Trunk pitch
        point(TrajectoryTypes::TRUNK_AXIS_Y,
              0.0f,
              trunk_axis_pos_at_last.y(),
              trunk_axis_vel_at_last.y(),
              trunk_axis_acc_at_last.y());
        point(TrajectoryTypes::TRUNK_AXIS_Y, half_period, params.trunk_pitch, 0.0f);

        // Trunk yaw
        float trunk_yaw_vel =
            utility::math::angle::angleDistance(foot_step.getLast().z(), foot_step.getNext().z()) / period;

        point(TrajectoryTypes::TRUNK_AXIS_Z,
              0.0f,
              trunk_axis_pos_at_last.z(),
              trunk_axis_vel_at_last.z(),
              trunk_axis_acc_at_last.z());
        point(TrajectoryTypes::TRUNK_AXIS_Z,
              half_period,
              0.5f * foot_step.getLast().z() + 0.5f * foot_step.getNext().z(),
              trunk_yaw_vel);
    }

    void QuinticWalkEngine::buildWalkDisableTrajectories(const Eigen::Vector3f& orders,
                                                         const bool& footInIdlePosition) {
        // Save the current trunk state to use it later
        saveCurrentTrunkState();
        // Update support foot and compute odometry
        foot_step.stepFromOrders(orders);

        // Reset the trajectories
        trajectoriesInit(trajs);

        // Time length of float and single support phase during the half cycle
        const float double_support_length = params.double_support_ratio * half_period;
        const float single_support_length = half_period - double_support_length;

        // Sign of support foot with respect to lateral
        const float supportSign = foot_step.isLeftSupport() ? 1.0f : -1.0f;

        // Set float support phase
        const float isDoubleSupport = footInIdlePosition ? 1.0f : 0.0f;
        point(TrajectoryTypes::IS_DOUBLE_SUPPORT, 0.0f, isDoubleSupport);
        point(TrajectoryTypes::IS_DOUBLE_SUPPORT, half_period, isDoubleSupport);

        // Set support foot
        point(TrajectoryTypes::IS_LEFT_SUPPORT_FOOT, 0.0f, static_cast<float>(foot_step.isLeftSupport()));
        point(TrajectoryTypes::IS_LEFT_SUPPORT_FOOT, half_period, static_cast<float>(foot_step.isLeftSupport()));

        // Add points for flying foot position
        // Foot x position
        point(TrajectoryTypes::FOOT_POS_X, 0.0f, foot_step.getLast().x());
        point(TrajectoryTypes::FOOT_POS_X, double_support_length, foot_step.getLast().x());
        point(TrajectoryTypes::FOOT_POS_X,
              double_support_length + single_support_length * params.foot_put_down_phase * params.foot_overshoot_phase,
              -foot_step.getLast().x() * params.foot_overshoot_ratio);
        point(TrajectoryTypes::FOOT_POS_X,
              double_support_length + single_support_length * params.foot_put_down_phase,
              0.0f);
        point(TrajectoryTypes::FOOT_POS_X, half_period, 0.0f);

        // Foot y position
        point(TrajectoryTypes::FOOT_POS_Y, 0.0f, foot_step.getLast().y());
        point(TrajectoryTypes::FOOT_POS_Y, double_support_length, foot_step.getLast().y());
        point(TrajectoryTypes::FOOT_POS_Y,
              double_support_length + single_support_length * params.foot_put_down_phase * params.foot_overshoot_phase,
              -supportSign * params.foot_distance
                  + (-supportSign * params.foot_distance - foot_step.getLast().y()) * params.foot_overshoot_ratio);
        point(TrajectoryTypes::FOOT_POS_Y,
              double_support_length + single_support_length * params.foot_put_down_phase,
              -supportSign * params.foot_distance);
        point(TrajectoryTypes::FOOT_POS_Y, half_period, -supportSign * params.foot_distance);

        // Foot z position
        // If the walk has just been disabled, make one single step to neutral pose
        if (!footInIdlePosition) {
            point(TrajectoryTypes::FOOT_POS_Z, 0.0f, 0.0f);
            point(TrajectoryTypes::FOOT_POS_Z, double_support_length, 0.0f);
            point(TrajectoryTypes::FOOT_POS_Z,
                  double_support_length + single_support_length * params.foot_apex_phase
                      - 0.5f * params.foot_z_pause * single_support_length,
                  params.foot_rise);
            point(TrajectoryTypes::FOOT_POS_Z,
                  double_support_length + single_support_length * params.foot_apex_phase
                      + 0.5f * params.foot_z_pause * single_support_length,
                  params.foot_rise);
            point(TrajectoryTypes::FOOT_POS_Z,
                  double_support_length + single_support_length * params.foot_put_down_phase,
                  params.foot_put_down_z_offset);
            point(TrajectoryTypes::FOOT_POS_Z, half_period, 0.0f);
        }
        else {
            // dont move the foot in last single step before stop since we only move the trunk back to
            // the center
            point(TrajectoryTypes::FOOT_POS_Z, 0.0f, 0.0f);
            point(TrajectoryTypes::FOOT_POS_Z, half_period, 0.0f);
        }

        // Add points for flying foot orientation
        // Flying foot x axis orientation
        point(TrajectoryTypes::FOOT_AXIS_X, 0.0f, 0.0f);
        point(TrajectoryTypes::FOOT_AXIS_X, half_period, 0.0f);

        // Flying foot y axis orientation
        point(TrajectoryTypes::FOOT_AXIS_Y, 0.0f, 0.0f);
        point(TrajectoryTypes::FOOT_AXIS_Y, half_period, 0.0f);

        // Flying foot z axis orientation
        point(TrajectoryTypes::FOOT_AXIS_Z, 0.0f, foot_step.getLast().z());
        point(TrajectoryTypes::FOOT_AXIS_Z, double_support_length, foot_step.getLast().z());
        point(TrajectoryTypes::FOOT_AXIS_Z,
              double_support_length + single_support_length * params.foot_put_down_phase,
              0.0f);
        point(TrajectoryTypes::FOOT_AXIS_Z, half_period, 0.0f);

        // Add points for trunk position
        // Trunk x position
        point(TrajectoryTypes::TRUNK_POS_X, 0.0f, trunk_pos_at_last.x(), trunk_vel_at_last.x(), trunk_acc_at_last.x());
        point(TrajectoryTypes::TRUNK_POS_X, half_period, params.trunk_x_offset);

        // Trunk y position
        point(TrajectoryTypes::TRUNK_POS_Y, 0.0f, trunk_pos_at_last.y(), trunk_vel_at_last.y(), trunk_acc_at_last.y());
        point(TrajectoryTypes::TRUNK_POS_Y,
              half_period,
              -supportSign * 0.5f * params.foot_distance + params.trunk_y_offset);

        // Trunk z position
        point(TrajectoryTypes::TRUNK_POS_Z, 0.0f, trunk_pos_at_last.z(), trunk_vel_at_last.z(), trunk_acc_at_last.z());
        point(TrajectoryTypes::TRUNK_POS_Z, half_period, params.trunk_height);

        // Trunk x axis orientation
        point(TrajectoryTypes::TRUNK_AXIS_X,
              0.0f,
              trunk_axis_pos_at_last.x(),
              trunk_axis_vel_at_last.x(),
              trunk_axis_acc_at_last.x());
        point(TrajectoryTypes::TRUNK_AXIS_X, half_period, 0.0f);

        // Trunk y axis orientation
        point(TrajectoryTypes::TRUNK_AXIS_Y,
              0.0f,
              trunk_axis_pos_at_last.y(),
              trunk_axis_vel_at_last.y(),
              trunk_axis_acc_at_last.y());
        point(TrajectoryTypes::TRUNK_AXIS_Y, half_period, params.trunk_pitch);

        // Trunk z axis orientation
        point(TrajectoryTypes::TRUNK_AXIS_Z,
              0.0f,
              trunk_axis_pos_at_last.z(),
              trunk_axis_vel_at_last.z(),
              trunk_axis_acc_at_last.z());
        point(TrajectoryTypes::TRUNK_AXIS_Z, half_period, 0.0f);
    }

    void QuinticWalkEngine::resetTrunkLastState() {
        if (foot_step.isLeftSupport()) {
            trunk_pos_at_last << params.trunk_x_offset, -params.foot_distance * 0.5f + params.trunk_y_offset,
                params.trunk_height;
        }
        else {
            trunk_pos_at_last << params.trunk_x_offset, params.foot_distance * 0.5f + params.trunk_y_offset,
                params.trunk_height;
        }
        trunk_vel_at_last.setZero();
        trunk_acc_at_last.setZero();
        trunk_axis_pos_at_last << 0.0f, params.trunk_pitch, 0.0f;
        trunk_axis_vel_at_last.setZero();
        trunk_axis_acc_at_last.setZero();
    }

    QuinticWalkEngine::PositionSupportTuple QuinticWalkEngine::computeCartesianPosition() const {
        // Compute trajectories time
        const float time = getTrajsTime();
        return computeCartesianPositionAtTime(time);
    }

    QuinticWalkEngine::PositionSupportTuple QuinticWalkEngine::computeCartesianPositionAtTime(const float& time) const {
        // Evaluate target cartesian state from trajectories
        const auto [trunkPos, trunkAxis, footPos, footAxis] = trajectoriesTrunkFootPos(time, trajs);
        // Discard isDoubleSupport because we don't use it
        const auto [_, isLeftSupportFoot] = trajectoriesSupportFootState(time, trajs);
        return {trunkPos, trunkAxis, footPos, footAxis, isLeftSupportFoot};
    }


}  // namespace module::motion
