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

namespace utility::skill {

    QuinticWalkEngine::QuinticWalkEngine() {
        trajectories_init(trajs);
    }

    bool QuinticWalkEngine::update_state(const float& dt, const Eigen::Vector3f& orders) {
        const bool orders_zero = orders.isZero();
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
            if (orders_zero) {
                // we are in idle and don't have orders. current state is fine, just do nothing
                return false;
            }
        }

        // update the current phase
        update_phase(dt);

        // check if we will finish a half step with this update
        const bool half_step_finished = (last_phase < 0.5f && phase >= 0.5f) || (last_phase > 0.5f && phase < 0.5f);

        // small state machine
        switch (engine_state) {
            case WalkEngineState::IDLE:
                // state is idle and orders are not zero, we can start walking
                build_start_movement_trajectories(orders);
                engine_state = WalkEngineState::START_MOVEMENT;
                break;
            case WalkEngineState::START_MOVEMENT:
                // in this state we do a single "step" where we only move the trunk
                if (half_step_finished) {
                    if (orders_zero) {
                        engine_state = WalkEngineState::STOP_MOVEMENT;
                        build_stop_movement_trajectories(orders);
                    }
                    else {
                        // start step is finished, go to next state
                        build_normal_trajectories(orders);
                        engine_state = WalkEngineState::START_STEP;
                    }
                }
                break;
            case WalkEngineState::START_STEP:
                if (half_step_finished) {
                    if (orders_zero) {
                        // we have zero command vel -> we should stop
                        engine_state = WalkEngineState::STOP_STEP;
                        // phase = 0.0f;
                        build_stop_step_trajectories(orders);
                    }
                    else {
                        // start step is finished, go to next state
                        build_normal_trajectories(orders);
                        engine_state = WalkEngineState::WALKING;
                    }
                }
                break;
            case WalkEngineState::WALKING:
                // check if a half step was finished and we are unstable
                if (half_step_finished && pause_requested) {
                    // go into pause
                    engine_state    = WalkEngineState::PAUSED;
                    pause_requested = false;
                    return false;
                }
                else if (half_step_finished
                         && ((left_kick_requested && !foot_step.is_left_support())
                             || (right_kick_requested && foot_step.is_left_support()))) {
                    // lets do a kick
                    build_kick_trajectories(orders);
                    engine_state         = WalkEngineState::KICK;
                    left_kick_requested  = false;
                    right_kick_requested = false;
                }
                else if (half_step_finished) {
                    // current step is finished, lets see if we have to change state
                    if (orders_zero) {
                        // we have zero command vel -> we should stop
                        engine_state = WalkEngineState::STOP_STEP;
                        // phase = 0.0f;
                        build_stop_step_trajectories(orders);
                    }
                    else {
                        // we can keep on walking
                        build_normal_trajectories(orders);
                    }
                }
                break;
            case WalkEngineState::KICK:
                // in this state we do a kick while doing a step
                if (half_step_finished) {
                    // kick step is finished, go on walking
                    engine_state = WalkEngineState::WALKING;
                    build_normal_trajectories(orders);
                }
                break;
            case WalkEngineState::STOP_STEP:
                // in this state we do a step back to get feet into idle pose
                if (half_step_finished) {
                    // stop step is finished, go to stop movement state
                    engine_state = WalkEngineState::STOP_MOVEMENT;
                    build_stop_movement_trajectories(orders);
                }
                break;
            case WalkEngineState::STOP_MOVEMENT:
                // in this state we do a "step" where we move the trunk back to idle position
                if (half_step_finished) {
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
        if ((phase < 0.5f && !foot_step.is_left_support()) || (phase >= 0.5f && foot_step.is_left_support())) {
            NUClear::log<NUClear::WARN>(
                fmt::format("Invalid state. phase={}, support={}, dt={}", phase, foot_step.is_left_support(), dt));
            return false;
        }
        last_phase = phase;

        return true;
    }

    void QuinticWalkEngine::update_phase(const float& dt) {
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
        foot_step.set_foot_distance(params.foot_distance);
        foot_step.reset(false);
        // Reset the trunk saved state
        reset_trunk_last_state();
    }

    void QuinticWalkEngine::save_current_trunk_state() {
        // Evaluate current trunk state (position, velocity, acceleration) in next support foot frame

        // Compute current point in time to save state by multiplying the half period time with the advancement of
        // period time
        float factor = last_phase;
        if (factor < 0.5f) {
            factor = factor * 2.0f;
        }

        float period_time = half_period * factor;

        Eigen::Vector2f trunk_pos(trajs.get(TrajectoryTypes::TRUNK_POS_X).pos(period_time),
                                  trajs.get(TrajectoryTypes::TRUNK_POS_Y).pos(period_time));
        Eigen::Vector2f trunk_vel(trajs.get(TrajectoryTypes::TRUNK_POS_X).vel(period_time),
                                  trajs.get(TrajectoryTypes::TRUNK_POS_Y).vel(period_time));
        Eigen::Vector2f trunk_acc(trajs.get(TrajectoryTypes::TRUNK_POS_X).acc(period_time),
                                  trajs.get(TrajectoryTypes::TRUNK_POS_Y).acc(period_time));

        // Convert in next support foot frame
        trunk_pos.x() -= foot_step.get_next().x();
        trunk_pos.y() -= foot_step.get_next().y();
        trunk_pos = Eigen::Rotation2Df(-foot_step.get_next().z()).toRotationMatrix() * trunk_pos;
        trunk_vel = Eigen::Rotation2Df(-foot_step.get_next().z()).toRotationMatrix() * trunk_vel;
        trunk_acc = Eigen::Rotation2Df(-foot_step.get_next().z()).toRotationMatrix() * trunk_acc;

        // Save state
        trunk_pos_at_last.x() = trunk_pos.x();
        trunk_pos_at_last.y() = trunk_pos.y();
        trunk_vel_at_last.x() = trunk_vel.x();
        trunk_vel_at_last.y() = trunk_vel.y();
        trunk_acc_at_last.x() = trunk_acc.x();
        trunk_acc_at_last.y() = trunk_acc.y();

        // No transformation for height
        trunk_pos_at_last.z() = trajs.get(TrajectoryTypes::TRUNK_POS_Z).pos(period_time);
        trunk_vel_at_last.z() = trajs.get(TrajectoryTypes::TRUNK_POS_Z).vel(period_time);
        trunk_acc_at_last.z() = trajs.get(TrajectoryTypes::TRUNK_POS_Z).acc(period_time);

        // Evaluate and save trunk orientation in next support foot frame
        Eigen::Vector3f trunk_axis(trajs.get(TrajectoryTypes::TRUNK_AXIS_X).pos(period_time),
                                   trajs.get(TrajectoryTypes::TRUNK_AXIS_Y).pos(period_time),
                                   trajs.get(TrajectoryTypes::TRUNK_AXIS_Z).pos(period_time));

        // Transform to next support foot
        trunk_axis.z() -= foot_step.get_next().z();

        trunk_axis_pos_at_last = trunk_axis;

        // Evaluate trunk orientation velocity and acceleration without frame transformation
        trunk_axis_vel_at_last.x() = trajs.get(TrajectoryTypes::TRUNK_AXIS_X).vel(period_time);
        trunk_axis_vel_at_last.y() = trajs.get(TrajectoryTypes::TRUNK_AXIS_Y).vel(period_time);
        trunk_axis_vel_at_last.z() = trajs.get(TrajectoryTypes::TRUNK_AXIS_Z).vel(period_time);
        trunk_axis_acc_at_last.x() = trajs.get(TrajectoryTypes::TRUNK_AXIS_X).acc(period_time);
        trunk_axis_acc_at_last.y() = trajs.get(TrajectoryTypes::TRUNK_AXIS_Y).acc(period_time);
        trunk_axis_acc_at_last.z() = trajs.get(TrajectoryTypes::TRUNK_AXIS_Z).acc(period_time);
    }

    void QuinticWalkEngine::build_trajectories(const Eigen::Vector3f& orders,
                                               const bool& start_movement,
                                               const bool& start_step,
                                               const bool& kick_step) {


        // Time length of double and single support phase during the half cycle
        float double_support_length = params.double_support_ratio * half_period;
        float single_support_length = half_period - double_support_length;

        // Save the current trunk state to use it later and compute next step position
        if (start_movement) {
            trunk_pos_at_last.y() -= foot_step.get_next().y();
            foot_step.step_from_orders(Eigen::Vector3f::Zero());
            // Only move the trunk on the first half cycle after a walk enable
            double_support_length = half_period;
            single_support_length = 0.0f;
        }
        else {
            // Save the previous trunk state
            save_current_trunk_state();
            foot_step.step_from_orders(orders);
        }

        // Reset the trajectories
        trajectories_init(trajs);

        // Set double support phase
        point(TrajectoryTypes::IS_DOUBLE_SUPPORT, 0.0f, 1.0f);
        point(TrajectoryTypes::IS_DOUBLE_SUPPORT, double_support_length, 1.0f);
        point(TrajectoryTypes::IS_DOUBLE_SUPPORT, double_support_length, 0.0f);
        point(TrajectoryTypes::IS_DOUBLE_SUPPORT, half_period, 0.0f);

        // Set support foot
        point(TrajectoryTypes::IS_LEFT_SUPPORT_FOOT, 0.0f, static_cast<float>(foot_step.is_left_support()));
        point(TrajectoryTypes::IS_LEFT_SUPPORT_FOOT, half_period, static_cast<float>(foot_step.is_left_support()));

        //  ******************************** Flying foot position ******************************** //

        //  Flying foot x position
        point(TrajectoryTypes::FOOT_POS_X, 0.0f, foot_step.get_last().x());
        point(TrajectoryTypes::FOOT_POS_X, double_support_length, foot_step.get_last().x());
        if (kick_step) {
            point(TrajectoryTypes::FOOT_POS_X,
                  double_support_length + single_support_length * params.kick_phase,
                  foot_step.get_next().x() + params.kick_length,
                  params.kick_vel);
        }
        point(TrajectoryTypes::FOOT_POS_X, half_period, foot_step.get_next().x());

        //  Flying foot y position
        point(TrajectoryTypes::FOOT_POS_Y, 0.0f, foot_step.get_last().y());
        point(TrajectoryTypes::FOOT_POS_Y, double_support_length, foot_step.get_last().y());
        point(TrajectoryTypes::FOOT_POS_Y, half_period, foot_step.get_next().y());

        //  Flying foot z position
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
        point(TrajectoryTypes::FOOT_AXIS_Z, 0.0f, foot_step.get_last().z());
        point(TrajectoryTypes::FOOT_AXIS_Z, double_support_length, foot_step.get_last().z());
        point(TrajectoryTypes::FOOT_AXIS_Z, half_period, foot_step.get_next().z());

        //  ******************************** Trunk position ******************************** //

        const float period      = 2.0f * half_period;
        const float pauseLength = 0.5f * params.trunk_pause * half_period;
        const float timeShift   = (double_support_length - half_period) * 0.5f + params.trunk_phase * half_period;

        const Eigen::Vector2f trunk_point_support(
            params.trunk_x_offset + params.trunk_x_offset_p_coef_forward * foot_step.get_next().x()
                + params.trunk_x_offset_p_coef_turn * std::fabs(foot_step.get_next().z()),
            params.trunk_y_offset);

        const Eigen::Vector2f trunk_point_next(
            foot_step.get_next().x() + params.trunk_x_offset
                + params.trunk_x_offset_p_coef_forward * foot_step.get_next().x()
                + params.trunk_x_offset_p_coef_turn * std::fabs(foot_step.get_next().z()),
            foot_step.get_next().y() + params.trunk_y_offset);

        // Trunk middle neutral (no swing) position
        const Eigen::Vector2f trunk_point_middle = 0.5f * (trunk_point_support + trunk_point_next);

        // Trunk vector from middle to support apex
        Eigen::Vector2f trunk_vect = trunk_point_support - trunk_point_middle;

        // Apply swing amplitude ratio
        trunk_vect.y() *= params.trunk_swing;

        // Trunk support and next apex position
        const Eigen::Vector2f trunkApexSupport = trunk_point_middle + trunk_vect;
        const Eigen::Vector2f trunkApexNext    = trunk_point_middle - trunk_vect;

        const float trunk_velSupport = (foot_step.get_next().x() - foot_step.get_last().x()) / period;
        const float trunk_velNext    = foot_step.get_next().x() / half_period;

        // Trunk x position
        if (start_step) {
            point(TrajectoryTypes::TRUNK_POS_X, 0.0f, 0.0f, 0.0f, 0.0f);
        }
        else {
            point(TrajectoryTypes::TRUNK_POS_X,
                  0.0f,
                  trunk_pos_at_last.x(),
                  trunk_vel_at_last.x(),
                  trunk_acc_at_last.x());
            point(TrajectoryTypes::TRUNK_POS_X, half_period + timeShift, trunkApexSupport.x(), trunk_velSupport);
        }
        point(TrajectoryTypes::TRUNK_POS_X, period + timeShift, trunkApexNext.x(), trunk_velNext);

        // Trunk y position
        point(TrajectoryTypes::TRUNK_POS_Y, 0.0f, trunk_pos_at_last.y(), trunk_vel_at_last.y(), trunk_acc_at_last.y());
        if (start_step || start_movement) {
            point(TrajectoryTypes::TRUNK_POS_Y,
                  half_period + timeShift - pauseLength,
                  trunk_point_middle.y() + trunk_vect.y() * params.first_step_swing_factor);
            point(TrajectoryTypes::TRUNK_POS_Y,
                  half_period + timeShift + pauseLength,
                  trunk_point_middle.y() + trunk_vect.y() * params.first_step_swing_factor);
            point(TrajectoryTypes::TRUNK_POS_Y,
                  period + timeShift - pauseLength,
                  trunk_point_middle.y() - trunk_vect.y() * params.first_step_swing_factor);
            point(TrajectoryTypes::TRUNK_POS_Y,
                  period + timeShift + pauseLength,
                  trunk_point_middle.y() - trunk_vect.y() * params.first_step_swing_factor);
        }
        else {
            point(TrajectoryTypes::TRUNK_POS_Y, half_period + timeShift - pauseLength, trunkApexSupport.y());
            point(TrajectoryTypes::TRUNK_POS_Y, half_period + timeShift + pauseLength, trunkApexSupport.y());
            point(TrajectoryTypes::TRUNK_POS_Y, period + timeShift - pauseLength, trunkApexNext.y());
            point(TrajectoryTypes::TRUNK_POS_Y, period + timeShift + pauseLength, trunkApexNext.y());
        }

        // Trunk z position
        point(TrajectoryTypes::TRUNK_POS_Z, 0.0f, trunk_pos_at_last.z(), trunk_vel_at_last.z(), trunk_acc_at_last.z());
        point(TrajectoryTypes::TRUNK_POS_Z, half_period + timeShift, params.trunk_height);
        point(TrajectoryTypes::TRUNK_POS_Z, period + timeShift, params.trunk_height);


        //  ******************************** Trunk orientation ******************************** //

        // Trunk roll
        point(TrajectoryTypes::TRUNK_AXIS_X,
              0.0f,
              trunk_axis_pos_at_last.x(),
              trunk_axis_vel_at_last.x(),
              trunk_axis_acc_at_last.x());
        point(TrajectoryTypes::TRUNK_AXIS_X, half_period + timeShift, 0.0f, 0.0f);
        point(TrajectoryTypes::TRUNK_AXIS_X, period + timeShift, 0.0f, 0.0f);

        // Trunk pitch
        point(TrajectoryTypes::TRUNK_AXIS_Y,
              0.0f,
              trunk_axis_pos_at_last.y(),
              trunk_axis_vel_at_last.y(),
              trunk_axis_acc_at_last.y());
        point(TrajectoryTypes::TRUNK_AXIS_Y, half_period, params.trunk_pitch, 0.0f);
        point(TrajectoryTypes::TRUNK_AXIS_Y, period + timeShift, params.trunk_pitch, 0.0f);

        // Trunk yaw
        float trunk_yaw_vel =
            utility::math::angle::angleDistance(foot_step.get_last().z(), foot_step.get_next().z()) / half_period;

        point(TrajectoryTypes::TRUNK_AXIS_Z,
              0.0f,
              trunk_axis_pos_at_last.z(),
              trunk_axis_vel_at_last.z(),
              trunk_axis_acc_at_last.z());
        point(TrajectoryTypes::TRUNK_AXIS_Z,
              half_period + timeShift,
              0.5f * foot_step.get_last().z() + 0.5f * foot_step.get_next().z(),
              trunk_yaw_vel);
        point(TrajectoryTypes::TRUNK_AXIS_Z, period + timeShift, foot_step.get_next().z(), trunk_yaw_vel);
    }

    void QuinticWalkEngine::build_walk_disable_trajectories(const Eigen::Vector3f& orders,
                                                            const bool& foot_in_idle_position) {
        // Save the current trunk state to use it later
        save_current_trunk_state();
        // Update support foot and compute odometry
        foot_step.step_from_orders(orders);

        // Reset the trajectories
        trajectories_init(trajs);

        // Time length of float and single support phase during the half cycle
        const float double_support_length = params.double_support_ratio * half_period;
        const float single_support_length = half_period - double_support_length;

        // Sign of support foot with respect to lateral
        const float support_sign = foot_step.is_left_support() ? 1.0f : -1.0f;

        // Set float support phase
        const float is_double_support = foot_in_idle_position ? 1.0f : 0.0f;
        point(TrajectoryTypes::IS_DOUBLE_SUPPORT, 0.0f, is_double_support);
        point(TrajectoryTypes::IS_DOUBLE_SUPPORT, half_period, is_double_support);

        // Set support foot
        point(TrajectoryTypes::IS_LEFT_SUPPORT_FOOT, 0.0f, static_cast<float>(foot_step.is_left_support()));
        point(TrajectoryTypes::IS_LEFT_SUPPORT_FOOT, half_period, static_cast<float>(foot_step.is_left_support()));

        //  ******************************** Flying foot position ******************************** //


        // Foot x position
        point(TrajectoryTypes::FOOT_POS_X, 0.0f, foot_step.get_last().x());
        point(TrajectoryTypes::FOOT_POS_X, double_support_length, foot_step.get_last().x());
        point(TrajectoryTypes::FOOT_POS_X,
              double_support_length + single_support_length * params.foot_put_down_phase * params.foot_overshoot_phase,
              -foot_step.get_last().x() * params.foot_overshoot_ratio);
        point(TrajectoryTypes::FOOT_POS_X,
              double_support_length + single_support_length * params.foot_put_down_phase,
              0.0f);
        point(TrajectoryTypes::FOOT_POS_X, half_period, 0.0f);


        // Foot y position
        point(TrajectoryTypes::FOOT_POS_Y, 0.0f, foot_step.get_last().y());
        point(TrajectoryTypes::FOOT_POS_Y, double_support_length, foot_step.get_last().y());
        point(TrajectoryTypes::FOOT_POS_Y,
              double_support_length + single_support_length * params.foot_put_down_phase * params.foot_overshoot_phase,
              -support_sign * params.foot_distance
                  + (-support_sign * params.foot_distance - foot_step.get_last().y()) * params.foot_overshoot_ratio);
        point(TrajectoryTypes::FOOT_POS_Y,
              double_support_length + single_support_length * params.foot_put_down_phase,
              -support_sign * params.foot_distance);
        point(TrajectoryTypes::FOOT_POS_Y, half_period, -support_sign * params.foot_distance);

        // Foot z position
        // If the walk has just been disabled, make one single step to neutral pose
        if (!foot_in_idle_position) {
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

        //  ******************************** Flying foot orientation ******************************** //
        // Flying foot x axis orientation
        point(TrajectoryTypes::FOOT_AXIS_X, 0.0f, 0.0f);
        point(TrajectoryTypes::FOOT_AXIS_X, half_period, 0.0f);

        // Flying foot y axis orientation
        point(TrajectoryTypes::FOOT_AXIS_Y, 0.0f, 0.0f);
        point(TrajectoryTypes::FOOT_AXIS_Y, half_period, 0.0f);

        // Flying foot z axis orientation
        point(TrajectoryTypes::FOOT_AXIS_Z, 0.0f, foot_step.get_last().z());
        point(TrajectoryTypes::FOOT_AXIS_Z, double_support_length, foot_step.get_last().z());
        point(TrajectoryTypes::FOOT_AXIS_Z,
              double_support_length + single_support_length * params.foot_put_down_phase,
              0.0f);
        point(TrajectoryTypes::FOOT_AXIS_Z, half_period, 0.0f);

        //  ******************************** Trunk position******************************** //
        // Trunk x position
        point(TrajectoryTypes::TRUNK_POS_X, 0.0f, trunk_pos_at_last.x(), trunk_vel_at_last.x(), trunk_acc_at_last.x());
        point(TrajectoryTypes::TRUNK_POS_X, half_period, params.trunk_x_offset);

        // Trunk y position
        point(TrajectoryTypes::TRUNK_POS_Y, 0.0f, trunk_pos_at_last.y(), trunk_vel_at_last.y(), trunk_acc_at_last.y());
        point(TrajectoryTypes::TRUNK_POS_Y,
              half_period,
              -support_sign * 0.5f * params.foot_distance + params.trunk_y_offset);

        // Trunk z position
        point(TrajectoryTypes::TRUNK_POS_Z, 0.0f, trunk_pos_at_last.z(), trunk_vel_at_last.z(), trunk_acc_at_last.z());
        point(TrajectoryTypes::TRUNK_POS_Z, half_period, params.trunk_height);

        //  ******************************** Trunk orientation ******************************** //
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

    void QuinticWalkEngine::reset_trunk_last_state() {
        if (foot_step.is_left_support()) {
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

    QuinticWalkEngine::PositionSupportTuple QuinticWalkEngine::compute_cartesian_position() const {
        // Compute trajectories time
        const float time = get_trajs_time();
        return compute_cartesian_position_at_time(time);
    }

    QuinticWalkEngine::PositionSupportTuple QuinticWalkEngine::compute_cartesian_position_at_time(
        const float& time) const {
        // Evaluate target cartesian state from trajectories
        const auto [trunk_pos, trunk_axis, footPos, footAxis] = trajectories_trunk_foot_pos(time, trajs);
        // Discard is_double_support because we don't use it
        const auto [_, is_left_supportFoot] = trajectories_support_foot_state(time, trajs);
        return {trunk_pos, trunk_axis, footPos, footAxis, is_left_supportFoot};
    }


}  // namespace utility::skill
