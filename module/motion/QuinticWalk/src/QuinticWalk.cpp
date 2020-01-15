#include "QuinticWalk.h"

#include <fmt/format.h>

#include "extension/Configuration.h"
#include "message/behaviour/FixedWalkCommand.h"
#include "message/motion/GetupCommand.h"
#include "message/motion/KinematicsModel.h"
#include "message/motion/ServoTarget.h"
#include "message/motion/WalkCommand.h"
#include "message/support/SaveConfiguration.h"
#include "utility/math/comparison.h"
#include "utility/math/euler.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace motion {

    using extension::Configuration;

    using message::behaviour::ServoCommand;
    using message::input::Sensors;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::ExecuteGetup;
    using message::motion::KillGetup;
    using message::motion::KinematicsModel;
    using message::motion::ServoTarget;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;

    using utility::input::ServoID;
    using utility::math::matrix::Transform3D;
    using utility::motion::kinematics::calculateLegJoints;

    QuinticWalk::QuinticWalk(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("QuinticWalk.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file QuinticWalk.yaml
            params.freq                          = cfg["walk"]["freq"].as<float>();
            params.double_support_ratio          = cfg["walk"]["double_support_ratio"].as<float>();
            params.first_step_swing_factor       = cfg["walk"]["first_step_swing_factor"].as<float>();
            params.foot_distance                 = cfg["walk"]["foot"]["distance"].as<float>();
            params.foot_rise                     = cfg["walk"]["foot"]["rise"].as<float>();
            params.foot_z_pause                  = cfg["walk"]["foot"]["z_pause"].as<float>();
            params.foot_put_down_z_offset        = cfg["walk"]["foot"]["put_down"]["z_offset"].as<float>();
            params.foot_put_down_phase           = cfg["walk"]["foot"]["put_down"]["phase"].as<float>();
            params.foot_put_down_roll_offset     = cfg["walk"]["foot"]["put_down"]["roll_offset"].as<float>();
            params.foot_apex_phase               = cfg["walk"]["foot"]["apex_phase"].as<float>();
            params.foot_overshoot_ratio          = cfg["walk"]["foot"]["overshoot"]["ratio"].as<float>();
            params.foot_overshoot_phase          = cfg["walk"]["foot"]["overshoot"]["phase"].as<float>();
            params.trunk_height                  = cfg["walk"]["trunk"]["height"].as<float>();
            params.trunk_pitch                   = cfg["walk"]["trunk"]["pitch"].as<float>();
            params.trunk_phase                   = cfg["walk"]["trunk"]["phase"].as<float>();
            params.trunk_x_offset                = cfg["walk"]["trunk"]["x_offset"].as<float>();
            params.trunk_y_offset                = cfg["walk"]["trunk"]["y_offset"].as<float>();
            params.trunk_swing                   = cfg["walk"]["trunk"]["swing"].as<float>();
            params.trunk_pause                   = cfg["walk"]["trunk"]["pause"].as<float>();
            params.trunk_x_offset_p_coef_forward = cfg["walk"]["trunk"]["x_offset_p_coef"]["forward"].as<float>();
            params.trunk_x_offset_p_coef_turn    = cfg["walk"]["trunk"]["x_offset_p_coef"]["turn"].as<float>();
            params.trunk_pitch_p_coef_forward    = cfg["walk"]["trunk"]["pitch_p_coef"]["forward"].as<float>();
            params.trunk_pitch_p_coef_turn       = cfg["walk"]["trunk"]["pitch_p_coef"]["turn"].as<float>();
            params.kick_length                   = cfg["walk"]["kick"]["length"].as<float>();
            params.kick_phase                    = cfg["walk"]["kick"]["phase"].as<float>();
            params.kick_vel                      = cfg["walk"]["kick"]["vel"].as<float>();
            params.pause_duration                = cfg["walk"]["pause"]["duration"].as<float>();

            walk_engine.setParameters(params);

            config.max_step[0] = cfg["max_step"]["x"].as<float>();
            config.max_step[1] = cfg["max_step"]["y"].as<float>();
            config.max_step[2] = cfg["max_step"]["z"].as<float>();
            config.max_step_xy = cfg["max_step"]["xy"].as<float>();

            config.imu_active          = cfg["imu"]["active"].as<bool>();
            config.imu_pitch_threshold = cfg["imu"]["pitch"]["threshold"].as<float>();
            config.imu_roll_threshold  = cfg["imu"]["roll"]["threshold"].as<float>();

            for (int i = 0; i < ServoID::NUMBER_OF_SERVOS; ++i) {
                if ((i >= 6) && (i < 18)) {
                    jointGains[i] = cfg["gains"]["legs"].as<float>();
                }
            }
        });

        on<Startup, Trigger<KinematicsModel>>().then("Update Kinematics Model", [this](const KinematicsModel& model) {
            kinematicsModel = model;
            first_run       = true;
            current_orders.setZero();
            is_left_support  = true;
            falling          = false;
            last_update_time = NUClear::clock::now();
            walk_engine.reset();
        });

        on<Trigger<ExecuteGetup>>().then([this]() { falling = true; });

        on<Trigger<KillGetup>>().then([this]() { falling = false; });

        on<Trigger<StopCommand>>().then([this] { current_orders.setZero(); });

        on<Trigger<WalkCommand>>().then([this](const WalkCommand& walkCommand) {
            // the engine expects orders in [m] not [m/s]. We have to compute by dividing by step frequency which is a
            // double step factor 2 since the order distance is only for a single step, not double step
            const float factor             = (1.0 / (params.freq)) / 2.0;
            const Eigen::Vector3f& command = walkCommand.command.cast<float>() * factor;

            // Clamp velocity command
            Eigen::Vector3f orders =
                command.array().max(-config.max_step.array()).min(config.max_step.array()).matrix();

            // translational orders (x+y) should not exceed combined limit. scale if necessary
            if (config.max_step_xy != 0) {
                float scaling_factor = 1.0f / std::max(1.0f, (orders.x() + orders.y()) / config.max_step_xy);
                orders.cwiseProduct(Eigen::Vector3f(scaling_factor, scaling_factor, 1.0f));
            }

            // warn user that speed was limited
            if (command.x() * factor != orders.x() || command.y() * factor != orders.y()
                || command.z() * factor != orders.z()) {
                log<NUClear::WARN>(
                    fmt::format("Speed command was x: {} y: {} z: {} xy: {} but maximum is x: {} y: {} z: {} xy: {}",
                                command.x(),
                                command.y(),
                                command.z(),
                                command.x() + command.y(),
                                config.max_step[0] / factor,
                                config.max_step[1] / factor,
                                config.max_step[2] / factor,
                                config.max_step_xy / factor));
            }

            // Update orders
            current_orders = orders;
        });

        on<Trigger<Sensors>>().then([this](const Sensors& sensors) {
            if (config.imu_active) {
                Eigen::Vector3f RPY =
                    utility::math::euler::MatrixToEulerIntrinsic(sensors.Htw.topLeftCorner<3, 3>().cast<float>());

                // compute the pitch offset to the currently wanted pitch of the engine
                float wanted_pitch =
                    params.trunk_pitch + params.trunk_pitch_p_coef_forward * walk_engine.getFootstep().getNext().x()
                    + params.trunk_pitch_p_coef_turn * std::abs(walk_engine.getFootstep().getNext().z());
                RPY.y() += wanted_pitch;

                // threshold pitch and roll
                if (std::abs(RPY[0]) > config.imu_roll_threshold) {
                    log<NUClear::WARN>(fmt::format(
                        "Robot roll exceeds threshold - {} > {}", std::abs(RPY.x()), config.imu_roll_threshold));
                    walk_engine.requestPause();
                }
                else if (std::abs(RPY[1]) > config.imu_pitch_threshold) {
                    log<NUClear::WARN>(fmt::format(
                        "Robot pitch exceeds threshold - {} > {}", std::abs(RPY.y()), config.imu_pitch_threshold));
                    walk_engine.requestPause();
                }
            }
        });

        on<Trigger<EnableWalkEngineCommand>>().then([this](const EnableWalkEngineCommand& command) {
            subsumptionId = command.subsumptionId;
            walk_engine.reset();
            update_handle.enable();
        });

        on<Trigger<DisableWalkEngineCommand>>().then([this] { update_handle.disable(); });

        update_handle = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>>().then([this]() {
            const float dt = getTimeDelta();

            if (falling) {
                // We are falling, reset walk engine
                walk_engine.reset();
            }
            else {
                // we don't want to walk, even if we have orders, if we are not in the right state
                /* Our robots will soon^TM be able to sit down and stand up autonomously, when
                 * sitting down the motors are off but will turn on automatically which is why
                 * MOTOR_OFF is a valid walkable state.
                 */
                // TODO Figure out a better way than having integration knowledge that HCM will play
                // an animation to stand up
                bool walkableState = true;  // TODO: what is our equivalent here?
                // bool walkableState = _robotState ==
                // humanoid_league_msgs::RobotControlState::CONTROLABLE
                //                      || _robotState ==
                //                      humanoid_league_msgs::RobotControlState::WALKING
                //                      || _robotState ==
                //                      humanoid_league_msgs::RobotControlState::MOTOR_OFF;
                // see if the walk engine has new goals for us
                bool newGoals = walk_engine.updateState(dt, current_orders, walkableState);
                if (walk_engine.getState() != engine::WalkEngineState::IDLE) {  // todo
                    calculateJointGoals();
                }
            }
        });
    }  // namespace motion

    float QuinticWalk::getTimeDelta() {
        // compute time delta depended if we are currently in simulation or reality
        float dt;
        auto current_time = NUClear::clock::now();
        // only take real time difference if walking was not stopped before
        // TODO Is this really necessary?
        auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_update_time);
        dt                = time_diff_ms.count() / 1000.0f;
        if (dt == 0.0f) {
            log<NUClear::WARN>(fmt::format("dt was 0 ({})", time_diff_ms.count()));
            dt = 0.001f;
        }
        last_update_time = current_time;
        // time is wrong when we run it for the first time
        if (first_run) {
            first_run = false;
            dt        = 0.0001f;
        }
        return dt;
    }

    void QuinticWalk::calculateJointGoals() {
        /*
        This method computes the next motor goals and publishes them.
        */
        auto setRPY = [&](const float& roll, const float& pitch, const float& yaw) {
            const float halfYaw   = yaw * 0.5f;
            const float halfPitch = pitch * 0.5f;
            const float halfRoll  = roll * 0.5f;
            const float cosYaw    = std::cos(halfYaw);
            const float sinYaw    = std::sin(halfYaw);
            const float cosPitch  = std::cos(halfPitch);
            const float sinPitch  = std::sin(halfPitch);
            const float cosRoll   = std::cos(halfRoll);
            const float sinRoll   = std::sin(halfRoll);
            return Eigen::Quaternionf(cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw,  // formerly yzx
                                      sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,  // x
                                      cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,  // y
                                      cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw   // z
                                      )
                .normalized()
                .toRotationMatrix();
        };

        // read the cartesian positions and orientations for trunk and fly foot
        walk_engine.computeCartesianPosition(trunk_pos, trunk_axis, foot_pos, foot_axis, is_left_support);

        // change goals from support foot based coordinate system to trunk based coordinate system
        Eigen::Affine3f Hst;  // trunk_to_support_foot_goal
        Hst.linear()      = setRPY(trunk_axis[0], trunk_axis[1], trunk_axis[2]).transpose();
        Hst.translation() = -Hst.rotation() * trunk_pos;

        Eigen::Affine3f Hfs;  // support_to_flying_foot
        Hfs.linear()      = setRPY(foot_axis[0], foot_axis[1], foot_axis[2]);
        Hfs.translation() = foot_pos;

        // Eigen::Affine3f Hft = Hst * Hfs;  // trunk_to_flying_foot_goal
        Eigen::Affine3f Hft = Hfs * Hst;  // trunk_to_flying_foot_goal ????? should be this????

        // Calculate leg joints
        std::vector<std::pair<ServoID, float>> joints;
        Eigen::Matrix4d left_foot =
            walk_engine.getFootstep().isLeftSupport() ? Hst.matrix().cast<double>() : Hft.matrix().cast<double>();
        Eigen::Matrix4d right_foot =
            walk_engine.getFootstep().isLeftSupport() ? Hft.matrix().cast<double>() : Hst.matrix().cast<double>();
        joints = calculateLegJoints(kinematicsModel, Transform3D(convert(left_foot)), Transform3D(convert(right_foot)));

        auto waypoints = motionLegs(joints);

        emit(waypoints);
    }

    std::unique_ptr<std::vector<ServoCommand>> QuinticWalk::motionLegs(
        const std::vector<std::pair<ServoID, float>>& joints) {
        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
        waypoints->reserve(16);

        NUClear::clock::time_point time = NUClear::clock::now() + Per<std::chrono::seconds>(UPDATE_FREQUENCY);


        for (auto& joint : joints) {
            waypoints->push_back({subsumptionId,
                                  time,
                                  joint.first,
                                  joint.second,
                                  jointGains[joint.first],
                                  100});  // TODO: support separate gains for each leg
        }

        return waypoints;
    }
}  // namespace motion
}  // namespace module
