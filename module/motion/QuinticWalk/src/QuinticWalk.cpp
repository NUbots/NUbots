#include "QuinticWalk.hpp"

#include <fmt/format.h>

#include "extension/Configuration.hpp"

#include "message/behaviour/Behaviour.hpp"
#include "message/motion/GetupCommand.hpp"
#include "message/motion/KinematicsModel.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/support/SaveConfiguration.hpp"

#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/motion/InverseKinematics.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::motion {

    using extension::Configuration;

    using message::behaviour::Behaviour;
    using message::behaviour::ServoCommands;
    using message::input::Sensors;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::ExecuteGetup;
    using message::motion::KillGetup;
    using message::motion::KinematicsModel;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;
    using utility::support::Expression;

    using utility::input::ServoID;
    using utility::motion::kinematics::calculateLegJoints;

    /**
     * @brief loads the configuration from cfg into config
     * @param cfg A Configuration provided by the Configuration extension
     * @param config The destination configuration that we will write to.
     */
    void QuinticWalk::load_quintic_walk(const Configuration& cfg, Config& config) {
        config.params.freq                          = cfg["walk"]["freq"].as<float>();
        config.params.double_support_ratio          = cfg["walk"]["double_support_ratio"].as<float>();
        config.params.first_step_swing_factor       = cfg["walk"]["first_step_swing_factor"].as<float>();
        config.params.foot_distance                 = cfg["walk"]["foot"]["distance"].as<float>();
        config.params.foot_rise                     = cfg["walk"]["foot"]["rise"].as<float>();
        config.params.foot_z_pause                  = cfg["walk"]["foot"]["z_pause"].as<float>();
        config.params.foot_put_down_z_offset        = cfg["walk"]["foot"]["put_down"]["z_offset"].as<float>();
        config.params.foot_put_down_phase           = cfg["walk"]["foot"]["put_down"]["phase"].as<float>();
        config.params.foot_put_down_roll_offset     = cfg["walk"]["foot"]["put_down"]["roll_offset"].as<float>();
        config.params.foot_apex_phase               = cfg["walk"]["foot"]["apex_phase"].as<float>();
        config.params.foot_overshoot_ratio          = cfg["walk"]["foot"]["overshoot"]["ratio"].as<float>();
        config.params.foot_overshoot_phase          = cfg["walk"]["foot"]["overshoot"]["phase"].as<float>();
        config.params.trunk_height                  = cfg["walk"]["trunk"]["height"].as<float>();
        config.params.trunk_pitch                   = 1.0f + cfg["walk"]["trunk"]["pitch"].as<Expression>();
        config.params.trunk_phase                   = cfg["walk"]["trunk"]["phase"].as<float>();
        config.params.trunk_x_offset                = cfg["walk"]["trunk"]["x_offset"].as<float>();
        config.params.trunk_y_offset                = cfg["walk"]["trunk"]["y_offset"].as<float>();
        config.params.trunk_swing                   = cfg["walk"]["trunk"]["swing"].as<float>();
        config.params.trunk_pause                   = cfg["walk"]["trunk"]["pause"].as<float>();
        config.params.trunk_x_offset_p_coef_forward = cfg["walk"]["trunk"]["x_offset_p_coef"]["forward"].as<float>();
        config.params.trunk_x_offset_p_coef_turn    = cfg["walk"]["trunk"]["x_offset_p_coef"]["turn"].as<float>();
        config.params.trunk_pitch_p_coef_forward =
            1.0f + cfg["walk"]["trunk"]["pitch_p_coef"]["forward"].as<Expression>();
        config.params.trunk_pitch_p_coef_turn = 1.0f + cfg["walk"]["trunk"]["pitch_p_coef"]["turn"].as<Expression>();
        config.params.kick_length             = cfg["walk"]["kick"]["length"].as<float>();
        config.params.kick_phase              = cfg["walk"]["kick"]["phase"].as<float>();
        config.params.kick_vel                = cfg["walk"]["kick"]["vel"].as<float>();
        config.params.pause_duration          = cfg["walk"]["pause"]["duration"].as<float>();

        config.max_step.x() = cfg["max_step"]["x"].as<float>();
        config.max_step.y() = cfg["max_step"]["y"].as<float>();
        config.max_step.z() = cfg["max_step"]["z"].as<float>();
        config.max_step_xy  = cfg["max_step"]["xy"].as<float>();

        config.imu_active          = cfg["imu"]["active"].as<bool>();
        config.imu_pitch_threshold = 1.0f + cfg["imu"]["pitch"]["threshold"].as<float>();
        config.imu_roll_threshold  = cfg["imu"]["roll"]["threshold"].as<float>();

        for (int id = 0; id < ServoID::NUMBER_OF_SERVOS; ++id) {
            // Sets the leg gains
            if ((id >= ServoID::R_HIP_YAW) && (id < ServoID::HEAD_YAW)) {
                config.jointGains[id] = cfg["gains"]["legs"].as<float>();
            }
            // Sets the arm gains
            if (id < ServoID::R_HIP_YAW) {
                config.jointGains[id] = cfg["gains"]["arms"].as<float>();
            }
        }

        config.arm_positions.emplace_back(ServoID::R_SHOULDER_PITCH, cfg["arms"]["right_shoulder_pitch"].as<float>());
        config.arm_positions.emplace_back(ServoID::L_SHOULDER_PITCH, cfg["arms"]["left_shoulder_pitch"].as<float>());
        config.arm_positions.emplace_back(ServoID::R_SHOULDER_ROLL, cfg["arms"]["right_shoulder_roll"].as<float>());
        config.arm_positions.emplace_back(ServoID::L_SHOULDER_ROLL, cfg["arms"]["left_shoulder_roll"].as<float>());
        config.arm_positions.emplace_back(ServoID::R_ELBOW, cfg["arms"]["right_elbow"].as<float>());
        config.arm_positions.emplace_back(ServoID::L_ELBOW, cfg["arms"]["left_elbow"].as<float>());
    }

    QuinticWalk::QuinticWalk(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        imu_reaction = on<Trigger<Sensors>>().then([this](const Sensors& sensors) {
            Eigen::Vector3f RPY =
                utility::math::euler::MatrixToEulerIntrinsic(sensors.Htw.topLeftCorner<3, 3>().cast<float>());

            // compute the pitch offset to the currently wanted pitch of the engine
            float wanted_pitch =
                current_config.params.trunk_pitch
                + current_config.params.trunk_pitch_p_coef_forward * walk_engine.getFootstep().getNext().x()
                + current_config.params.trunk_pitch_p_coef_turn * std::abs(walk_engine.getFootstep().getNext().z());
            RPY.y() += wanted_pitch;

            // threshold pitch and roll
            if (std::abs(RPY.x()) > current_config.imu_roll_threshold) {
                log<NUClear::WARN>(fmt::format("Robot roll exceeds threshold - {} > {}",
                                               std::abs(RPY.x()),
                                               current_config.imu_roll_threshold));
                walk_engine.requestPause();
            }
            else if (std::abs(RPY.y()) > current_config.imu_pitch_threshold) {
                log<NUClear::WARN>(fmt::format("Robot pitch exceeds threshold - {} > {}",
                                               std::abs(RPY.y()),
                                               current_config.imu_pitch_threshold));
                walk_engine.requestPause();
            }
        });

        on<Configuration>("QuinticWalk.yaml").then([this](const Configuration& cfg) {
            log_level = cfg["log_level"].as<NUClear::LogLevel>();

            load_quintic_walk(cfg, normal_config);

            // Make sure the walk engine has the parameters at least once
            if (first_config) {
                // Send these parameters to the walk engine
                walk_engine.setParameters(current_config.params);

                imu_reaction.enable(current_config.imu_active);

                first_config = false;
            }
        });

        on<Configuration>("goalie/QuinticWalk.yaml").then([this](const Configuration& cfg) {
            load_quintic_walk(cfg, goalie_config);
        });

        on<Trigger<Behaviour::State>>().then("Switching walk state", [this](const Behaviour::State& behaviour) {
            imu_reaction.enable(false);

            if (behaviour == Behaviour::State::GOALIE_WALK) {
                current_config = goalie_config;
            }
            else {
                current_config = normal_config;
            }

            // Send these parameters to the walk engine
            walk_engine.setParameters(current_config.params);

            imu_reaction.enable(current_config.imu_active);
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

        on<Trigger<StopCommand>>().then([this](const StopCommand& walkCommand) {
            subsumptionId = walkCommand.subsumption_id;
            current_orders.setZero();
        });

        on<Trigger<WalkCommand>>().then([this](const WalkCommand& walkCommand) {
            subsumptionId = walkCommand.subsumption_id;

            // the engine expects orders in [m] not [m/s]. We have to compute by dividing by step frequency which is
            // a double step factor 2 since the order distance is only for a single step, not double step
            const float factor             = (1.0f / (current_config.params.freq)) * 0.5f;
            const Eigen::Vector3f& command = walkCommand.command.cast<float>() * factor;

            // Clamp velocity command
            Eigen::Vector3f orders =
                command.array().max(-current_config.max_step.array()).min(current_config.max_step.array()).matrix();

            // translational orders (x+y) should not exceed combined limit. scale if necessary
            if (current_config.max_step_xy != 0) {
                float scaling_factor = 1.0f / std::max(1.0f, (orders.x() + orders.y()) / current_config.max_step_xy);
                orders.cwiseProduct(Eigen::Vector3f(scaling_factor, scaling_factor, 1.0f));
            }

            // warn user that speed was limited
            if (command.x() != orders.x() || command.y() != orders.y() || command.z() != orders.z()) {
                log<NUClear::WARN>(
                    fmt::format("Speed command was x: {} y: {} z: {} xy: {} but maximum is x: {} y: {} z: {} xy: {}",
                                command.x(),
                                command.y(),
                                command.z(),
                                command.x() + command.y(),
                                current_config.max_step.x() / factor,
                                current_config.max_step.y() / factor,
                                current_config.max_step.z() / factor,
                                current_config.max_step_xy / factor));
            }

            // Update orders
            current_orders = orders;
        });

        on<Trigger<EnableWalkEngineCommand>>().then([this](const EnableWalkEngineCommand& command) {
            subsumptionId = command.subsumption_id;
            walk_engine.reset();
            update_handle.enable();
        });

        on<Trigger<DisableWalkEngineCommand>>().then([this](const DisableWalkEngineCommand& command) {
            subsumptionId = command.subsumption_id;
            update_handle.disable();
        });

        update_handle = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Single>().then([this]() {
            const float dt = getTimeDelta();

            if (falling) {
                // We are falling, reset walk engine
                walk_engine.reset();
            }
            else {

                // see if the walk engine has new goals for us
                if (walk_engine.updateState(dt, current_orders)) {
                    calculateJointGoals();
                }
            }
        });
    }

    float QuinticWalk::getTimeDelta() {
        // compute time delta depended if we are currently in simulation or reality
        const auto current_time = NUClear::clock::now();
        float dt =
            std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_update_time).count() / 1000.0f;

        if (dt == 0.0f) {
            // log<NUClear::WARN>(fmt::format("dt was 0 ({})", time_diff_ms.count()));
            dt = 0.001f;
        }

        // time is wrong when we run it for the first time
        if (first_run) {
            first_run = false;
            dt        = 0.0001f;
        }

        last_update_time = current_time;
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
            return Eigen::Quaternionf(Eigen::Vector4f(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,   // x
                                                      cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,   // y
                                                      cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,   // z
                                                      cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw))  // w
                .normalized()  // Rotation quaternions should have unit norm
                .toRotationMatrix();
        };
        // Read the cartesian positions and orientations for trunk and fly foot
        std::tie(trunk_pos, trunk_axis, foot_pos, foot_axis, is_left_support) = walk_engine.computeCartesianPosition();

        // Change goals from support foot based coordinate system to trunk based coordinate system
        Eigen::Affine3f Hst;  // trunk_to_support_foot_goal
        Hst.linear()      = setRPY(trunk_axis.x(), trunk_axis.y(), trunk_axis.z()).transpose();
        Hst.translation() = -Hst.rotation() * trunk_pos;

        Eigen::Affine3f Hfs;  // support_to_flying_foot
        Hfs.linear()      = setRPY(foot_axis.x(), foot_axis.y(), foot_axis.z());
        Hfs.translation() = foot_pos;

        const Eigen::Affine3f Hft = Hfs * Hst;  // trunk_to_flying_foot_goal

        // Calculate leg joints
        const Eigen::Matrix4f left_foot  = walk_engine.getFootstep().isLeftSupport() ? Hst.matrix() : Hft.matrix();
        const Eigen::Matrix4f right_foot = walk_engine.getFootstep().isLeftSupport() ? Hft.matrix() : Hst.matrix();

        const auto joints =
            calculateLegJoints<float>(kinematicsModel, Eigen::Affine3f(left_foot), Eigen::Affine3f(right_foot));

        auto waypoints = motion(joints);
        emit(std::move(waypoints));
    }

    std::unique_ptr<ServoCommands> QuinticWalk::motion(const std::vector<std::pair<ServoID, float>>& joints) {
        auto waypoints = std::make_unique<ServoCommands>();
        waypoints->commands.reserve(joints.size() + current_config.arm_positions.size());

        const NUClear::clock::time_point time = NUClear::clock::now() + Per<std::chrono::seconds>(UPDATE_FREQUENCY);

        for (const auto& joint : joints) {
            waypoints->commands.emplace_back(subsumptionId,
                                             time,
                                             joint.first,
                                             joint.second,
                                             current_config.jointGains[joint.first],
                                             100);
        }

        for (const auto& joint : current_config.arm_positions) {
            waypoints->commands.emplace_back(subsumptionId,
                                             time,
                                             joint.first,
                                             joint.second,
                                             current_config.jointGains[joint.first],
                                             100);
        }

        return waypoints;
    }
}  // namespace module::motion
