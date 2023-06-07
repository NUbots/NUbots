#include "QuinticWalk.hpp"

#include <fmt/format.h>

#include "extension/Configuration.hpp"

#include "message/actuation/KinematicsModel.hpp"
#include "message/behaviour/Behaviour.hpp"
#include "message/eye/DataPoint.hpp"
#include "message/motion/GetupCommand.hpp"
#include "message/motion/WalkCommand.hpp"

#include "utility/actuation/InverseKinematics.hpp"
#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::motion {

    using extension::Configuration;

    using message::actuation::KinematicsModel;
    using message::behaviour::Behaviour;
    using message::behaviour::ServoCommands;
    using message::input::Sensors;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::ExecuteGetup;
    using message::motion::KillGetup;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;

    using utility::actuation::kinematics::calculateLegJoints;
    using utility::input::ServoID;
    using utility::math::euler::EulerIntrinsicToMatrix;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::nusight::graph;
    using utility::support::Expression;

    /**
     * @brief loads the configuration from cfg into config
     * @param cfg A Configuration provided by the Configuration extension
     * @param config The destination configuration that we will write to.
     */
    void QuinticWalk::load_quintic_walk(const Configuration& cfg, Config& config) {
        config.params.freq                          = cfg["walk"]["freq"].as<double>();
        config.params.double_support_ratio          = cfg["walk"]["double_support_ratio"].as<double>();
        config.params.first_step_swing_factor       = cfg["walk"]["first_step_swing_factor"].as<double>();
        config.params.foot_distance                 = cfg["walk"]["foot"]["distance"].as<double>();
        config.params.foot_rise                     = cfg["walk"]["foot"]["rise"].as<double>();
        config.params.foot_z_pause                  = cfg["walk"]["foot"]["z_pause"].as<double>();
        config.params.foot_put_down_z_offset        = cfg["walk"]["foot"]["put_down"]["z_offset"].as<double>();
        config.params.foot_put_down_phase           = cfg["walk"]["foot"]["put_down"]["phase"].as<double>();
        config.params.foot_put_down_roll_offset     = cfg["walk"]["foot"]["put_down"]["roll_offset"].as<double>();
        config.params.foot_apex_phase               = cfg["walk"]["foot"]["apex_phase"].as<double>();
        config.params.foot_overshoot_ratio          = cfg["walk"]["foot"]["overshoot"]["ratio"].as<double>();
        config.params.foot_overshoot_phase          = cfg["walk"]["foot"]["overshoot"]["phase"].as<double>();
        config.params.trunk_height                  = cfg["walk"]["trunk"]["height"].as<double>();
        config.params.trunk_pitch                   = cfg["walk"]["trunk"]["pitch"].as<Expression>();
        config.params.trunk_phase                   = cfg["walk"]["trunk"]["phase"].as<double>();
        config.params.trunk_x_offset                = cfg["walk"]["trunk"]["x_offset"].as<double>();
        config.params.trunk_y_offset                = cfg["walk"]["trunk"]["y_offset"].as<double>();
        config.params.trunk_swing                   = cfg["walk"]["trunk"]["swing"].as<double>();
        config.params.trunk_pause                   = cfg["walk"]["trunk"]["pause"].as<double>();
        config.params.trunk_x_offset_p_coef_forward = cfg["walk"]["trunk"]["x_offset_p_coef"]["forward"].as<double>();
        config.params.trunk_x_offset_p_coef_turn    = cfg["walk"]["trunk"]["x_offset_p_coef"]["turn"].as<double>();
        config.params.trunk_pitch_p_coef_forward =
            1.0 + cfg["walk"]["trunk"]["pitch_p_coef"]["forward"].as<Expression>();
        config.params.trunk_pitch_p_coef_turn = 1.0 + cfg["walk"]["trunk"]["pitch_p_coef"]["turn"].as<Expression>();
        config.params.kick_length             = cfg["walk"]["kick"]["length"].as<double>();
        config.params.kick_phase              = cfg["walk"]["kick"]["phase"].as<double>();
        config.params.kick_vel                = cfg["walk"]["kick"]["vel"].as<double>();
        config.params.pause_duration          = cfg["walk"]["pause"]["duration"].as<double>();

        config.max_step.x() = cfg["max_step"]["x"].as<double>();
        config.max_step.y() = cfg["max_step"]["y"].as<double>();
        config.max_step.z() = cfg["max_step"]["z"].as<double>();
        config.max_step_xy  = cfg["max_step"]["xy"].as<double>();

        config.imu_active          = cfg["imu"]["active"].as<bool>();
        config.imu_pitch_threshold = 1.0 + cfg["imu"]["pitch"]["threshold"].as<double>();
        config.imu_roll_threshold  = cfg["imu"]["roll"]["threshold"].as<double>();

        for (int id = 0; id < ServoID::NUMBER_OF_SERVOS; ++id) {
            // Sets the leg gains
            if ((id >= ServoID::R_HIP_YAW) && (id < ServoID::HEAD_YAW)) {
                config.jointGains[id] = cfg["gains"]["legs"].as<double>();
            }
            // Sets the arm gains
            if (id < ServoID::R_HIP_YAW) {
                config.jointGains[id] = cfg["gains"]["arms"].as<double>();
            }
        }

        config.arm_positions.emplace_back(ServoID::R_SHOULDER_PITCH, cfg["arms"]["right_shoulder_pitch"].as<double>());
        config.arm_positions.emplace_back(ServoID::L_SHOULDER_PITCH, cfg["arms"]["left_shoulder_pitch"].as<double>());
        config.arm_positions.emplace_back(ServoID::R_SHOULDER_ROLL, cfg["arms"]["right_shoulder_roll"].as<double>());
        config.arm_positions.emplace_back(ServoID::L_SHOULDER_ROLL, cfg["arms"]["left_shoulder_roll"].as<double>());
        config.arm_positions.emplace_back(ServoID::R_ELBOW, cfg["arms"]["right_elbow"].as<double>());
        config.arm_positions.emplace_back(ServoID::L_ELBOW, cfg["arms"]["left_elbow"].as<double>());
    }

    QuinticWalk::QuinticWalk(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        imu_reaction = on<Trigger<Sensors>>().then([this](const Sensors& sensors) {
            Eigen::Vector3d RPY =
                utility::math::euler::MatrixToEulerIntrinsic(sensors.Htw.topLeftCorner<3, 3>().cast<double>());

            // compute the pitch offset to the currently wanted pitch of the engine
            double wanted_pitch =
                current_config.params.trunk_pitch
                + current_config.params.trunk_pitch_p_coef_forward * walk_engine.get_footstep().get_next().x()
                + current_config.params.trunk_pitch_p_coef_turn * std::abs(walk_engine.get_footstep().get_next().z());
            RPY.y() += wanted_pitch;

            // threshold pitch and roll
            if (std::abs(RPY.x()) > current_config.imu_roll_threshold) {
                log<NUClear::WARN>(fmt::format("Robot roll exceeds threshold - {} > {}",
                                               std::abs(RPY.x()),
                                               current_config.imu_roll_threshold));
                walk_engine.request_pause();
            }
            else if (std::abs(RPY.y()) > current_config.imu_pitch_threshold) {
                log<NUClear::WARN>(fmt::format("Robot pitch exceeds threshold - {} > {}",
                                               std::abs(RPY.y()),
                                               current_config.imu_pitch_threshold));
                walk_engine.request_pause();
            }
        });

        on<Configuration>("quinticwalk.yaml").then([this](const Configuration& cfg) {
            log_level = cfg["log_level"].as<NUClear::LogLevel>();

            load_quintic_walk(cfg, normal_config);

            // Make sure the walk engine has the parameters at least once
            if (first_config) {
                // Send these parameters to the walk engine
                walk_engine.set_parameters(current_config.params);

                imu_reaction.enable(current_config.imu_active);

                first_config = false;
            }
        });

        on<Configuration>("goalie/quinticwalk.yaml").then([this](const Configuration& cfg) {
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
            walk_engine.set_parameters(current_config.params);

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
            subsumption_id = walkCommand.subsumption_id;
            current_orders.setZero();
        });

        on<Trigger<WalkCommand>>().then([this](const WalkCommand& walkCommand) {
            subsumption_id = walkCommand.subsumption_id;

            // the engine expects orders in [m] not [m/s]. We have to compute by dividing by step frequency which is
            // a double step factor 2 since the order distance is only for a single step, not double step
            const double factor            = (1.0 / (current_config.params.freq)) * 0.5;
            const Eigen::Vector3d& command = walkCommand.command.cast<double>() * factor;

            // Clamp velocity command
            Eigen::Vector3d orders =
                command.array().max(-current_config.max_step.array()).min(current_config.max_step.array()).matrix();

            // translational orders (x+y) should not exceed combined limit. scale if necessary
            if (current_config.max_step_xy != 0) {
                double scaling_factor = 1.0 / std::max(1.0, (orders.x() + orders.y()) / current_config.max_step_xy);
                orders.cwiseProduct(Eigen::Vector3d(scaling_factor, scaling_factor, 1.0));
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
            subsumption_id = command.subsumption_id;
            walk_engine.reset();
            update_handle.enable();
        });

        on<Trigger<DisableWalkEngineCommand>>().then([this](const DisableWalkEngineCommand& command) {
            subsumption_id = command.subsumption_id;
            update_handle.disable();
        });

        update_handle = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Single>().then([this]() {
            const double dt = get_time_delta();

            if (falling) {
                // We are falling, reset walk engine
                walk_engine.reset();
            }
            else {

                // see if the walk engine has new goals for us
                if (walk_engine.update_state(dt, current_orders)) {
                    calculate_joint_goals();
                }
            }
        });
    }

    double QuinticWalk::get_time_delta() {
        // compute time delta depended if we are currently in simulation or reality
        const auto current_time = NUClear::clock::now();
        double dt =
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

    /*
    This method computes the next motor goals and publishes them.
    */
    void QuinticWalk::calculate_joint_goals() {

        // Position of trunk {t} relative to support foot {s}
        Eigen::Vector3d rTSs = Eigen::Vector3d::Zero();
        // Euler angles [Roll, Pitch, Yaw] of trunk {t} relative to support foot {s}
        Eigen::Vector3d thetaST = Eigen::Vector3d::Zero();
        // Position of flying foot {f} relative to support foot {s}
        Eigen::Vector3d rFSs = Eigen::Vector3d::Zero();
        // Euler angles [Roll, Pitch, Yaw] of flying foot {f} relative to support foot {s}
        Eigen::Vector3d thetaSF = Eigen::Vector3d::Zero();

        // Read the cartesian positions and orientations for trunk and fly foot
        std::tie(rTSs, thetaST, rFSs, thetaSF, is_left_support) = walk_engine.compute_cartesian_position();

        // Change goals from support foot based coordinate system to trunk based coordinate system
        // Trunk {t} from support foot {s}
        Eigen::Isometry3d Hst;
        Hst.linear()      = EulerIntrinsicToMatrix(thetaST);
        Hst.translation() = rTSs;

        // Flying foot {f} from support foot {s}
        Eigen::Isometry3d Hsf;
        Hsf.linear()      = EulerIntrinsicToMatrix(thetaSF);
        Hsf.translation() = rFSs;

        // Support foot {s} from trunk {t}
        const Eigen::Isometry3d Hts = Hst.inverse();

        // Flying foot {f} from trunk {t}
        const Eigen::Isometry3d Htf = Hts * Hsf;

        // Get desired transform for left foot {l}
        const Eigen::Isometry3d Htl = walk_engine.get_footstep().is_left_support() ? Hts : Htf;

        // Get desired transform for right foot {r}
        const Eigen::Isometry3d Htr = walk_engine.get_footstep().is_left_support() ? Htf : Hts;

        // Compute inverse kinematics for left and right foot
        const auto joints = calculateLegJoints<double>(kinematicsModel, Htl, Htr);
        auto waypoints    = motion(joints);
        emit(std::move(waypoints));

        // Plot graphs of desired trajectories
        if (log_level <= NUClear::DEBUG) {
            Eigen::Vector3d thetaTL = MatrixToEulerIntrinsic(Htl.linear());
            emit(graph("Left foot desired position (x,y,z)", Htl(0, 3), Htl(1, 3), Htl(2, 3)));
            emit(graph("Left foot desired orientation (r,p,y)", thetaTL.x(), thetaTL.y(), thetaTL.z()));

            Eigen::Vector3d thetaTR = MatrixToEulerIntrinsic(Htr.linear());
            emit(graph("Right foot desired position (x,y,z)", Htr(0, 3), Htr(1, 3), Htr(2, 3)));
            emit(graph("Right foot desired orientation (r,p,y)", thetaTR.x(), thetaTR.y(), thetaTR.z()));

            emit(graph("Trunk desired position (x,y,z)", Hst(0, 3), Hst(1, 3), Hst(2, 3)));
            emit(graph("Trunk desired orientation (r,p,y)", thetaST.x(), thetaST.y(), thetaST.z()));
        }
    }

    std::unique_ptr<ServoCommands> QuinticWalk::motion(const std::vector<std::pair<ServoID, double>>& joints) {
        auto waypoints = std::make_unique<ServoCommands>();
        waypoints->commands.reserve(joints.size() + current_config.arm_positions.size());

        const NUClear::clock::time_point time = NUClear::clock::now() + Per<std::chrono::seconds>(UPDATE_FREQUENCY);

        for (const auto& joint : joints) {
            waypoints->commands.emplace_back(subsumption_id,
                                             time,
                                             joint.first,
                                             joint.second,
                                             current_config.jointGains[joint.first],
                                             100);
        }

        for (const auto& joint : current_config.arm_positions) {
            waypoints->commands.emplace_back(subsumption_id,
                                             time,
                                             joint.first,
                                             joint.second,
                                             current_config.jointGains[joint.first],
                                             100);
        }

        return waypoints;
    }
}  // namespace module::motion
