#include "K1WalkPolicy.hpp"

#include <cmath>
#include <Eigen/Geometry>

#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/booster/BoosterHeadRot.hpp"
#include "message/booster/BoosterLowCmd.hpp"
#include "message/booster/BoosterMode.hpp"
#include "message/booster/BoosterOdometry.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/skill/Walk.hpp"

#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::booster::BoosterHeadRot;
    using message::booster::BoosterLowCmd;
    using message::booster::BoosterMode;
    using message::booster::BoosterOdometry;
    using message::booster::K1Mode;
    using message::platform::RawSensors;
    using WalkTask = message::skill::Walk;

    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::nusight::graph;
    using utility::support::Expression;

    namespace {

        constexpr double PI     = 3.141592653589793;
        constexpr double TWO_PI = 6.283185307179586;

        // Booster SDK RotateHead limits: pitch down-positive [-0.3, 1.0], yaw [-0.785, 0.785]
        constexpr double HEAD_PITCH_MIN = -0.3;
        constexpr double HEAD_PITCH_MAX = 1.0;
        constexpr double HEAD_YAW_LIMIT = 0.785;

        // JointIndexK1 slots of the two head joints (masked in the observation, overridden
        // in the action -- the policy does not own the head at deployment)
        constexpr std::size_t HEAD_YAW   = 0;
        constexpr std::size_t HEAD_PITCH = 1;

        template <std::size_t N>
        std::array<double, N> load_joint_array(const Configuration& config, const char* key) {
            const auto values = config[key].as<std::vector<double>>();
            if (values.size() != N) {
                throw std::runtime_error(std::string("K1WalkPolicy.yaml: ") + key + " must have "
                                         + std::to_string(N) + " entries (JointIndexK1 order), got "
                                         + std::to_string(values.size()));
            }
            std::array<double, N> out{};
            std::copy(values.begin(), values.end(), out.begin());
            return out;
        }

    }  // namespace

    void K1WalkPolicy::reset_policy_state() {
        last_action.fill(0.0f);
        phase          = {0.0, PI};
        have_last_odom = false;
        linvel_body.setZero();
    }

    void K1WalkPolicy::update_linvel(const Eigen::Vector3d& odom_now) {
        const auto now = NUClear::clock::now();
        if (have_last_odom) {
            const double dt = std::chrono::duration<double>(now - last_odom_time).count();
            // Skip degenerate/stale intervals (startup, provider stalls); keep the old estimate
            if (dt > 1e-4 && dt < 0.5) {
                const Eigen::Vector2d v_world = (odom_now.head<2>() - last_odom.head<2>()) / dt;
                const Eigen::Vector2d v_body  = Eigen::Rotation2Dd(-odom_now.z()) * v_world;
                linvel_body = cfg.linvel_alpha * v_body + (1.0 - cfg.linvel_alpha) * linvel_body;
            }
        }
        last_odom      = odom_now;
        last_odom_time = now;
        have_last_odom = true;
    }

    K1WalkPolicy::K1WalkPolicy(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("K1WalkPolicy.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.model_path      = config["model_path"].as<std::string>();
            cfg.gait_frequency  = config["gait_frequency"].as<double>();
            cfg.stand_threshold = config["stand_threshold"].as<double>();
            cfg.linvel_alpha    = config["linvel_alpha"].as<double>();
            cfg.head_kp         = config["head"]["kp"].as<double>();
            cfg.head_kd         = config["head"]["kd"].as<double>();
            cfg.kick_velocity   = Eigen::Vector3d(config["kick"]["velocity"].as<Expression>());
            cfg.kick_duration   = std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["kick"]["duration"].as<double>()));

            cfg.kp                 = load_joint_array<JOINT_COUNT>(config, "kp");
            cfg.kd                 = load_joint_array<JOINT_COUNT>(config, "kd");
            cfg.action_scale_joint = load_joint_array<JOINT_COUNT>(config, "action_scale_joint");
            cfg.default_pose       = load_joint_array<JOINT_COUNT>(config, "default_pose");
            const double action_scale = config["action_scale"].as<double>();
            for (double& s : cfg.action_scale_joint) {
                s *= action_scale;
            }

            try {
                compiled_model = core.compile_model(cfg.model_path, "CPU");
                infer_request  = compiled_model.create_infer_request();
                model_loaded   = true;
                log<INFO>("Loaded walk policy", cfg.model_path);
            }
            catch (const std::exception& e) {
                model_loaded = false;
                log<ERROR>("Failed to load walk policy", cfg.model_path, e.what());
            }

            emit(std::make_unique<Stability>(Stability::UNKNOWN));
        });

        on<Trigger<BoosterHeadRot>>().then([this](const BoosterHeadRot& head) {
            head_target.x() = utility::math::clamp(-HEAD_YAW_LIMIT, head.rot.x(), HEAD_YAW_LIMIT);
            head_target.y() = utility::math::clamp(HEAD_PITCH_MIN, head.rot.y(), HEAD_PITCH_MAX);
        });

        on<Start<WalkTask>>().then([this]() {
            // Never take the robot into CUSTOM with nothing to stream: with no LowCmd
            // arriving the robot holds stale torques and collapses.
            if (!model_loaded) {
                log<ERROR>("Walk task started but no walk policy is loaded; staying out of CUSTOM mode");
                return;
            }
            reset_policy_state();
            // Low-level joint commands are only honoured in CUSTOM mode
            auto mode  = std::make_unique<BoosterMode>();
            mode->mode = K1Mode::CUSTOM;
            emit(std::move(mode));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
            last_walk_state = int(WalkState::State::STOPPED);
        });

        on<Stop<WalkTask>>().then([this] {
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
            last_walk_state = int(WalkState::State::STOPPED);
        });

        // 50 Hz inference loop, matching the training control rate (ctrl_dt = 0.02 s). The
        // simulator/robot PD-tracks the latest LowCmd between ticks.
        on<Provide<WalkTask>, Every<50, Per<std::chrono::seconds>>, With<RawSensors>, With<BoosterOdometry>,
           With<Stability>, Single>()
            .then([this](const WalkTask& walk, const RunReason& run_reason, const RawSensors& raw,
                         const BoosterOdometry& odo, const Stability& stability) {
                if (!model_loaded) {
                    return;
                }
                // While fallen/recovering the get-up policy owns the low-level channel
                // (K1GetUpPolicy emits Stability FALLEN on start, STANDING when done);
                // streaming two LowCmd sources at once would fight each other.
                if (stability == Stability::FALLEN) {
                    return;
                }

                // In-walk kick: no kick primitive in the policy, so emulate it as a forward
                // velocity burst held for kick_duration (same emulation as skill::K1Walk).
                Eigen::Vector3d cmd = walk.velocity_target;
                if (walk.kick) {
                    if (run_reason == RunReason::NEW_TASK) {
                        kick_start_time = NUClear::clock::now();
                        log<INFO>("K1WalkPolicy starting in-walk kick");
                    }
                    if (NUClear::clock::now() - kick_start_time > cfg.kick_duration) {
                        log<INFO>("K1WalkPolicy in-walk kick complete");
                        emit<Task>(std::make_unique<Done>());
                        return;
                    }
                    cmd = cfg.kick_velocity;
                    emit<Task>(std::make_unique<Continue>());
                }

                update_linvel(Eigen::Vector3d(odo.x, odo.y, odo.theta));
                if (log_level <= NUClear::LogLevel::DEBUG) {
                    emit(graph("Policy linvel estimate", linvel_body.x(), linvel_body.y()));
                    emit(graph("Policy command", cmd.x(), cmd.y(), cmd.z()));
                }

                // --- observation (NUSim docs/OBS_ACTION_CONTRACT.md, 82 floats) ---
                std::array<float, OBS_DIM> obs{};
                std::size_t idx = 0;

                // [0:3] base linear velocity, body frame (odometry-differentiated; z unobservable)
                obs[idx++] = static_cast<float>(linvel_body.x());
                obs[idx++] = static_cast<float>(linvel_body.y());
                obs[idx++] = 0.0f;

                // [3:6] gyro, body frame
                obs[idx++] = raw.gyroscope.x();
                obs[idx++] = raw.gyroscope.y();
                obs[idx++] = raw.gyroscope.z();

                // [6:9] projected gravity: world (0,0,-1) in the body frame, from the firmware
                // attitude estimate
                const Eigen::Matrix3d Rwt =
                    rpy_intrinsic_to_mat(Eigen::Vector3d(raw.imu_rpy.x(), raw.imu_rpy.y(), raw.imu_rpy.z()));
                const Eigen::Vector3d grav = Rwt.transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
                obs[idx++] = static_cast<float>(grav.x());
                obs[idx++] = static_cast<float>(grav.y());
                obs[idx++] = static_cast<float>(grav.z());

                // [9:12] command [vx, vy, vyaw]
                obs[idx++] = static_cast<float>(cmd.x());
                obs[idx++] = static_cast<float>(cmd.y());
                obs[idx++] = static_cast<float>(cmd.z());

                // Servo feedback in JointIndexK1 order (ankles come pre-converted to serial
                // pitch/roll by the platform)
                const RawSensors::Servo* servos[JOINT_COUNT] = {
                    &raw.servo.head_pan,         &raw.servo.head_tilt,
                    &raw.servo.l_shoulder_pitch, &raw.servo.l_shoulder_roll,
                    &raw.servo.l_elbow,          &raw.servo.l_elbow_yaw,
                    &raw.servo.r_shoulder_pitch, &raw.servo.r_shoulder_roll,
                    &raw.servo.r_elbow,          &raw.servo.r_elbow_yaw,
                    &raw.servo.l_hip_pitch,      &raw.servo.l_hip_roll,
                    &raw.servo.l_hip_yaw,        &raw.servo.l_knee,
                    &raw.servo.l_ankle_pitch,    &raw.servo.l_ankle_roll,
                    &raw.servo.r_hip_pitch,      &raw.servo.r_hip_roll,
                    &raw.servo.r_hip_yaw,        &raw.servo.r_knee,
                    &raw.servo.r_ankle_pitch,    &raw.servo.r_ankle_roll,
                };

                // [12:34] q - default_pose, [34:56] dq. Head entries are masked to the default
                // pose / zero velocity: the policy was trained with the head near default, and
                // real head deflections push the observation out of distribution.
                for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                    const bool head = (j == HEAD_YAW || j == HEAD_PITCH);
                    obs[idx++] =
                        head ? 0.0f : static_cast<float>(servos[j]->present_position - cfg.default_pose[j]);
                }
                for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                    const bool head = (j == HEAD_YAW || j == HEAD_PITCH);
                    obs[idx++]      = head ? 0.0f : servos[j]->present_velocity;
                }

                // [56:78] previous raw network output
                for (std::size_t k = 0; k < JOINT_COUNT; ++k) {
                    obs[idx++] = last_action[k];
                }

                // [78:82] gait phase [cos p0, cos p1, sin p0, sin p1], pinned to [pi, pi] while
                // the commanded speed is below the stand threshold
                const double speed = cmd.norm();
                const std::array<double, 2> ph =
                    speed >= cfg.stand_threshold ? phase : std::array<double, 2>{PI, PI};
                obs[idx++] = static_cast<float>(std::cos(ph[0]));
                obs[idx++] = static_cast<float>(std::cos(ph[1]));
                obs[idx++] = static_cast<float>(std::sin(ph[0]));
                obs[idx++] = static_cast<float>(std::sin(ph[1]));

                // --- inference ---
                ov::Tensor input(ov::element::f32, {1, OBS_DIM});
                std::copy(obs.begin(), obs.end(), input.data<float>());
                infer_request.set_input_tensor(input);
                infer_request.infer();
                const float* action = infer_request.get_output_tensor(0).data<float>();
                std::copy(action, action + JOINT_COUNT, last_action.begin());

                // Advance the gait phase once per inference regardless of the command (only the
                // *observed* phase is pinned while standing), wrapped to [-pi, pi)
                for (double& p : phase) {
                    p = std::fmod(p + TWO_PI * 0.02 * cfg.gait_frequency + PI, TWO_PI) - PI;
                }

                // --- action -> low-level joint command ---
                auto low      = std::make_unique<BoosterLowCmd>();
                low->cmd_type = BoosterLowCmd::CmdType::SERIAL;
                low->motor_cmd.resize(JOINT_COUNT);
                for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                    auto& motor = low->motor_cmd[j];
                    motor.mode  = 1;
                    motor.q     = static_cast<float>(cfg.default_pose[j]
                                                 + cfg.action_scale_joint[j] * last_action[j]);
                    motor.dq    = 0.0f;
                    motor.tau   = 0.0f;
                    motor.kp    = static_cast<float>(cfg.kp[j]);
                    motor.kd    = static_cast<float>(cfg.kd[j]);
                    motor.weight = 0.0f;
                }
                // The policy does not own the head: track the latest BoosterHeadRot instead
                low->motor_cmd[HEAD_YAW].q    = static_cast<float>(head_target.x());
                low->motor_cmd[HEAD_YAW].kp   = static_cast<float>(cfg.head_kp);
                low->motor_cmd[HEAD_YAW].kd   = static_cast<float>(cfg.head_kd);
                low->motor_cmd[HEAD_PITCH].q  = static_cast<float>(head_target.y());
                low->motor_cmd[HEAD_PITCH].kp = static_cast<float>(cfg.head_kp);
                low->motor_cmd[HEAD_PITCH].kd = static_cast<float>(cfg.head_kd);
                emit(std::move(low));

                const auto state = cmd.isZero() ? WalkState::State::STOPPED : WalkState::State::WALKING;
                if (int(state) != last_walk_state) {
                    emit(std::make_unique<WalkState>(state, cmd));
                    last_walk_state = int(state);
                }
            });
    }

}  // namespace module::skill
