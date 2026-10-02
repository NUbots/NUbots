#include "K1WalkPolicy.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/actuation/K1Servos.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/booster/BoosterHeadRot.hpp"
#include "message/booster/BoosterLowCmd.hpp"
#include "message/booster/BoosterMode.hpp"
#include "message/booster/BoosterModeState.hpp"
#include "message/booster/BoosterOdometryTwist.hpp"
#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/skill/Walk.hpp"

#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::actuation::K1Servos;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::booster::BoosterHeadRot;
    using message::booster::BoosterLowCmd;
    using message::booster::BoosterMode;
    using message::booster::BoosterModeState;
    using message::booster::BoosterOdometryTwist;
    using message::booster::K1Mode;
    using message::input::Sensors;
    using message::platform::RawSensors;
    using WalkTask = message::skill::Walk;

    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::nusight::graph;
    using utility::support::Expression;

    namespace {

        // Booster SDK RotateHead limits: pitch down-positive [-0.3, 1.0], yaw [-0.785, 0.785]
        constexpr double HEAD_PITCH_MIN = -0.3;
        constexpr double HEAD_PITCH_MAX = 1.0;
        constexpr double HEAD_YAW_LIMIT = 0.785;

        // Bounds on the measured loop period reported by the debug graph. A resumed provider or a
        // clock step must not show as a period of seconds.
        constexpr double MIN_TICK_DT = 0.005;
        constexpr double MAX_TICK_DT = 0.100;

        // The training control period. The observation window spans `history_window` control
        // *steps*, so a loop that misses this by much feeds the encoder a window of the wrong
        // duration -- which is a bug to fix, not to compensate for. Warn past this much error.
        constexpr double NOMINAL_TICK_DT   = 0.02;
        constexpr double TICK_DT_TOLERANCE = 0.005;

        // How often the loop-rate summary above may be logged. At 50 Hz a per-tick warning would
        // bury the log it is warning about.
        constexpr std::chrono::seconds TIMING_REPORT_PERIOD{2};

        // How often to re-ask for CUSTOM while the robot is not in it
        constexpr std::chrono::seconds MODE_RETRY_PERIOD{1};

        // How often a missing or stale base linear velocity may be warned about
        constexpr std::chrono::seconds LINEAR_VELOCITY_WARNING_PERIOD{2};  // remove

        template <std::size_t N>
        std::array<double, N> load_joint_array(const Configuration& config, const char* key) {
            const auto values = config[key].as<std::vector<double>>();
            if (values.size() != N) {
                throw std::runtime_error(std::string("K1WalkPolicy.yaml: ") + key + " must have " + std::to_string(N)
                                         + " entries (JointIndexK1 order), got " + std::to_string(values.size()));
            }
            std::array<double, N> out{};
            std::copy(values.begin(), values.end(), out.begin());
            return out;
        }

        /// Servo feedback in JointIndexK1 order (ankles pre-converted to serial by the platform)
        std::array<const RawSensors::Servo*, K1WalkPolicy::JOINT_COUNT> servos_of(const RawSensors& raw) {
            return {&raw.servo.head_pan,         &raw.servo.head_tilt,       &raw.servo.l_shoulder_pitch,
                    &raw.servo.l_shoulder_roll,  &raw.servo.l_elbow,         &raw.servo.l_elbow_yaw,
                    &raw.servo.r_shoulder_pitch, &raw.servo.r_shoulder_roll, &raw.servo.r_elbow,
                    &raw.servo.r_elbow_yaw,      &raw.servo.l_hip_pitch,     &raw.servo.l_hip_roll,
                    &raw.servo.l_hip_yaw,        &raw.servo.l_knee,          &raw.servo.l_ankle_pitch,
                    &raw.servo.l_ankle_roll,     &raw.servo.r_hip_pitch,     &raw.servo.r_hip_roll,
                    &raw.servo.r_hip_yaw,        &raw.servo.r_knee,          &raw.servo.r_ankle_pitch,
                    &raw.servo.r_ankle_roll};
        }

        template <typename Shape>
        std::size_t numel(const Shape& shape) {
            return std::accumulate(shape.begin(), shape.end(), std::size_t{1}, [](std::size_t n, auto d) {
                return n * static_cast<std::size_t>(d);
            });
        }

    }  // namespace

    void K1WalkPolicy::reset_policy_state() {
        std::fill(last_action.begin(), last_action.end(), 0.0f);
        history.clear();
        have_last_tick = false;

        off_rate_ticks     = 0;
        timing_report_tick = tick;
        last_timing_report = NUClear::clock::now();
    }

    K1WalkPolicy::K1WalkPolicy(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("K1WalkPolicy.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.model_path              = config["model_path"].as<std::string>();
            cfg.use_tensorrt            = config["use_tensorrt"].as<bool>();
            cfg.history_window          = config["history_window"].as<std::size_t>();
            cfg.linear_velocity_max_age = config["linear_velocity_max_age"].as<double>();
            cfg.handoff_blend           = config["handoff_blend"].as<double>();
            cfg.head_kp                 = config["head"]["kp"].as<double>();
            cfg.head_kd                 = config["head"]["kd"].as<double>();
            cfg.kick_velocity           = Eigen::Vector3d(config["kick"]["velocity"].as<Expression>());
            cfg.kick_duration           = std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["kick"]["duration"].as<double>()));

            if (cfg.history_window < 1) {
                throw std::runtime_error("K1WalkPolicy.yaml: history_window must be >= 1");
            }

            // The policy joints must be distinct, in range, and exclude the head (vision owns it)
            cfg.policy_joints = config["policy_joints"].as<std::vector<std::size_t>>();
            std::set<std::size_t> seen{};
            for (const std::size_t j : cfg.policy_joints) {
                if (j >= JOINT_COUNT || j == HEAD_YAW || j == HEAD_PITCH || !seen.insert(j).second) {
                    throw std::runtime_error(
                        "K1WalkPolicy.yaml: policy_joints must be distinct non-head JointIndexK1 indices, got "
                        + std::to_string(j));
                }
            }

            cfg.kp                    = load_joint_array<JOINT_COUNT>(config, "kp");
            cfg.kd                    = load_joint_array<JOINT_COUNT>(config, "kd");
            cfg.action_scale_joint    = load_joint_array<JOINT_COUNT>(config, "action_scale_joint");
            cfg.default_pose          = load_joint_array<JOINT_COUNT>(config, "default_pose");
            cfg.joint_lower           = load_joint_array<JOINT_COUNT>(config, "joint_lower");
            cfg.joint_upper           = load_joint_array<JOINT_COUNT>(config, "joint_upper");
            const double action_scale = config["action_scale"].as<double>();
            for (double& s : cfg.action_scale_joint) {
                s *= action_scale;
            }

            last_action.assign(cfg.policy_joints.size(), 0.0f);
            reset_policy_state();
            load_model();

            emit(std::make_unique<Stability>(Stability::UNKNOWN));
        });

        // Cached rather than a With<>: BoosterModeState only exists on the real HardwareIO path,
        // and a With<> would silently stop the whole walk provider in NUSim.
        on<Trigger<BoosterModeState>>().then([this](const BoosterModeState& state) {  //
            last_mode = int(state.mode);
        });

        // The base linear velocity the policy observes: the controller's odometry twist, in the body
        // frame (the twist's child_frame_id, by ROS convention), as the training velocimeter at the
        // trunk's IMU site measures it
        on<Trigger<BoosterOdometryTwist>>().then([this](const BoosterOdometryTwist& twist) {
            const std::lock_guard<std::mutex> lock(linear_velocity_mutex);
            linear_velocity = twist.linear;
            emit(graph("base velocity", linear_velocity.x(), linear_velocity.y(), linear_velocity.z()));
            linear_velocity_time = NUClear::clock::now();
            have_linear_velocity = true;
        });

        // The policy does not own the head: track whatever the look skills last asked for
        on<Trigger<BoosterHeadRot>>().then([this](const BoosterHeadRot& head) {
            head_target.x() = std::clamp(head.rot.x(), -HEAD_YAW_LIMIT, HEAD_YAW_LIMIT);
            head_target.y() = std::clamp(head.rot.y(), HEAD_PITCH_MIN, HEAD_PITCH_MAX);
        });

        on<Start<WalkTask>>().then([this]() {
            // Never take the robot into CUSTOM with nothing to stream: with no LowCmd arriving the
            // robot holds stale torques and collapses.
            if (!model_loaded) {
                log<ERROR>("Walk task started but no walk policy is loaded; staying out of CUSTOM mode");
                return;
            }
            tick = 0;
            reset_policy_state();
            walk_since = NUClear::clock::now();

            // Low-level joint commands are only honoured in CUSTOM mode
            auto mode  = std::make_unique<BoosterMode>();
            mode->mode = K1Mode::CUSTOM;
            emit(std::move(mode));

            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
            last_walk_state   = int(WalkState::State::STOPPED);
            last_walk_command = Eigen::Vector3d::Zero();
        });

        on<Stop<WalkTask>>().then([this] {
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3d::Zero()));
            last_walk_state   = int(WalkState::State::STOPPED);
            last_walk_command = Eigen::Vector3d::Zero();
        });

        // 50 Hz inference loop, matching the training control rate (0.02 s). The simulator/robot
        // PD-tracks the latest LowCmd between ticks.
        on<Provide<WalkTask>,
           Every<50, Per<std::chrono::seconds>>,
           With<RawSensors>,
           With<Sensors>,
           With<Stability>,
           Single>()
            .then([this](const WalkTask& walk,
                         const RunReason& run_reason,
                         const RawSensors& raw,
                         const Sensors& sensors,
                         const Stability& stability) {
                if (!model_loaded) {
                    return;
                }
                // While fallen/recovering the get-up policy owns the low-level channel
                // (K1GetUpPolicy emits Stability FALLEN on start, STANDING when done); streaming
                // two LowCmd sources at once would fight each other. Drop the window rather than
                // let the next walk tick read a 0.5 s history that straddles the fall.
                if (stability == Stability::FALLEN) {
                    reset_policy_state();
                    return;
                }

                // Re-request CUSTOM until the robot is actually in it. The Walk task can go active
                // before this module's configuration has loaded -- strategy::StandStill emits
                // Walk(0,0,0) at startup -- in which case Start<WalkTask> hit the !model_loaded
                // guard, returned without requesting CUSTOM, and will never fire again because the
                // task never stopped. Asking from here instead keeps the guard's property (never
                // enter CUSTOM with nothing to stream: this loop only runs once the model is
                // loaded) without depending on winning that race. HardwareIO no-ops a request for
                // the mode the robot is already in, so this is idempotent; rate-limited anyway so
                // a robot that refuses the mode does not get an RPC every 20 ms.
                if (last_mode != int(K1Mode::CUSTOM) && NUClear::clock::now() - last_mode_request > MODE_RETRY_PERIOD) {
                    last_mode_request = NUClear::clock::now();
                    auto mode         = std::make_unique<BoosterMode>();
                    mode->mode        = K1Mode::CUSTOM;
                    emit(std::move(mode));
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

                // Measured control period. The observation window spans history_window *steps*, so a
                // loop off the trained 0.02 s is a bug to fix: say so rather than absorb it quietly.
                const auto now = NUClear::clock::now();
                double tick_dt = NOMINAL_TICK_DT;
                if (have_last_tick) {
                    tick_dt = std::chrono::duration<double>(now - last_tick_time).count();
                    off_rate_ticks += std::abs(tick_dt - NOMINAL_TICK_DT) > TICK_DT_TOLERANCE ? 1 : 0;
                    tick_dt = utility::math::clamp(MIN_TICK_DT, tick_dt, MAX_TICK_DT);
                }
                last_tick_time = now;
                have_last_tick = true;

                // Rate-limited: at 50 Hz a per-tick warning would bury the log it is warning about
                if (now - last_timing_report > TIMING_REPORT_PERIOD) {
                    if (off_rate_ticks > 0) {
                        log<WARN>("K1WalkPolicy missed the trained 50 Hz on",
                                  off_rate_ticks,
                                  "of the last",
                                  tick - timing_report_tick,
                                  "ticks; the observation window spans the wrong duration");
                    }
                    off_rate_ticks     = 0;
                    timing_report_tick = tick;
                    last_timing_report = now;
                }

                if (log_level <= NUClear::LogLevel::DEBUG) {
                    emit(graph("Policy command", cmd.x(), cmd.y(), cmd.z()));
                    emit(graph("Policy loop period (s)", tick_dt));
                }

                const auto servos = servos_of(raw);  // FIXME we shouldn't be using raw sensors

                // --- observation frame (72 floats; see README.md) ---
                std::vector<float> frame{};
                frame.reserve(frame_dim());

                // [0:3] base linear velocity, body frame, from the odometry twist. Without a recent
                // one the policy is shown zero, which it reads as standing still: warn, because it
                // will walk badly on it.
                Eigen::Vector3d base_velocity = Eigen::Vector3d::Zero();
                {
                    const std::lock_guard<std::mutex> lock(linear_velocity_mutex);
                    const bool fresh = have_linear_velocity
                                       && std::chrono::duration<double>(now - linear_velocity_time).count()
                                              <= cfg.linear_velocity_max_age;
                    if (fresh) {
                        base_velocity = linear_velocity;
                    }
                    else if (now - last_linear_velocity_warning > LINEAR_VELOCITY_WARNING_PERIOD) {
                        last_linear_velocity_warning = now;
                        log<WARN>("No base linear velocity from rt/odom in the last",
                                  cfg.linear_velocity_max_age,
                                  "s: the walk policy is observing zero");
                    }
                }
                // frame.push_back(static_cast<float>(base_velocity.x()));
                // frame.push_back(static_cast<float>(base_velocity.y()));
                // frame.push_back(static_cast<float>(base_velocity.z()));

                // [3:6] gyro, body frame
                frame.push_back(sensors.gyroscope.x());
                frame.push_back(sensors.gyroscope.y());
                frame.push_back(sensors.gyroscope.z());

                // [6:9] projected gravity: world (0,0,-1) in the body frame, from the firmware
                // attitude estimate

                // const Eigen::Matrix3d Rwt =
                //     rpy_intrinsic_to_mat(Eigen::Vector3d(raw.imu_rpy.x(), raw.imu_rpy.y(), raw.imu_rpy.z()));
                // const Eigen::Vector3d gravity = Rwt.transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
                const Eigen::Vector3d g_world(0.0, 0.0, -1.0);
                Eigen::Vector3d gravity = sensors.Htw.rotation() * g_world;
                frame.push_back(static_cast<float>(gravity.x()));
                frame.push_back(static_cast<float>(gravity.y()));
                frame.push_back(static_cast<float>(gravity.z()));
                log<INFO>("gravity", gravity.x(), gravity.y(), gravity.z());

                // [9:29] q - default_pose, [29:49] dq, both over the policy joints only
                for (const std::size_t j : cfg.policy_joints) {
                    frame.push_back(static_cast<float>(servos[j]->present_position - cfg.default_pose[j]));
                }
                for (const std::size_t j : cfg.policy_joints) {
                    frame.push_back(servos[j]->present_velocity);
                }

                // [49:69] previous raw network output
                frame.insert(frame.end(), last_action.begin(), last_action.end());

                // [69:72] command [vx, vy, wz], passed through as-is
                frame.push_back(static_cast<float>(cmd.x()));
                frame.push_back(static_cast<float>(cmd.y()));
                frame.push_back(static_cast<float>(cmd.z()));

                // --- observation window: seed by repeating the first frame, as the training-side
                // circular buffer backfills on reset ---
                if (history.empty()) {
                    history.assign(cfg.history_window, frame);
                }
                else {
                    history.push_back(frame);
                    while (history.size() > cfg.history_window) {
                        history.pop_front();
                    }
                }
                std::vector<float> input{};
                input.reserve(cfg.history_window * frame_dim());
                for (const auto& f : history) {
                    input.insert(input.end(), f.begin(), f.end());
                }

                // --- inference ---
                const std::vector<float> action = infer(input);
                std::copy_n(action.begin(), last_action.size(), last_action.begin());

                // Full observation trace. Statistics over a log that mixes CUSTOM-mode walking with
                // frozen non-CUSTOM ticks are meaningless, so every record carries the tick counter,
                // the robot's reported mode and a timestamp; tools/analysis/segment_walk_log.py
                // splits a capture on those before reporting anything.
                if (log_level <= NUClear::LogLevel::TRACE) {
                    std::ostringstream line;
                    line << "WALKOBS " << tick << " mode=" << last_mode
                         << " t=" << std::chrono::duration<double>(now.time_since_epoch()).count() << " dt=" << tick_dt;
                    for (const float v : frame) {
                        line << ' ' << v;
                    }
                    log<TRACE>(line.str());
                }
                ++tick;

                // --- action -> joint targets: offsets on the default pose for the policy joints,
                // the latest look target for the head, the default pose for anything else ---
                std::array<double, JOINT_COUNT> target = cfg.default_pose;
                std::array<double, JOINT_COUNT> kp     = cfg.kp;
                std::array<double, JOINT_COUNT> kd     = cfg.kd;
                for (std::size_t k = 0; k < cfg.policy_joints.size(); ++k) {
                    const std::size_t j = cfg.policy_joints[k];
                    target[j]           = cfg.default_pose[j] + cfg.action_scale_joint[j] * last_action[k];
                }
                target[HEAD_YAW]   = head_target.x();
                target[HEAD_PITCH] = head_target.y();
                kp[HEAD_YAW] = kp[HEAD_PITCH] = cfg.head_kp;
                kd[HEAD_YAW] = kd[HEAD_PITCH] = cfg.head_kd;

                // Cross-fade from the measured pose into the policy target: training always starts
                // at the default pose, deployment enters CUSTOM from wherever PREP left the robot.
                const double blend_age = std::chrono::duration<double>(now - walk_since).count();
                const double alpha     = cfg.handoff_blend > 0.0 ? std::min(1.0, blend_age / cfg.handoff_blend) : 1.0;

                auto low      = std::make_unique<BoosterLowCmd>();
                low->cmd_type = BoosterLowCmd::CmdType::SERIAL;
                low->motor_cmd.resize(JOINT_COUNT);
                std::array<double, JOINT_COUNT> cmd_q{};
                // How many of this tick's targets the joint-range clamp had to move. Non-zero means
                // the policy is asking for a pose the trained ranges do not contain, which on the
                // robot is a limb driving into a mechanical stop.
                int clamped = 0;
                for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                    const double blended = (1.0 - alpha) * servos[j]->present_position + alpha * target[j];
                    const double q       = std::clamp(blended, cfg.joint_lower[j], cfg.joint_upper[j]);
                    clamped += q != blended ? 1 : 0;

                    auto& motor  = low->motor_cmd[j];
                    motor.mode   = 1;
                    motor.q      = static_cast<float>(q);
                    motor.dq     = 0.0f;
                    motor.tau    = 0.0f;
                    motor.kp     = static_cast<float>(kp[j]);
                    motor.kd     = static_cast<float>(kd[j]);
                    motor.weight = 0.0f;
                    cmd_q[j]     = q;
                }
                if (clamped > 0) {
                    log<DEBUG>("Clamped", clamped, "joint targets to the joint limits");
                }

                // Emit the command as a Director-arbitrated K1Servos subtask (not a raw
                // BoosterLowCmd), so the Director owns the low-level channel and subsumes walk when
                // a higher-priority kick or get-up is active.
                auto k1_servos     = std::make_unique<K1Servos>();
                k1_servos->command = *low;
                emit<Task>(std::move(k1_servos));

                const auto state = cmd.isZero() ? WalkState::State::STOPPED : WalkState::State::WALKING;
                if (state == WalkState::State::WALKING && log_level <= NUClear::LogLevel::DEBUG) {
                    std::ostringstream out;
                    out << std::fixed << std::setprecision(4);
                    out << "K1WalkPolicy sim2real"
                        // The command the planner actually asked for. Training's final envelope is
                        // vx [-1.0, 2.0], vy [-0.8, 0.8], wz [-2.0, 2.0].
                        << " cmd=[" << cmd.x() << ',' << cmd.y() << ',' << cmd.z() << ']' << " base_velocity=["
                        << base_velocity.x() << ',' << base_velocity.y() << ',' << base_velocity.z() << ']'
                        << " clamped=" << clamped;
                    std::ostringstream action_stream;
                    std::ostringstream joint_pos_rel_stream;
                    std::ostringstream joint_vel_stream;
                    std::ostringstream cmd_q_stream;
                    for (std::size_t k = 0; k < cfg.policy_joints.size(); ++k) {
                        const std::size_t j = cfg.policy_joints[k];
                        if (k != 0) {
                            action_stream << ',';
                            joint_pos_rel_stream << ',';
                            joint_vel_stream << ',';
                            cmd_q_stream << ',';
                        }
                        action_stream << last_action[k];
                        joint_pos_rel_stream << (servos[j]->present_position - cfg.default_pose[j]);
                        joint_vel_stream << servos[j]->present_velocity;
                        cmd_q_stream << cmd_q[j];
                    }
                    out << " action=[" << action_stream.str() << ']' << " joint_pos_rel=[" << joint_pos_rel_stream.str()
                        << ']' << " joint_vel=[" << joint_vel_stream.str() << ']' << " cmd_q=[" << cmd_q_stream.str()
                        << ']';
                    log<DEBUG>(out.str());
                }

                // Re-emit on a command change too: velocity_target is what Overview and
                // RobotCommunication report as the walk command.
                if (int(state) != last_walk_state || cmd != last_walk_command) {
                    emit(std::make_unique<WalkState>(state, cmd));
                    last_walk_state   = int(state);
                    last_walk_command = cmd;
                }
            });
    }

    void K1WalkPolicy::load_model() {
        model_loaded = false;
        trt.reset();

        const std::size_t expected_in  = cfg.history_window * frame_dim();
        const std::size_t expected_out = cfg.policy_joints.size();
        const auto check               = [&](std::size_t in, std::size_t out) {
            if (in != expected_in || out != expected_out) {
                throw std::runtime_error("ONNX I/O is " + std::to_string(in) + " -> " + std::to_string(out)
                                         + " but the configured contract is " + std::to_string(cfg.history_window)
                                         + " x " + std::to_string(frame_dim()) + " = " + std::to_string(expected_in)
                                         + " -> " + std::to_string(expected_out));
            }
        };

        // TensorRT first, OpenVINO CPU as the fallback. A contract mismatch is fatal either way:
        // falling back would only load the same wrong graph on a different device.
        if (cfg.use_tensorrt) {
            try {
                trt = std::make_unique<utility::vision::TensorRT>(cfg.model_path, false);
            }
            catch (const std::exception& e) {
                trt.reset();
                log<INFO>("TensorRT unavailable, falling back to OpenVINO:", e.what());
            }
        }

        try {
            if (trt) {
                check(numel(trt->input_shape()), numel(trt->output_shape()));
                log<INFO>("Loaded walk policy (TensorRT)", cfg.model_path);
            }
            else {
                compiled_model = core.compile_model(cfg.model_path, "CPU");
                check(numel(compiled_model.input().get_shape()), numel(compiled_model.output().get_shape()));
                infer_request = compiled_model.create_infer_request();
                log<INFO>("Loaded walk policy (OpenVINO CPU)", cfg.model_path);
            }
            model_loaded = true;
        }
        catch (const std::exception& e) {
            trt.reset();
            log<ERROR>("Failed to load walk policy", cfg.model_path, e.what());
        }
    }

    std::vector<float> K1WalkPolicy::infer(const std::vector<float>& input) {
        if (trt) {
            return trt->infer(input);
        }
        ov::Tensor tensor(ov::element::f32, {1, input.size()});
        std::copy(input.begin(), input.end(), tensor.data<float>());
        infer_request.set_input_tensor(tensor);
        infer_request.infer();
        const ov::Tensor output = infer_request.get_output_tensor(0);
        const float* data       = output.data<float>();
        return std::vector<float>(data, data + output.get_size());
    }

}  // namespace module::skill
