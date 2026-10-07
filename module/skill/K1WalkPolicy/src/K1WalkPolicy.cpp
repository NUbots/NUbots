#include "K1WalkPolicy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <Eigen/Geometry>

#include "extension/Configuration.hpp"

#include "message/actuation/K1Servos.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/booster/BoosterHeadRot.hpp"
#include "message/booster/BoosterLowCmd.hpp"
#include "message/booster/BoosterMode.hpp"
#include "message/booster/BoosterModeState.hpp"
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
    using message::booster::K1Mode;
    using message::platform::RawSensors;
    using WalkTask = message::skill::Walk;

    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::nusight::graph;
    using utility::support::Expression;

    namespace {

        constexpr double TWO_PI = 6.283185307179586;

        // Booster SDK RotateHead limits: pitch down-positive [-0.3, 1.0], yaw [-0.785, 0.785]
        constexpr double HEAD_PITCH_MIN = -0.3;
        constexpr double HEAD_PITCH_MAX = 1.0;
        constexpr double HEAD_YAW_LIMIT = 0.785;

        // Bounds on the measured loop period used to advance the gait phase. A resumed provider or
        // a clock step must not fling the phase forward.
        constexpr double MIN_TICK_DT = 0.005;
        constexpr double MAX_TICK_DT = 0.100;

        // The training control period. The joint velocities, the action-rate dynamics and (for a
        // history policy) the duration the observation window spans are all trained against it, so
        // a loop that misses it by much is a bug to fix, not to compensate for. Warn past this
        // much error.
        constexpr double NOMINAL_TICK_DT   = 0.02;
        constexpr double TICK_DT_TOLERANCE = 0.005;

        // How often the loop-rate summary above may be logged. At 50 Hz a per-tick warning would
        // bury the log it is warning about.
        constexpr std::chrono::seconds TIMING_REPORT_PERIOD{2};

        // How often to re-ask for CUSTOM while the robot is not in it
        constexpr std::chrono::seconds MODE_RETRY_PERIOD{1};

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
        gait_phase     = 0.0;
        have_last_tick = false;

        off_rate_ticks     = 0;
        timing_report_tick = tick;
        last_timing_report = NUClear::clock::now();
    }

    K1WalkPolicy::K1WalkPolicy(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("K1WalkPolicy.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.model_path             = config["model_path"].as<std::string>();
            cfg.device                 = config["device"].as<std::string>();
            cfg.history_window         = config["history_window"].as<std::size_t>();
            cfg.gait_clock             = config["gait_clock"].as<bool>();
            cfg.gait_period            = config["gait_period"].as<double>();
            cfg.command_threshold      = config["command_threshold"].as<double>();
            cfg.handoff_blend          = config["handoff_blend"].as<double>();
            cfg.head_policy_controlled = config["head"]["policy_controlled"].as<bool>();
            cfg.head_kp                = config["head"]["kp"].as<double>();
            cfg.head_kd                = config["head"]["kd"].as<double>();
            cfg.kick_velocity          = Eigen::Vector3d(config["kick"]["velocity"].as<Expression>());
            cfg.kick_duration          = std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["kick"]["duration"].as<double>()));

            if (cfg.history_window < 1) {
                throw std::runtime_error("K1WalkPolicy.yaml: history_window must be >= 1");
            }
            if (cfg.gait_clock && cfg.gait_period <= 0.0) {
                throw std::runtime_error("K1WalkPolicy.yaml: gait_period must be > 0 when gait_clock is set");
            }

            // The policy joints must be distinct and in range. The head may or may not be among
            // them: the current policy acts on all 22 joints, the previous one on the 20 non-head
            // joints. Which of the two drives the head at run time is head.policy_controlled, not
            // this list -- the head still has to be *observed* when the policy was trained with it.
            cfg.policy_joints = config["policy_joints"].as<std::vector<std::size_t>>();
            std::set<std::size_t> seen{};
            for (const std::size_t j : cfg.policy_joints) {
                if (j >= JOINT_COUNT || !seen.insert(j).second) {
                    throw std::runtime_error(
                        "K1WalkPolicy.yaml: policy_joints must be distinct JointIndexK1 indices in [0, "
                        + std::to_string(JOINT_COUNT) + "), got " + std::to_string(j));
                }
            }
            // Nothing else drives the head, so asking the policy for it when it has no head action
            // would leave the head pinned at the default pose with no way to look at the ball.
            if (cfg.head_policy_controlled && (seen.count(HEAD_YAW) == 0 || seen.count(HEAD_PITCH) == 0)) {
                throw std::runtime_error("K1WalkPolicy.yaml: head.policy_controlled is set but the head joints are "
                                         "not in policy_joints, so the policy has no head action to apply");
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
        on<Provide<WalkTask>, Every<50, Per<std::chrono::seconds>>, With<RawSensors>, With<Stability>, Single>().then(
            [this](const WalkTask& walk,
                   const RunReason& run_reason,
                   const RawSensors& raw,
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

                // Measured control period. The gait phase is advanced on this rather than on a
                // hardcoded 0.02 s, so a loop running slow keeps the gait at its trained
                // wall-clock rate. The observation window still spans history_window *steps*
                // though, so a slow loop is a bug to fix: say so rather than absorb it quietly.
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
                        log<WARN>("K1WalkPolicy missed the trained 50 Hz on", off_rate_ticks, "of the last",
                                  tick - timing_report_tick, "ticks; the policy is seeing the wrong dynamics");
                    }
                    off_rate_ticks     = 0;
                    timing_report_tick = tick;
                    last_timing_report = now;
                }

                if (log_level <= NUClear::LogLevel::DEBUG) {
                    emit(graph("Policy command", cmd.x(), cmd.y(), cmd.z()));
                    emit(graph("Policy loop period (s)", tick_dt));
                }

                const auto servos = servos_of(raw);

                // --- observation frame (frame_dim() floats; see README.md) ---
                std::vector<float> frame{};
                frame.reserve(frame_dim());

                // [0:3] gyro, body frame
                frame.push_back(raw.gyroscope.x());
                frame.push_back(raw.gyroscope.y());
                frame.push_back(raw.gyroscope.z());

                // [3:6] projected gravity: world (0,0,-1) in the body frame, from the firmware
                // attitude estimate
                const Eigen::Matrix3d Rwt =
                    rpy_intrinsic_to_mat(Eigen::Vector3d(raw.imu_rpy.x(), raw.imu_rpy.y(), raw.imu_rpy.z()));
                const Eigen::Vector3d gravity = Rwt.transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
                frame.push_back(static_cast<float>(gravity.x()));
                frame.push_back(static_cast<float>(gravity.y()));
                frame.push_back(static_cast<float>(gravity.z()));

                // q - default_pose, then dq, both over the policy joints and in policy order
                for (const std::size_t j : cfg.policy_joints) {
                    frame.push_back(static_cast<float>(servos[j]->present_position - cfg.default_pose[j]));
                }
                for (const std::size_t j : cfg.policy_joints) {
                    frame.push_back(servos[j]->present_velocity);
                }

                // previous raw network output
                frame.insert(frame.end(), last_action.begin(), last_action.end());

                // command [vx, vy, wz], passed through as-is
                frame.push_back(static_cast<float>(cmd.x()));
                frame.push_back(static_cast<float>(cmd.y()));
                frame.push_back(static_cast<float>(cmd.z()));

                // Gait clock, for the policies trained against one. Collapsed to (0, 0) -- off the
                // unit circle, not pinned to a phase -- while the command is below threshold, which
                // is the distinct "standing" input those policies were trained on. The internal
                // phase keeps advancing regardless, as it does in training where it is indexed on
                // episode time.
                if (cfg.gait_clock) {
                    const double command_magnitude = cmd.head<2>().norm() + std::abs(cmd.z());
                    const bool clock_active        = command_magnitude > cfg.command_threshold;
                    frame.push_back(clock_active ? static_cast<float>(std::sin(TWO_PI * gait_phase)) : 0.0f);
                    frame.push_back(clock_active ? static_cast<float>(std::cos(TWO_PI * gait_phase)) : 0.0f);
                }

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

                // Advance the gait phase once per inference regardless of the command (only the
                // *observed* clock is gated), wrapped into [0, 1)
                if (cfg.gait_clock) {
                    gait_phase = std::fmod(gait_phase + tick_dt / cfg.gait_period, 1.0);
                }

                // Full observation trace. Statistics over a log that mixes CUSTOM-mode walking with
                // frozen non-CUSTOM ticks are meaningless, so every record carries the tick counter,
                // the robot's reported mode and a timestamp; tools/analysis/segment_walk_log.py
                // splits a capture on those before reporting anything.
                if (log_level <= NUClear::LogLevel::TRACE) {
                    std::ostringstream line;
                    line << "WALKOBS " << tick << " mode=" << last_mode
                         << " t=" << std::chrono::duration<double>(now.time_since_epoch()).count()
                         << " dt=" << tick_dt;
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
                // The head is the one place deployment deliberately overrides the policy: vision
                // has to be able to look at the ball. The policy still saw its own head action fed
                // back in the observation above, so only the applied target differs from training.
                if (!cfg.head_policy_controlled) {
                    target[HEAD_YAW]   = head_target.x();
                    target[HEAD_PITCH] = head_target.y();
                    kp[HEAD_YAW] = kp[HEAD_PITCH] = cfg.head_kp;
                    kd[HEAD_YAW] = kd[HEAD_PITCH] = cfg.head_kd;
                }

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
                        // The command the planner actually asked for. See README.md for the
                        // trained envelope; commands are passed through unclipped, as in training.
                        << " cmd=[" << cmd.x() << ',' << cmd.y() << ',' << cmd.z() << ']';
                    if (cfg.gait_clock) {
                        out << " phase=" << gait_phase;
                    }
                    out << " clamped=" << clamped;
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
                    out << " action=[" << action_stream.str() << ']' << " joint_pos_rel=["
                        << joint_pos_rel_stream.str() << ']' << " joint_vel=[" << joint_vel_stream.str() << ']'
                        << " cmd_q=[" << cmd_q_stream.str() << ']';
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
        onnx_rt.reset();

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

        // A contract mismatch is fatal on any device: falling back would only load the same wrong
        // graph on the CPU. FP16 is off, see K1WalkPolicy.yaml.
        try {
            std::string device = cfg.device;
            try {
                onnx_rt = std::make_unique<utility::onnx::ONNXRuntime>(cfg.model_path, device, false);
            }
            catch (const std::exception& e) {
                if (device != "gpu") {
                    throw;
                }
                log<INFO>("TensorRT unavailable, falling back to the CPU:", e.what());
                device  = "cpu";
                onnx_rt = std::make_unique<utility::onnx::ONNXRuntime>(cfg.model_path, device, false);
            }
            check(numel(onnx_rt->input_shape()), numel(onnx_rt->output_shape()));
            model_loaded = true;
            log<INFO>("Loaded walk policy on", device, cfg.model_path);
        }
        catch (const std::exception& e) {
            onnx_rt.reset();
            log<ERROR>("Failed to load walk policy", cfg.model_path, e.what());
        }
    }

    std::vector<float> K1WalkPolicy::infer(const std::vector<float>& input) {
        return onnx_rt->infer(input);
    }

}  // namespace module::skill
