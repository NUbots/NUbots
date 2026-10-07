#include "K1KickPolicy.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/actuation/K1Servos.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/booster/BoosterLowCmd.hpp"
#include "message/booster/BoosterMode.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/skill/Kick.hpp"

#include "utility/math/euler.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::booster::BoosterLowCmd;
    using message::booster::BoosterMode;
    using message::booster::K1Mode;
    using message::input::Sensors;
    using message::platform::RawSensors;
    using LocalisationBall = message::localisation::Ball;
    using KickTask         = message::skill::Kick;

    using utility::math::euler::rpy_intrinsic_to_mat;

    namespace {

        template <std::size_t N>
        std::array<double, N> load_joint_array(const Configuration& config, const char* key) {
            const auto values = config[key].as<std::vector<double>>();
            if (values.size() != N) {
                throw std::runtime_error(std::string("K1KickPolicy.yaml: ") + key + " must have "
                                         + std::to_string(N) + " entries (JointIndexK1 order), got "
                                         + std::to_string(values.size()));
            }
            std::array<double, N> out{};
            std::copy(values.begin(), values.end(), out.begin());
            return out;
        }

        /// Number of elements in a tensor shape
        std::size_t numel(const std::vector<int64_t>& shape) {
            return std::accumulate(shape.begin(), shape.end(), std::size_t{1}, [](std::size_t n, int64_t d) {
                return n * static_cast<std::size_t>(d);
            });
        }

    }  // namespace

    K1KickPolicy::K1KickPolicy(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("K1KickPolicy.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.model_path                = config["model_path"].as<std::string>();
            cfg.device                    = config["device"].as<std::string>();
            cfg.kick_duration             = config["kick_duration"].as<double>();
            cfg.ball_confidence_threshold = config["ball_confidence_threshold"].as<double>();
            cfg.ball_stale_timeout        = config["ball_stale_timeout"].as<double>();
            cfg.gait_frequency            = config["gait_frequency"].as<double>();
            cfg.handoff_blend             = config["handoff_blend"].as<double>();
            const auto dir                = config["kick_direction"].as<std::vector<double>>();
            if (dir.size() != 2) {
                throw std::runtime_error("K1KickPolicy.yaml: kick_direction must have 2 entries");
            }
            cfg.kick_direction = {dir[0], dir[1]};

            cfg.kp                 = load_joint_array<JOINT_COUNT>(config, "kp");
            cfg.kd                 = load_joint_array<JOINT_COUNT>(config, "kd");
            cfg.action_scale_joint = load_joint_array<JOINT_COUNT>(config, "action_scale_joint");
            cfg.default_pose       = load_joint_array<JOINT_COUNT>(config, "default_pose");
            const double action_scale = config["action_scale"].as<double>();
            for (double& s : cfg.action_scale_joint) {
                s *= action_scale;
            }

            // GPU first, CPU as the fallback so a machine without a CUDA device (or with a driver
            // mismatch) still runs. fp16 is off: the net is a small MLP, so there is no speed to
            // win and the actions drive servos directly.
            model_loaded = false;
            onnx_rt.reset();
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
                if (numel(onnx_rt->input_shape()) != OBS_DIM || numel(onnx_rt->output_shape()) != JOINT_COUNT) {
                    throw std::runtime_error("ONNX I/O is " + std::to_string(numel(onnx_rt->input_shape())) + " -> "
                                             + std::to_string(numel(onnx_rt->output_shape())) + " but expected "
                                             + std::to_string(OBS_DIM) + " -> " + std::to_string(JOINT_COUNT));
                }
                model_loaded = true;
                log<INFO>("Loaded kick policy on", device, cfg.model_path);
            }
            catch (const std::exception& e) {
                onnx_rt.reset();
                log<ERROR>("Failed to load kick policy", cfg.model_path, e.what());
            }
        });

        // Cache the latest ball estimate; the obs transforms it into the torso frame each
        // control step with the current Htw (the robot moves while the world ball is fixed).
        on<Trigger<LocalisationBall>>().then([this](const LocalisationBall& ball) {
            if (ball.confidence < cfg.ball_confidence_threshold) {
                return;
            }
            ball_rBWw       = ball.rBWw;
            ball_confidence = ball.confidence;
            ball_time       = NUClear::clock::now();
            ball_seen       = true;
        });

        on<Start<KickTask>>().then([this]() {
            if (!model_loaded) {
                log<ERROR>("Kick task started but no kick policy is loaded; staying out of CUSTOM mode");
                return;
            }
            log<INFO>("Kicking (policy)...");
            last_action.fill(0.0f);
            // Reset the gait-phase clock (same init the walk policy uses) and defer the
            // sweep-side choice until the first fresh ball estimate this kick.
            phase        = {0.0, M_PI};
            kick_dir_set = false;
            kick_since   = NUClear::clock::now();
            // Low-level joint commands are only honoured in CUSTOM mode
            auto mode  = std::make_unique<BoosterMode>();
            mode->mode = K1Mode::CUSTOM;
            emit(std::move(mode));
        });

        // 50 Hz inference loop, matching the training control rate (ctrl_dt = 0.02 s)
        on<Provide<KickTask>, Every<50, Per<std::chrono::seconds>>, With<RawSensors>, With<Sensors>, Single>().then(
            [this](const RawSensors& raw, const Sensors& sensors) {
                if (!model_loaded) {
                    emit<Task>(std::make_unique<Continue>());
                    return;
                }

                // Servo feedback in JointIndexK1 order (ankles pre-converted to serial)
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

                // --- observation (v5, 80): gyro(3), gravity(3), kick_dir(2), ball_xy(2),
                //     phase(4), q-default(22), dq(22), last_action(22) ---
                std::array<float, OBS_DIM> obs{};
                std::size_t idx = 0;

                obs[idx++] = raw.gyroscope.x();
                obs[idx++] = raw.gyroscope.y();
                obs[idx++] = raw.gyroscope.z();

                const Eigen::Matrix3d Rwt =
                    rpy_intrinsic_to_mat(Eigen::Vector3d(raw.imu_rpy.x(), raw.imu_rpy.y(), raw.imu_rpy.z()));
                const Eigen::Vector3d grav = Rwt.transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
                obs[idx++] = static_cast<float>(grav.x());
                obs[idx++] = static_cast<float>(grav.y());
                obs[idx++] = static_cast<float>(grav.z());

                // Ball position in the torso frame (xy), matching training's relative_ball_pos.
                // Htw maps a world point into the torso frame; the world ball estimate is
                // treated as absent if never seen or too stale, in which case the policy sees
                // a ball at the origin (as at training reset before the ball is placed).
                const double ball_age =
                    std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now() - ball_time)
                        .count();
                const bool ball_fresh = ball_seen && ball_age < cfg.ball_stale_timeout;
                Eigen::Vector2d ball_xy = Eigen::Vector2d::Zero();
                if (ball_fresh) {
                    ball_xy = (sensors.Htw * ball_rBWw).head<2>();
                }

                // Commanded lateral sweep direction (torso frame, xy). The v5 policy sweeps the
                // ball sideways with the inside foot: sweep it toward the robot's centreline, so
                // a ball on the right (y<0) is swept left (+y) and vice versa. Fixed on the first
                // fresh ball this kick; falls back to the configured default when unseen.
                if (!kick_dir_set) {
                    if (ball_fresh) {
                        const double s = ball_xy.y() >= 0.0 ? -1.0 : 1.0;
                        kick_dir_robot = {0.0, s};
                        kick_dir_set   = true;
                    }
                    else {
                        kick_dir_robot = cfg.kick_direction;
                    }
                }
                obs[idx++] = static_cast<float>(kick_dir_robot[0]);
                obs[idx++] = static_cast<float>(kick_dir_robot[1]);

                obs[idx++] = static_cast<float>(ball_xy.x());
                obs[idx++] = static_cast<float>(ball_xy.y());

                // Gait phase, walk-policy layout: [cos_L, cos_R, sin_L, sin_R]. Advanced each
                // step below (kick advances the clock unconditionally, unlike the walk policy
                // which pins it while standing).
                obs[idx++] = static_cast<float>(std::cos(phase[0]));
                obs[idx++] = static_cast<float>(std::cos(phase[1]));
                obs[idx++] = static_cast<float>(std::sin(phase[0]));
                obs[idx++] = static_cast<float>(std::sin(phase[1]));

                for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                    obs[idx++] = static_cast<float>(servos[j]->present_position - cfg.default_pose[j]);
                }
                for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                    obs[idx++] = servos[j]->present_velocity;
                }
                for (std::size_t k = 0; k < JOINT_COUNT; ++k) {
                    obs[idx++] = last_action[k];
                }

                // --- inference ---
                const std::vector<float> action = onnx_rt->infer(std::vector<float>(obs.begin(), obs.end()));
                std::copy(action.begin(), action.end(), last_action.begin());

                // Advance the gait-phase clock once per inference, wrapped to [-pi, pi), the
                // same 50 Hz step (0.02 s) and wrap the walk policy uses.
                constexpr double TWO_PI = 2.0 * M_PI;
                for (double& p : phase) {
                    p = std::fmod(p + TWO_PI * 0.02 * cfg.gait_frequency + M_PI, TWO_PI) - M_PI;
                }

                // --- action -> low-level joint command: offsets on the DEFAULT pose (the
                // walk/joystick convention the kick task was trained with) ---
                // Cross-fade the commanded pose from the robot's current stance into the
                // policy target over cfg.handoff_blend so the mid-walk robot is not snapped
                // to the default crouch on the first step (training always starts at the
                // default pose; deployment starts from a walk stance).
                const double blend_age =
                    std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now() - kick_since)
                        .count();
                const double alpha = cfg.handoff_blend > 0.0
                                         ? std::min(1.0, blend_age / cfg.handoff_blend)
                                         : 1.0;
                auto low      = std::make_unique<BoosterLowCmd>();
                low->cmd_type = BoosterLowCmd::CmdType::SERIAL;
                low->motor_cmd.resize(JOINT_COUNT);
                for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                    auto& motor         = low->motor_cmd[j];
                    motor.mode          = 1;
                    const double target = cfg.default_pose[j] + cfg.action_scale_joint[j] * last_action[j];
                    motor.q = static_cast<float>((1.0 - alpha) * servos[j]->present_position + alpha * target);
                    motor.dq     = 0.0f;
                    motor.tau    = 0.0f;
                    motor.kp     = static_cast<float>(cfg.kp[j]);
                    motor.kd     = static_cast<float>(cfg.kd[j]);
                    motor.weight = 0.0f;
                }
                // --- completion: run the policy for a fixed window then hand back ---
                const auto now = NUClear::clock::now();
                if (now - kick_since > std::chrono::duration_cast<NUClear::clock::duration>(
                        std::chrono::duration<double>(cfg.kick_duration))) {
                    log<INFO>("Finished kicking (policy)");
                    emit<Task>(std::make_unique<Done>());
                    return;
                }

                // Emit the command as a Director-arbitrated K1Servos subtask (not a raw
                // BoosterLowCmd), so the Director owns the low-level channel: kick subsumes
                // walk, get-up subsumes kick, by task priority. The subtask keeps the Kick
                // task alive, so no Continue is needed.
                auto k1_servos     = std::make_unique<message::actuation::K1Servos>();
                k1_servos->command = *low;
                emit<Task>(std::move(k1_servos));
            });
    }

}  // namespace module::skill
