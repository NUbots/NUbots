#include "K1GetUpPolicy.hpp"

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
#include "message/platform/RawSensors.hpp"
#include "message/skill/GetUp.hpp"

#include "utility/math/euler.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::booster::BoosterLowCmd;
    using message::booster::BoosterMode;
    using message::booster::K1Mode;
    using message::platform::RawSensors;
    using GetUpTask = message::skill::GetUp;

    using utility::math::euler::rpy_intrinsic_to_mat;

    namespace {

        template <std::size_t N>
        std::array<double, N> load_joint_array(const Configuration& config, const char* key) {
            const auto values = config[key].as<std::vector<double>>();
            if (values.size() != N) {
                throw std::runtime_error(std::string("K1GetUpPolicy.yaml: ") + key + " must have "
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

    K1GetUpPolicy::K1GetUpPolicy(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("K1GetUpPolicy.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.model_path          = config["model_path"].as<std::string>();
            cfg.device              = config["device"].as<std::string>();
            cfg.upright_angle       = config["upright_angle"].as<double>();
            cfg.upright_time        = config["upright_time"].as<double>();
            cfg.knee_extended_angle = config["knee_extended_angle"].as<double>();
            cfg.settled_gyro        = config["settled_gyro"].as<double>();

            cfg.kp                 = load_joint_array<JOINT_COUNT>(config, "kp");
            cfg.kd                 = load_joint_array<JOINT_COUNT>(config, "kd");
            cfg.action_scale_joint = load_joint_array<JOINT_COUNT>(config, "action_scale_joint");
            cfg.default_pose       = load_joint_array<JOINT_COUNT>(config, "default_pose");
            const double action_scale = config["action_scale"].as<double>();
            for (double& s : cfg.action_scale_joint) {
                s *= action_scale;
            }

            // GPU first, CPU as the fallback so a machine without a CUDA device (or with a driver
            // mismatch) still runs. fp16 is off, as for the other policies: the actions drive servos
            // directly.
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
                log<INFO>("Loaded get-up policy on", device, cfg.model_path);
            }
            catch (const std::exception& e) {
                onnx_rt.reset();
                log<ERROR>("Failed to load get-up policy", cfg.model_path, e.what());
            }
        });

        on<Start<GetUpTask>>().then([this]() {
            // Never take the robot into CUSTOM with nothing to stream (see K1WalkPolicy)
            if (!model_loaded) {
                log<ERROR>("GetUp task started but no get-up policy is loaded; staying out of CUSTOM mode");
                return;
            }
            log<INFO>("Getting up (policy)...");
            last_action.fill(0.0f);
            was_upright = false;
            // Make sure other policy skills yield the low-level channel until we finish
            emit(std::make_unique<Stability>(Stability::FALLEN));
            auto mode  = std::make_unique<BoosterMode>();
            mode->mode = K1Mode::CUSTOM;
            emit(std::move(mode));
        });

        // 50 Hz inference loop, matching the training control rate (ctrl_dt = 0.02 s)
        on<Provide<GetUpTask>, Every<50, Per<std::chrono::seconds>>, With<RawSensors>, Single>().then(
            [this](const RawSensors& raw) {
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

                // --- observation: gyro(3), gravity(3), q-default(22), dq(22), last_action(22) ---
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

                // --- action -> low-level joint command: offsets on the CURRENT pose ---
                auto low      = std::make_unique<BoosterLowCmd>();
                low->cmd_type = BoosterLowCmd::CmdType::SERIAL;
                low->motor_cmd.resize(JOINT_COUNT);
                for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                    auto& motor  = low->motor_cmd[j];
                    motor.mode   = 1;
                    motor.q      = static_cast<float>(servos[j]->present_position
                                                 + cfg.action_scale_joint[j] * last_action[j]);
                    motor.dq     = 0.0f;
                    motor.tau    = 0.0f;
                    motor.kp     = static_cast<float>(cfg.kp[j]);
                    motor.kd     = static_cast<float>(cfg.kd[j]);
                    motor.weight = 0.0f;
                }
                // --- completion: upright attitude AND knees near extension, sustained ---
                // Attitude alone fires mid-rise (torso is vertical while still deep in a
                // crouch) and hands the robot to the walk policy before it can balance;
                // there is no torso-height sensor on this side, so knee angle is the
                // "actually standing" proxy (stand ~0.4 rad, mid-rise > 1 rad).
                const bool knees_extended =
                    raw.servo.l_knee.present_position < cfg.knee_extended_angle
                    && raw.servo.r_knee.present_position < cfg.knee_extended_angle;
                // Posture alone also fires while the robot is still oscillating from a
                // dynamic rise; require the gyro to be quiet too so the walk policy takes
                // over a settled robot, not a moving one.
                const bool settled = raw.gyroscope.norm() < cfg.settled_gyro;
                const bool upright = knees_extended && settled
                                     && std::abs(raw.imu_rpy.x()) < cfg.upright_angle
                                     && std::abs(raw.imu_rpy.y()) < cfg.upright_angle;
                const auto now = NUClear::clock::now();
                if (upright && !was_upright) {
                    upright_since = now;
                }
                was_upright = upright;

                if (upright
                    && now - upright_since > std::chrono::duration_cast<NUClear::clock::duration>(
                           std::chrono::duration<double>(cfg.upright_time))) {
                    log<INFO>("Finished getting up (policy)");
                    emit(std::make_unique<Stability>(Stability::STANDING));
                    emit<Task>(std::make_unique<Done>());
                    return;
                }

                // Emit the command as a Director-arbitrated K1Servos subtask (not a raw
                // BoosterLowCmd), so get-up owns the low-level channel by priority while it
                // runs. The subtask keeps the GetUp task alive, so no Continue is needed.
                auto k1_servos     = std::make_unique<message::actuation::K1Servos>();
                k1_servos->command = *low;
                emit<Task>(std::move(k1_servos));
            });
    }

}  // namespace module::skill
