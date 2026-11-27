/*
 * MIT License
 *
 * Copyright (c) 2009 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fmt/format.h>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "Walk.hpp"
#include "ball_skill.hpp"
#include "ball_skill_policy.hpp"
#include "k1_joints.hpp"

#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/booster/BoosterMode.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/skill/BallSkill.hpp"
#include "message/vision/Ball.hpp"

#include "utility/math/euler.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using BallSkillTask    = message::skill::BallSkill;
    using LocalisationBall = message::localisation::Ball;
    using VisionBalls      = message::vision::Balls;

    using utility::math::euler::rpy_intrinsic_to_mat;

    namespace {

        /// The skills' names, in BallSkill::Type order, as BallSkills.yaml's models keys
        constexpr std::array<const char*, ball_skill::KIND_COUNT> KIND_NAMES =
            {"receive", "dribble", "kick", "kick_air", "first_touch", "first_pass"};

        /// The training control period
        constexpr double CONTROL_DT = 0.02;

        constexpr double DEG = M_PI / 180.0;

        template <std::size_t N>
        std::array<double, N> load_array(const Configuration& config, const char* key) {
            const auto values = config[key].as<std::vector<double>>();
            if (values.size() != N) {
                throw std::runtime_error(std::string("BallSkills.yaml: ") + key + " must have " + std::to_string(N)
                                         + " entries (JointIndexK1 order), got " + std::to_string(values.size()));
            }
            std::array<double, N> out{};
            std::copy(values.begin(), values.end(), out.begin());
            return out;
        }

        NUClear::clock::duration seconds(double s) {
            return std::chrono::duration_cast<NUClear::clock::duration>(std::chrono::duration<double>(s));
        }

    }  // namespace

    void Walk::setup_ball_skills() {

        on<Configuration, Sync<Walk>>("BallSkills.yaml").then([this](const Configuration& config) {
            for (std::size_t k = 0; k < ball_skill::KIND_COUNT; ++k) {
                ball_cfg.model_paths[k] = config["models"][KIND_NAMES[k]].as<std::string>();
            }
            ball_cfg.device      = config["device"].as<std::string>();
            ball_cfg.fallback_kp = load_array<JOINT_COUNT>(config, "fallback_kp");
            ball_cfg.fallback_kd = load_array<JOINT_COUNT>(config, "fallback_kd");

            const auto source = config["ball"]["source"].as<std::string>();
            if (source == "vision") {
                ball_cfg.source = BallConfig::Source::VISION;
            }
            else if (source == "localisation") {
                ball_cfg.source = BallConfig::Source::LOCALISATION;
            }
            else {
                throw std::runtime_error("BallSkills.yaml: ball.source must be vision or localisation, got " + source);
            }
            ball_cfg.ball_radius = config["ball"]["radius"].as<double>();
            ball_cfg.max_age     = seconds(config["ball"]["max_age"].as<double>());
            ball_cfg.predict     = config["ball"]["predict"].as<bool>();

            const auto camera = config["camera"];
            const auto offset = camera["offset"].as<std::vector<double>>();
            if (offset.size() != 3) {
                throw std::runtime_error("BallSkills.yaml: camera.offset must be [x, y, z]");
            }
            ball_cfg.camera.offset = Eigen::Vector3d(offset[0], offset[1], offset[2]);
            ball_cfg.camera.hfov   = camera["hfov_deg"].as<double>() * DEG;
            ball_cfg.camera.vfov   = camera["vfov_deg"].as<double>() * DEG;
            ball_cfg.camera.range  = camera["range"].as<double>();

            const auto limits               = config["limits"];
            ball_cfg.limits.dribble_min     = limits["dribble"][0].as<double>();
            ball_cfg.limits.dribble_max     = limits["dribble"][1].as<double>();
            ball_cfg.limits.kick_min        = limits["kick"][0].as<double>();
            ball_cfg.limits.kick_max        = limits["kick"][1].as<double>();
            ball_cfg.limits.kick_angle      = limits["kick_angle_deg"].as<double>() * DEG;
            ball_cfg.limits.shot_speed      = limits["shot_speed"].as<double>();
            ball_cfg.limits.shot_angle      = limits["shot_angle_deg"].as<double>() * DEG;
            ball_cfg.limits.first_touch_min = limits["first_touch"][0].as<double>();
            ball_cfg.limits.first_touch_max = limits["first_touch"][1].as<double>();
            ball_cfg.limits.first_pass_min  = limits["first_pass"][0].as<double>();
            ball_cfg.limits.first_pass_max  = limits["first_pass"][1].as<double>();
            ball_cfg.head_rate_limit        = config["head_rate_limit"].as<double>();
            ball_cfg.handover_window        = seconds(config["handover_window"].as<double>());

            load_ball_skills();
        });

        // The camera's mounting on the head, kept the same as Sensors' so the ball lands in the same place
        on<Configuration, Sync<Walk>>("Sensors.yaml").then([this](const Configuration& config) {
            const auto Hpc      = config["Hpc"];
            ball_cfg.camera.Hpc = Eigen::Isometry3d::Identity();
            ball_cfg.camera.Hpc.translation() << Hpc["translation"][0].as<double>(), Hpc["translation"][1].as<double>(),
                Hpc["translation"][2].as<double>();
            ball_cfg.camera.Hpc.linear() = rpy_intrinsic_to_mat(Eigen::Vector3d(Hpc["rotation_rpy"][0].as<double>(),
                                                                                Hpc["rotation_rpy"][1].as<double>(),
                                                                                Hpc["rotation_rpy"][2].as<double>()));
        });

        // Vision: the detection nearest the camera, put on the ball centre in the world
        on<Trigger<VisionBalls>, Sync<Walk>>().then([this](const VisionBalls& balls) {
            if (ball_cfg.source != BallConfig::Source::VISION) {
                return;
            }
            const Eigen::Isometry3d Hcw(balls.Hcw);
            const Eigen::Vector3d rCWw = Hcw.inverse().translation();
            std::optional<Eigen::Vector3d> nearest{};
            for (const auto& ball : balls.balls) {
                if (ball.is_invalid) {
                    continue;
                }
                const auto rBWw = ball_skill::ray_to_ball_centre(ball.uBCc, Hcw, ball_cfg.ball_radius);
                if (rBWw && (!nearest || (*rBWw - rCWw).norm() < (*nearest - rCWw).norm())) {
                    nearest = rBWw;
                }
            }
            if (nearest) {
                ball_rBWw = nearest;
                ball_vBw  = Eigen::Vector3d::Zero();
                ball_time = balls.timestamp;
            }
        });

        // Localisation (or NUSim's ground truth, which emits the same message): only the robot's own sightings
        on<Trigger<LocalisationBall>, Sync<Walk>>().then([this](const LocalisationBall& ball) {
            if (ball_cfg.source != BallConfig::Source::LOCALISATION || ball.confidence <= 0.0) {
                return;
            }
            ball_rBWw = Eigen::Vector3d(ball.rBWw);
            ball_vBw  = Eigen::Vector3d(ball.vBw);
            ball_time = ball.time_of_measurement;
        });

        on<Start<BallSkillTask>, Sync<Walk>>().then([this] {
            const auto loaded =
                std::find_if(ball_policies.begin(), ball_policies.end(), [](const auto& p) { return p != nullptr; });
            if (loaded == ball_policies.end()) {
                log<ERROR>("Ball-skill task started but no ball-skill policy is loaded; staying out of CUSTOM mode");
                return;
            }
            const ball_skill::Policy& policy = **loaded;

            ball_history.reset();
            ball_since = NUClear::clock::now();
            ball_tick  = 0;
            ball_kind  = -1;

            // Straight on from the walk: no blend, and the previous action is the target the walk last sent, in the
            // ball skills' convention (mjlab CONTROLLER_INTERFACE.md section 4). Every ball skill shares one
            // default pose and action scale.
            ball_blend = !handing_over_from(TargetSource::WALK);
            for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                ball_last_action[j] =
                    ball_blend ? 0.0f
                               : ball_skill::to_action(last_target[j], policy.default_pose[j], policy.action_scale[j]);
            }
            log<INFO>(ball_blend ? "Ball skill starting, blending in from the measured pose"
                                 : "Ball skill taking over from the walk");
            request_custom_mode();
        });

        on<Stop<BallSkillTask>, Sync<Walk>>().then([this] {
            if (ball_kind >= 0) {
                log<INFO>("Ball skill", KIND_NAMES[ball_kind], "stopped");
            }
            ball_kind = -1;
        });

        on<Provide<BallSkillTask>,
           Every<50, Per<std::chrono::seconds>>,
           With<Sensors>,
           With<Stability>,
           Single,
           Sync<Walk>,
           Priority::HIGH>()
            .then([this](const BallSkillTask& task,
                         const RunReason& run_reason,
                         const Sensors& sensors,
                         const Stability& stability) {
                // Step only on the timer, as the walk does, to keep the trained 50 Hz period
                if (run_reason != RunReason::OTHER_TRIGGER) {
                    emit<Task>(std::make_unique<Continue>());
                    return;
                }
                const int kind = static_cast<int>(task.type);
                if (kind < 0 || kind >= static_cast<int>(ball_skill::KIND_COUNT) || ball_policies[kind] == nullptr) {
                    log<WARN>("No",
                              kind >= 0 && kind < int(ball_skill::KIND_COUNT) ? KIND_NAMES[kind] : "such",
                              "ball-skill policy is loaded");
                    return;
                }
                if (sensors.servo.size() < utility::input::ServoID::NUMBER_OF_SERVOS) {
                    log<WARN>("Sensors has", sensors.servo.size(), "servos");
                    return;
                }
                // The get-up skill owns the servos while fallen, and the history must not span a fall
                if (stability == Stability::FALLEN) {
                    ball_history.reset();
                    ball_last_action.fill(0.0f);
                    ball_blend = true;
                    ball_since = NUClear::clock::now();
                    return;
                }
                if (last_mode != int(message::booster::K1Mode::CUSTOM)
                    && NUClear::clock::now() - last_mode_request > std::chrono::seconds(1)) {
                    request_custom_mode();
                }

                ball_skill::Policy& policy = *ball_policies[kind];
                if (kind != ball_kind) {
                    log<INFO>("Ball skill", ball_kind >= 0 ? KIND_NAMES[ball_kind] : "(none)", "->", KIND_NAMES[kind]);
                    ball_kind = kind;
                }

                const Eigen::Vector2d command =
                    ball_skill::clamp_command(ball_skill::Kind(kind), Eigen::Vector2d(task.velocity), ball_cfg.limits);
                const auto servos = k1::servos_of(sensors);

                ball_skill::Frame frame{};
                const Eigen::Vector3d gravity = sensors.Htw.linear() * Eigen::Vector3d(0.0, 0.0, -1.0);
                for (int k = 0; k < 3; ++k) {
                    frame.gyro[k]    = static_cast<float>(sensors.gyroscope[k]);
                    frame.gravity[k] = static_cast<float>(gravity[k]);
                }
                for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                    frame.q[j]  = static_cast<float>(servos[j]->present_position - policy.default_pose[j]);
                    frame.dq[j] = static_cast<float>(servos[j]->present_velocity);
                }
                frame.action  = ball_last_action;
                frame.command = {static_cast<float>(command.x()), static_cast<float>(command.y())};
                frame.ball    = ball_observation(sensors);
                ball_history.push(frame);

                ball_last_action = policy.infer(ball_history.input());

                const auto now = NUClear::clock::now();
                std::array<double, JOINT_COUNT> target{};
                for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                    target[j] = policy.default_pose[j] + policy.action_scale[j] * ball_last_action[j];
                }
                // The policy drives the head, within the head motor's speed
                const bool continuing =
                    handing_over_from(TargetSource::BALL_SKILL) || handing_over_from(TargetSource::WALK);
                if (continuing && ball_cfg.head_rate_limit > 0.0) {
                    const double step = ball_cfg.head_rate_limit * CONTROL_DT;
                    for (const std::size_t j : {HEAD_YAW, HEAD_PITCH}) {
                        target[j] = ball_skill::rate_limit(last_target[j], target[j], step);
                    }
                }
                // Blend in from the measured pose when not continuing from the walk
                const double age = std::chrono::duration<double>(now - ball_since).count();
                const double alpha =
                    !ball_blend || cfg.handoff_blend <= 0.0 ? 1.0 : std::min(1.0, age / cfg.handoff_blend);
                std::array<double, JOINT_COUNT> q{};
                for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                    const double blended = (1.0 - alpha) * servos[j]->present_position + alpha * target[j];
                    q[j]                 = std::clamp(blended, cfg.joint_lower[j], cfg.joint_upper[j]);
                }
                send_targets(q, policy.kp, policy.kd, TargetSource::BALL_SKILL);

                if (log_level <= DEBUG && ball_tick % 50 == 0) {
                    log<DEBUG>(fmt::format("{} cmd=({:.2f}, {:.2f}) ball=({:.2f}, {:.2f}, {:.2f}) seen={}",
                                           KIND_NAMES[kind],
                                           command.x(),
                                           command.y(),
                                           frame.ball[0],
                                           frame.ball[1],
                                           frame.ball[2],
                                           int(frame.ball[3])));
                }
                if (log_level <= TRACE) {
                    std::ostringstream line;
                    line << "BALLOBS " << ball_tick << ' ' << KIND_NAMES[kind]
                         << " t=" << std::chrono::duration<double>(now.time_since_epoch()).count();
                    for (const float v : ball_history.input()) {
                        line << ' ' << v;
                    }
                    log<TRACE>(line.str());
                }
                ++ball_tick;
            });
    }

    std::array<float, ball_skill::BALL_DIM> Walk::ball_observation(const Sensors& sensors) {
        // Out of view the last sighting is repeated with the flag down, as in training
        std::array<float, ball_skill::BALL_DIM> obs = last_ball_seen;
        obs[3]                                      = 0.0f;
        if (!ball_rBWw) {
            return obs;
        }
        const auto age = NUClear::clock::now() - ball_time;
        if (age > ball_cfg.max_age) {
            return obs;
        }
        Eigen::Vector3d rBWw = *ball_rBWw;
        if (ball_cfg.source == BallConfig::Source::LOCALISATION && ball_cfg.predict) {
            rBWw += ball_vBw * std::max(0.0, std::chrono::duration<double>(age).count());
        }
        const Eigen::Vector3d r = ball_skill::to_policy_camera(rBWw, Eigen::Isometry3d(sensors.Hcw), ball_cfg.camera);
        if (!ball_skill::in_view(r, ball_cfg.camera)) {
            return obs;
        }
        last_ball_seen = {static_cast<float>(r.x()), static_cast<float>(r.y()), static_cast<float>(r.z()), 1.0f};
        return last_ball_seen;
    }

    void Walk::load_ball_skills() {
        for (std::size_t k = 0; k < ball_skill::KIND_COUNT; ++k) {
            ball_policies[k].reset();
            const std::string& path = ball_cfg.model_paths[k];
            if (path.empty()) {
                continue;
            }
            try {
                ball_policies[k] = std::make_unique<ball_skill::Policy>(path,
                                                                        ball_cfg.device,
                                                                        ball_cfg.fallback_kp,
                                                                        ball_cfg.fallback_kd);
                if (!ball_policies[k]->gains_from_file) {
                    log<WARN>("Ball-skill policy", path, "has placeholder gains in its metadata; using fallback_kp/kd");
                }
                log<INFO>("Loaded", KIND_NAMES[k], "policy on", ball_cfg.device, path);
            }
            catch (const std::exception& e) {
                ball_policies[k].reset();
                log<ERROR>("Failed to load the", KIND_NAMES[k], "policy", path, e.what());
            }
        }
    }

}  // namespace module::skill
