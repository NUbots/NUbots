/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#include "WalkDataCollector.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <random>

#include "extension/Configuration.hpp"

#include "message/actuation/KinematicsModel.hpp"
#include "message/actuation/Limbs.hpp"
#include "message/actuation/ServoCommand.hpp"

#include "utility/actuation/InverseKinematics.hpp"
#include "utility/actuation/tinyrobotics.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::tools {

    using extension::Configuration;

    using message::actuation::KinematicsModel;
    using message::actuation::LeftLeg;
    using message::actuation::RightLeg;
    using message::actuation::ServoCommand;

    using utility::actuation::kinematics::calculate_leg_joints;
    using utility::actuation::tinyrobotics::configuration_to_servos;
    using utility::actuation::tinyrobotics::servos_to_configuration;
    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::support::Expression;

    using WalkState = message::behaviour::state::WalkState;

    tinyrobotics::InverseKinematicsMethod WalkDataCollector::ik_string_to_method(const std::string& method_string) {
        static std::map<std::string, tinyrobotics::InverseKinematicsMethod> string_to_method_map = {
            {"JACOBIAN", tinyrobotics::InverseKinematicsMethod::JACOBIAN},
            {"NLOPT", tinyrobotics::InverseKinematicsMethod::NLOPT},
            {"LEVENBERG_MARQUARDT", tinyrobotics::InverseKinematicsMethod::LEVENBERG_MARQUARDT},
            {"PARTICLE_SWARM", tinyrobotics::InverseKinematicsMethod::PARTICLE_SWARM},
            {"BFGS", tinyrobotics::InverseKinematicsMethod::BFGS}};

        auto it = string_to_method_map.find(method_string);
        if (it == string_to_method_map.end()) {
            throw std::invalid_argument("Unrecognized IK method string: " + method_string);
        }
        return it->second;
    }

    std::array<double, WalkDataCollector::n_leg_joints> WalkDataCollector::compute_leg_ik(
        const Eigen::Isometry3d& Htf,
        const LimbID& limb) {

        // Step 1: Analytical IK (same as Kinematics.cpp line 102)
        auto joints = calculate_leg_joints<double>(kinematics_model, Htf, limb);

        // Step 2: Pack analytical solution into a servo command structure for tinyrobotics
        // We need to create a temporary Leg message to use the conversion utilities
        if (limb == LimbID::LEFT_LEG) {
            auto servos = std::make_unique<LeftLeg>();
            for (const auto& joint : joints) {
                servos->servos[joint.first] = ServoCommand(NUClear::clock::now(), joint.second, {});
            }

            // Convert to tinyrobotics configuration vector (warm start)
            auto q0 = servos_to_configuration<LeftLeg, double, n_joints>(servos.get());

            // Step 3: Numerical IK refinement (same as Kinematics.cpp line 113)
            auto q_sol = tinyrobotics::inverse_kinematics(nugus_model_left,
                                                          cfg.left_foot_name,
                                                          cfg.torso_name,
                                                          Htf,
                                                          q0,
                                                          ik_options);

            // Step 4: Convert back to servo commands to extract joint angles
            configuration_to_servos(servos.get(), q_sol);

            // Extract the 6 left leg joint angles in our canonical ordering
            return {servos->servos.at(ServoID::L_HIP_YAW).position,
                    servos->servos.at(ServoID::L_HIP_ROLL).position,
                    servos->servos.at(ServoID::L_HIP_PITCH).position,
                    servos->servos.at(ServoID::L_KNEE).position,
                    servos->servos.at(ServoID::L_ANKLE_PITCH).position,
                    servos->servos.at(ServoID::L_ANKLE_ROLL).position};
        }
        else {
            auto servos = std::make_unique<RightLeg>();
            for (const auto& joint : joints) {
                servos->servos[joint.first] = ServoCommand(NUClear::clock::now(), joint.second, {});
            }

            auto q0 = servos_to_configuration<RightLeg, double, n_joints>(servos.get());

            auto q_sol = tinyrobotics::inverse_kinematics(nugus_model_right,
                                                          cfg.right_foot_name,
                                                          cfg.torso_name,
                                                          Htf,
                                                          q0,
                                                          ik_options);

            configuration_to_servos(servos.get(), q_sol);

            return {servos->servos.at(ServoID::R_HIP_YAW).position,
                    servos->servos.at(ServoID::R_HIP_ROLL).position,
                    servos->servos.at(ServoID::R_HIP_PITCH).position,
                    servos->servos.at(ServoID::R_KNEE).position,
                    servos->servos.at(ServoID::R_ANKLE_PITCH).position,
                    servos->servos.at(ServoID::R_ANKLE_ROLL).position};
        }
    }

    Eigen::Vector3d WalkDataCollector::sample_velocity(std::mt19937& rng) {
        std::uniform_real_distribution<double> vx_dist(cfg.vx_range[0], cfg.vx_range[1]);
        std::uniform_real_distribution<double> vy_dist(cfg.vy_range[0], cfg.vy_range[1]);
        std::uniform_real_distribution<double> vtheta_dist(cfg.vtheta_range[0], cfg.vtheta_range[1]);

        // 15% chance of standing still
        std::uniform_real_distribution<double> prob(0.0, 1.0);
        if (prob(rng) < 0.15) {
            return Eigen::Vector3d::Zero();
        }

        return Eigen::Vector3d(vx_dist(rng), vy_dist(rng), vtheta_dist(rng));
    }

    void WalkDataCollector::collect_data() {
        // Create output directory
        std::filesystem::create_directories(cfg.output_directory);

        const double dt = 1.0 / cfg.update_frequency;

        std::mt19937 rng(cfg.seed);
        std::uniform_real_distribution<double> interval_dist(cfg.velocity_change_interval[0],
                                                             cfg.velocity_change_interval[1]);

        log<NUClear::LogLevel::INFO>("Starting data collection:");
        log<NUClear::LogLevel::INFO>("  Episodes:", cfg.num_episodes);
        log<NUClear::LogLevel::INFO>("  Episode length:", cfg.episode_length, "timesteps",
                                     fmt::format("({:.1f}s)", cfg.episode_length * dt));
        log<NUClear::LogLevel::INFO>("  Output directory:", cfg.output_directory);
        log<NUClear::LogLevel::INFO>("  Observation dim:", obs_dim);
        log<NUClear::LogLevel::INFO>("  Target dim:", target_dim);
        log<NUClear::LogLevel::INFO>("  Total floats per sample:", sample_dim);

        size_t total_samples = 0;

        for (int episode = 0; episode < cfg.num_episodes; episode++) {
            if (episode % 1000 == 0) {
                log<NUClear::LogLevel::INFO>("Episode", episode, "/", cfg.num_episodes);
            }

            // Reset the walk engine for each episode
            walk_generator.set_parameters(cfg.walk_params);
            walk_generator.reset();

            // Initialise action history with zeros (standing pose will be computed on first step)
            std::array<std::array<double, n_total_leg_joints>, n_history_frames> action_history{};

            // Compute the standing pose to initialise history
            Eigen::Isometry3d Htl_init = walk_generator.get_foot_pose(LimbID::LEFT_LEG);
            Eigen::Isometry3d Htr_init = walk_generator.get_foot_pose(LimbID::RIGHT_LEG);
            auto left_init             = compute_leg_ik(Htl_init, LimbID::LEFT_LEG);
            auto right_init            = compute_leg_ik(Htr_init, LimbID::RIGHT_LEG);

            // Fill all history frames with the standing pose
            for (auto& hist : action_history) {
                for (int j = 0; j < n_leg_joints; j++) {
                    hist[j]                = left_init[j];
                    hist[j + n_leg_joints] = right_init[j];
                }
            }

            // Sample initial velocity target
            Eigen::Vector3d target_velocity  = sample_velocity(rng);
            Eigen::Vector3d current_velocity = Eigen::Vector3d::Zero();
            double next_change_time          = interval_dist(rng);
            double episode_time              = 0.0;

            // Allocate buffer for this episode's samples
            std::vector<float> episode_data;
            episode_data.reserve(cfg.episode_length * sample_dim);

            for (int t = 0; t < cfg.episode_length; t++) {
                // Change velocity target periodically
                if (episode_time >= next_change_time) {
                    static bool next_is_stop = false;

                    if (next_is_stop) {
                        // Force a complete stop after a short tap so the network learns STARTING -> STOPPING
                        target_velocity = Eigen::Vector3d::Zero();
                        next_change_time = episode_time + interval_dist(rng); // Wait long enough to fully settle
                        next_is_stop = false;
                    } else {
                        target_velocity = sample_velocity(rng);
                        
                        // Use a mix of long walks and very short taps to ensure the network learns
                        // both sustained walking and immediate STARTING -> STOPPING transitions!
                        if (!target_velocity.isZero()) {
                            std::uniform_real_distribution<double> short_tap(0.0, 1.0);
                            if (short_tap(rng) < 0.2) {
                                std::uniform_real_distribution<double> interval_dist_short(0.1, 0.5);
                                next_change_time = episode_time + interval_dist_short(rng);
                                next_is_stop = true; // Force the NEXT change to be a full stop
                            } else {
                                next_change_time = episode_time + interval_dist(rng);
                            }
                        } else {
                            next_change_time = episode_time + interval_dist(rng);
                        }
                    }
                }

                // Smooth velocity with acceleration limits (same as Walk.cpp:219-222)
                Eigen::Vector3d dv = cfg.acceleration * std::min(dt, 1.0);
                current_velocity   = current_velocity
                                   + (target_velocity - current_velocity).cwiseMax(-dv).cwiseMin(dv);

                // Update the walk engine (no physics, so planted foot phase matches internal phase)
                WalkState::State state =
                    walk_generator.update(dt, current_velocity, walk_generator.get_phase());

                // Get the current phase clock values
                double phase_time = walk_generator.get_time();
                double step_period = walk_generator.get_step_period();
                double phase_ratio = (step_period > 0) ? (phase_time / step_period) : 0.0;
                double phase_sin   = std::sin(2.0 * M_PI * phase_ratio);
                double phase_cos   = std::cos(2.0 * M_PI * phase_ratio);

                // Phase indicator: +1 for LEFT planted, -1 for RIGHT planted
                double phase_indicator =
                    (walk_generator.get_phase() == WalkState::Phase::LEFT) ? 1.0 : -1.0;

                // Engine state one-hot encoding
                std::array<float, 4> state_onehot = {0.0f, 0.0f, 0.0f, 0.0f};
                switch (state.value) {
                    case WalkState::State::STOPPED: state_onehot[0] = 1.0f; break;
                    case WalkState::State::STARTING: state_onehot[1] = 1.0f; break;
                    case WalkState::State::WALKING: state_onehot[2] = 1.0f; break;
                    case WalkState::State::STOPPING: state_onehot[3] = 1.0f; break;
                    default: break;
                }

                // Get desired foot poses in torso frame from walk engine
                Eigen::Isometry3d Htl = walk_generator.get_foot_pose(LimbID::LEFT_LEG);
                Eigen::Isometry3d Htr = walk_generator.get_foot_pose(LimbID::RIGHT_LEG);

                // Run the full IK pipeline (analytical + numerical) for both legs
                auto left_joints  = compute_leg_ik(Htl, LimbID::LEFT_LEG);
                auto right_joints = compute_leg_ik(Htr, LimbID::RIGHT_LEG);

                // Build the target vector (12 joint angles)
                std::array<double, n_total_leg_joints> target{};
                for (int j = 0; j < n_leg_joints; j++) {
                    target[j]                = left_joints[j];
                    target[j + n_leg_joints] = right_joints[j];
                }

                // ==================== Build observation vector ====================
                // [velocity_cmd(3), phase_clock(2), phase_indicator(1), engine_state(4),
                //  prev_joints_1(12), prev_joints_2(12), prev_joints_3(12)]

                // Velocity command (3)
                episode_data.push_back(static_cast<float>(current_velocity.x()));
                episode_data.push_back(static_cast<float>(current_velocity.y()));
                episode_data.push_back(static_cast<float>(current_velocity.z()));

                // Phase clock (2)
                episode_data.push_back(static_cast<float>(phase_sin));
                episode_data.push_back(static_cast<float>(phase_cos));

                // Phase indicator (1)
                episode_data.push_back(static_cast<float>(phase_indicator));

                // Engine state one-hot (4)
                for (int j = 0; j < 4; j++) {
                    episode_data.push_back(state_onehot[j]);
                }

                // Action history: most recent first (t-1, t-2, t-3)
                for (int h = 0; h < n_history_frames; h++) {
                    for (int j = 0; j < n_total_leg_joints; j++) {
                        episode_data.push_back(static_cast<float>(action_history[h][j]));
                    }
                }

                // ==================== Target vector ====================
                for (int j = 0; j < n_total_leg_joints; j++) {
                    episode_data.push_back(static_cast<float>(target[j]));
                }

                // Update action history (shift: 2→3, 1→2, current→1)
                action_history[2] = action_history[1];
                action_history[1] = action_history[0];
                action_history[0] = target;

                episode_time += dt;
            }

            // Write episode to binary file
            std::string filename =
                cfg.output_directory + "/episode_" + fmt::format("{:05d}", episode) + ".bin";
            std::ofstream out(filename, std::ios::binary);
            if (!out.is_open()) {
                log<NUClear::LogLevel::ERROR>("Failed to open file for writing: {}", filename);
                continue;
            }
            out.write(reinterpret_cast<const char*>(episode_data.data()),
                      static_cast<std::streamsize>(episode_data.size() * sizeof(float)));
            out.close();

            total_samples += cfg.episode_length;
        }

        log<NUClear::LogLevel::INFO>("Data collection complete!");
        log<NUClear::LogLevel::INFO>("  Total samples: {}", total_samples);
        log<NUClear::LogLevel::INFO>("  Total files: {}", cfg.num_episodes);
        log<NUClear::LogLevel::INFO>("  Output directory: {}", cfg.output_directory);

        // Write a metadata file for the Python training pipeline
        std::string meta_filename = cfg.output_directory + "/metadata.yaml";
        std::ofstream meta(meta_filename);
        meta << "# Auto-generated metadata for walk distillation training data\n";
        meta << "obs_dim: " << obs_dim << "\n";
        meta << "target_dim: " << target_dim << "\n";
        meta << "sample_dim: " << sample_dim << "\n";
        meta << "num_episodes: " << cfg.num_episodes << "\n";
        meta << "episode_length: " << cfg.episode_length << "\n";
        meta << "update_frequency: " << cfg.update_frequency << "\n";
        meta << "dt: " << dt << "\n";
        meta << "step_period: " << cfg.walk_params.step_period << "\n";
        meta << "n_history_frames: " << n_history_frames << "\n";
        meta << "seed: " << cfg.seed << "\n";
        meta << "observation_layout:\n";
        meta << "  - name: velocity_cmd\n";
        meta << "    start: 0\n";
        meta << "    dim: 3\n";
        meta << "  - name: phase_clock\n";
        meta << "    start: 3\n";
        meta << "    dim: 2\n";
        meta << "  - name: phase_indicator\n";
        meta << "    start: 5\n";
        meta << "    dim: 1\n";
        meta << "  - name: engine_state\n";
        meta << "    start: 6\n";
        meta << "    dim: 4\n";
        meta << "  - name: prev_joints_1\n";
        meta << "    start: 10\n";
        meta << "    dim: 12\n";
        meta << "  - name: prev_joints_2\n";
        meta << "    start: 22\n";
        meta << "    dim: 12\n";
        meta << "  - name: prev_joints_3\n";
        meta << "    start: 34\n";
        meta << "    dim: 12\n";
        meta << "target_layout:\n";
        meta << "  joint_order:\n";
        meta << "    - L_HIP_YAW\n";
        meta << "    - L_HIP_ROLL\n";
        meta << "    - L_HIP_PITCH\n";
        meta << "    - L_KNEE\n";
        meta << "    - L_ANKLE_PITCH\n";
        meta << "    - L_ANKLE_ROLL\n";
        meta << "    - R_HIP_YAW\n";
        meta << "    - R_HIP_ROLL\n";
        meta << "    - R_HIP_PITCH\n";
        meta << "    - R_KNEE\n";
        meta << "    - R_ANKLE_PITCH\n";
        meta << "    - R_ANKLE_ROLL\n";
        meta.close();

        log<NUClear::LogLevel::INFO>("Metadata written to: {}", meta_filename);
    }

    WalkDataCollector::WalkDataCollector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("WalkDataCollector.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Output settings
            cfg.output_directory = config["output_directory"].as<std::string>();

            // URDF and IK settings
            cfg.urdf_path         = config["urdf_path"].as<std::string>();
            cfg.ik_tolerance      = config["ik_tolerance"].as<double>();
            cfg.ik_max_iterations = config["ik_max_iterations"].as<int>();
            cfg.ik_method         = config["ik_method"].as<std::string>();
            cfg.torso_name        = config["links"]["torso"].as<std::string>();
            cfg.left_foot_name    = config["links"]["left_foot"].as<std::string>();
            cfg.right_foot_name   = config["links"]["right_foot"].as<std::string>();

            // Parse URDF for tinyrobotics models
            nugus_model_left  = tinyrobotics::import_urdf<double, n_joints>(cfg.urdf_path);
            nugus_model_right = tinyrobotics::import_urdf<double, n_joints>(cfg.urdf_path);

            // Set IK options
            ik_options.tolerance      = cfg.ik_tolerance;
            ik_options.max_iterations = cfg.ik_max_iterations;
            ik_options.method         = ik_string_to_method(cfg.ik_method);

            // Walk engine parameters (same as Walk.yaml)
            cfg.walk_params.step_period     = config["walk"]["period"].as<double>();
            cfg.walk_params.step_apex_ratio = config["walk"]["step"]["apex_ratio"].as<double>();
            cfg.walk_params.step_limits     = config["walk"]["step"]["limits"].as<Expression>();
            cfg.walk_params.step_height     = config["walk"]["step"]["height"].as<double>();
            cfg.walk_params.step_width      = config["walk"]["step"]["width"].as<double>();
            cfg.walk_params.torso_height    = config["walk"]["torso"]["height"].as<double>();
            cfg.walk_params.torso_pitch     = config["walk"]["torso"]["pitch"].as<Expression>();
            cfg.walk_params.torso_position_offset =
                config["walk"]["torso"]["position_offset"].as<Expression>();
            cfg.walk_params.torso_sway_offset =
                config["walk"]["torso"]["sway_offset"].as<Expression>();
            cfg.walk_params.torso_start_sway_offset =
                config["walk"]["torso"]["start_sway_offset"].as<Expression>();
            cfg.walk_params.torso_sway_ratio = config["walk"]["torso"]["sway_ratio"].as<double>();
            cfg.walk_params.torso_final_position_ratio =
                config["walk"]["torso"]["final_position_ratio"].as<Expression>();
            cfg.walk_params.only_switch_when_planted =
                config["walk"]["only_switch_when_planted"].as<bool>();

            cfg.acceleration = config["walk"]["acceleration"].as<Expression>();

            // Collection parameters
            cfg.num_episodes    = config["collection"]["num_episodes"].as<int>();
            cfg.episode_length  = config["collection"]["episode_length"].as<int>();
            cfg.update_frequency = config["collection"]["update_frequency"].as<int>();
            cfg.seed            = config["collection"]["seed"].as<int>();
            cfg.velocity_change_interval =
                config["collection"]["velocity_change_interval"].as<Expression>();

            // Velocity ranges
            cfg.vx_range     = config["velocity_ranges"]["vx"].as<Expression>();
            cfg.vy_range     = config["velocity_ranges"]["vy"].as<Expression>();
            cfg.vtheta_range = config["velocity_ranges"]["vtheta"].as<Expression>();
        });

        // Load the KinematicsModel from the KinematicsConfiguration (needed for analytical IK)
        on<Configuration>("KinematicsConfiguration.yaml").then([this](const Configuration& config) {
            // Parse leg parameters for analytical IK
            const Eigen::Vector3f leg_hip_offset = config["leg"]["hip_offset"].as<Expression>();
            kinematics_model.leg.HIP_OFFSET_X   = leg_hip_offset.x();
            kinematics_model.leg.HIP_OFFSET_Y   = leg_hip_offset.y();
            kinematics_model.leg.HIP_OFFSET_Z   = leg_hip_offset.z();

            kinematics_model.leg.UPPER_LEG_LENGTH = config["leg"]["upper_leg_length"].as<float>();
            kinematics_model.leg.LOWER_LEG_LENGTH = config["leg"]["lower_leg_length"].as<float>();

            const auto& foot             = config["leg"]["foot"];
            kinematics_model.leg.FOOT_HEIGHT = foot["height"].as<float>();

            kinematics_model.leg.LENGTH_BETWEEN_LEGS = 2.0f * kinematics_model.leg.HIP_OFFSET_Y;

            const auto& left_right                       = config["leg"]["left_to_right"];
            kinematics_model.leg.LEFT_TO_RIGHT_HIP_YAW   = left_right["hip_yaw"].as<int>();
            kinematics_model.leg.LEFT_TO_RIGHT_HIP_ROLL  = left_right["hip_roll"].as<int>();
            kinematics_model.leg.LEFT_TO_RIGHT_HIP_PITCH = left_right["hip_pitch"].as<int>();
            kinematics_model.leg.LEFT_TO_RIGHT_KNEE      = left_right["knee"].as<int>();
            kinematics_model.leg.LEFT_TO_RIGHT_ANKLE_PITCH = left_right["ankle_pitch"].as<int>();
            kinematics_model.leg.LEFT_TO_RIGHT_ANKLE_ROLL  = left_right["ankle_roll"].as<int>();

            log<NUClear::LogLevel::INFO>("KinematicsModel loaded: upper_leg=",
                                         kinematics_model.leg.UPPER_LEG_LENGTH,
                                         "lower_leg=",
                                         kinematics_model.leg.LOWER_LEG_LENGTH,
                                         "foot_height=",
                                         kinematics_model.leg.FOOT_HEIGHT);
        });

        // Run data collection once startup is complete
        on<Startup>().then([this] {
            // Small delay to ensure configs are loaded
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            collect_data();

            // Shut down after collection is complete
            powerplant.shutdown();
        });
    }

}  // namespace module::tools
