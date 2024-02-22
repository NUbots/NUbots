/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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
#include "WalkEvaluator.hpp"

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/skill/Walk.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::support::optimisation {
    using message::input::Sensors;
    using message::skill::Walk;
    using message::support::optimisation::NSGA2EvaluationRequest;
    using message::support::optimisation::NSGA2FitnessScores;
    using message::support::optimisation::NSGA2TrialExpired;

    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::support::Expression;

    using extension::Configuration;

    void WalkEvaluator::process_optimisation_robot_position(const OptimisationRobotPosition& position) {
        if (!initial_position_set) {
            initial_position_set   = true;
            initial_robot_position = position.value;
        }

        robot_position = position.value;
    }

    void WalkEvaluator::set_up_trial(const NSGA2EvaluationRequest& current_request) {
        // Set our generation and individual identifiers from the request
        trial_duration_limit = std::chrono::seconds(current_request.trial_duration_limit);

        // Set our walk command
        walk_command_velocity.x() = current_request.parameters.real_params[21];
        walk_command_velocity.y() = 0.0;
        walk_command_rotation     = 0.0;

        // Read the QuinticWalk config and overwrite the config parameters with the current individual's
        // parameters
        YAML::Node walk_config = YAML::LoadFile(current_request.task_config_path);
        NUClear::log<NUClear::INFO>("CurrentConfigPath", current_request.task_config_path);

        // The mapping of parameters depends on how the config file was read by the optimiser
        auto walk      = walk_config["walk"];
        walk["period"] = current_request.parameters.real_params[0];

        auto step          = walk["step"];
        step["limits"][0]  = current_request.parameters.real_params[1];
        step["limits"][1]  = current_request.parameters.real_params[2];
        step["limits"][2]  = current_request.parameters.real_params[3];
        step["height"]     = current_request.parameters.real_params[4];
        step["width"]      = current_request.parameters.real_params[5];
        step["apex_ratio"] = current_request.parameters.real_params[6];

        auto torso      = walk["torso"];
        torso["height"] = current_request.parameters.real_params[7];
        torso["pitch"]  = current_request.parameters.real_params[8];

        torso["position_offset"][0] = current_request.parameters.real_params[9];
        torso["position_offset"][1] = current_request.parameters.real_params[10];
        torso["position_offset"][2] = current_request.parameters.real_params[11];

        torso["sway_offset"][0] = current_request.parameters.real_params[12];
        torso["sway_offset"][1] = current_request.parameters.real_params[13];
        torso["sway_offset"][2] = current_request.parameters.real_params[14];

        torso["sway_ratio"] = current_request.parameters.real_params[15];

        torso["final_position_ratio"][0] = current_request.parameters.real_params[16];
        torso["final_position_ratio"][1] = current_request.parameters.real_params[17];
        torso["final_position_ratio"][2] = current_request.parameters.real_params[18];

        auto arms                    = walk_config["arms"];
        arms["right_shoulder_pitch"] = current_request.parameters.real_params[19];
        arms["left_shoulder_pitch"]  = current_request.parameters.real_params[19];
        arms["right_elbow"]          = current_request.parameters.real_params[20];
        arms["left_elbow"]           = current_request.parameters.real_params[20];

        // Write the updated config to disk
        std::ofstream overwrite_file_stream(current_request.task_config_path);
        overwrite_file_stream << YAML::Dump(walk_config);
        overwrite_file_stream.close();

        // Write the config to keep for later
        NUClear::log<NUClear::DEBUG>(fmt::format("Saving as: gen{:03d}_ind{:03d}_task-walk.yaml",
                                                 current_request.generation,
                                                 current_request.id));
        std::ofstream save_file_stream(
            fmt::format("gen{:03d}_ind{:03d}_task-walk.yaml", current_request.generation, current_request.id));
        save_file_stream << YAML::Dump(walk_config);
        save_file_stream.close();

        // Get constant variables
        YAML::Node config = YAML::LoadFile("config/NSGA2Evaluator.yaml");

        cfg.fallen_angle = config["fallen_angle"].as<float>();

        fallen = false;
    }

    void WalkEvaluator::reset_trial() {
        // Reset our local state
        trial_start_time       = NUClear::clock::now();
        robot_position         = Eigen::Vector3d::Zero();
        initial_robot_position = Eigen::Vector3d::Zero();
        max_field_plane_sway   = 0.0;
    }

    void WalkEvaluator::evaluating_state(NSGA2Evaluator* evaluator) {
        NUClear::log<NUClear::DEBUG>(fmt::format("Trialling with walk command: ({}, {}) {}",
                                                 walk_command_velocity.x(),
                                                 walk_command_velocity.y(),
                                                 walk_command_rotation));

        NUClear::log<NUClear::DEBUG>("Walk started");
        evaluator->walk(Eigen::Vector3d(walk_command_velocity.x(), walk_command_velocity.y(), walk_command_rotation));
        evaluator->schedule_trial_expired_message(0, trial_duration_limit);
    }

    bool WalkEvaluator::has_fallen(const Sensors& sensors) {
        update_max_field_plane_sway(sensors);

        // Transform to torso {t} from world {w} space
        Eigen::Matrix4d Hwt = sensors.Htw.inverse().matrix();
        // Basis Z vector of torso {t} in world {w} space
        Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);

        // Check if angle between torso z axis and world z axis is greater than config value cfg.fallen_angle
        if (!fallen && std::acos(Eigen::Vector3d::UnitZ().dot(uZTw)) > cfg.fallen_angle) {
            NUClear::log<NUClear::DEBUG>("Fallen!");
            fallen = true;
            return true;
        }
        return false;
    }

    void WalkEvaluator::update_max_field_plane_sway(const Sensors& sensors) {
        auto accelerometer = sensors.accelerometer;

        // Calculate the robot sway along the field plane (left/right, forward/backward)
        double field_plane_sway = std::pow(std::pow(accelerometer.x(), 2) + std::pow(accelerometer.y(), 2), 0.5);
        if (field_plane_sway > max_field_plane_sway) {
            max_field_plane_sway = field_plane_sway;
        }
    }

    std::unique_ptr<NSGA2FitnessScores> WalkEvaluator::calculate_fitness_scores(bool early_termination,
                                                                                int generation,
                                                                                int individual) {
        double trial_duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(NUClear::clock::now() - trial_start_time).count();

        auto scores      = calculate_scores();
        auto constraints = early_termination ? calculate_constraints(trial_duration) : constraints_not_violated();

        NUClear::log<NUClear::DEBUG>("Trial ran for", trial_duration);
        NUClear::log<NUClear::DEBUG>("SendFitnessScores for generation", generation, "individual", individual);
        NUClear::log<NUClear::DEBUG>("    scores:", scores[0], scores[1]);
        NUClear::log<NUClear::DEBUG>("    constraints:", constraints[0], constraints[1]);

        // Create the fitness scores message based on the given results and emit it back to the Optimiser
        std::unique_ptr<NSGA2FitnessScores> fitness_scores = std::make_unique<NSGA2FitnessScores>();
        fitness_scores->id                                 = individual;
        fitness_scores->generation                         = generation;
        fitness_scores->obj_score                          = scores;
        fitness_scores->constraints                        = constraints;
        return fitness_scores;
    }

    std::vector<double> WalkEvaluator::calculate_scores() {
        auto robot_distance_travelled = std::fabs(initial_robot_position.x() - robot_position.x());
        NUClear::log<NUClear::DEBUG>("Distance travelled", robot_distance_travelled);
        NUClear::log<NUClear::DEBUG>("Max field plane sway", max_field_plane_sway);
        return {
            max_field_plane_sway,           // Reduce the torso sway
            1.0 / robot_distance_travelled  // 1/x since the NSGA2 optimiser is a minimiser
        };
    }

    std::vector<double> WalkEvaluator::calculate_constraints(double trial_duration) {
        // Convert trial duration limit to ms, add 1 for overhead
        const auto overhead = std::chrono::seconds(1);
        double max_trial_duration =
            (std::chrono::duration_cast<std::chrono::milliseconds>(trial_duration_limit + overhead)).count();

        return {
            trial_duration - max_trial_duration,  // Punish for falling over, based on how long the trial took
                                                  // (more negative is worse)
            0.0                                   // Second constraint unused, fixed to 0
        };
    }

    std::vector<double> WalkEvaluator::constraints_not_violated() {
        return {
            0.0,  // Robot didn't fall
            0.0   // Second constraint unused, fixed to 0
        };
    }

}  // namespace module::support::optimisation
