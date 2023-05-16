#include "StrafeEvaluator.hpp"

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "message/motion/WalkCommand.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::support::optimisation {
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::WalkCommand;
    using message::platform::RawSensors;
    using message::support::optimisation::NSGA2EvaluationRequest;
    using message::support::optimisation::NSGA2FitnessScores;
    using message::support::optimisation::NSGA2TrialExpired;

    using utility::behaviour::RegisterAction;
    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::support::Expression;

    void StrafeEvaluator::process_raw_sensor_msg(const RawSensors& sensors, NSGA2Evaluator* evaluator) {
        update_max_field_plane_sway(sensors);
        if (check_for_fall(sensors)) {
            evaluator->emit(std::make_unique<NSGA2Evaluator::Event>(NSGA2Evaluator::Event::TERMINATE_EARLY));
        }
    }

    void StrafeEvaluator::process_optimisation_robot_position(const OptimisationRobotPosition& position) {
        if (!initial_position_set) {
            initial_position_set   = true;
            initial_robot_position = position.value;
        }

        robot_position = position.value;
    }

    void StrafeEvaluator::set_up_trial(const NSGA2EvaluationRequest& current_request) {
        // Set our generation and individual identifiers from the request

        trial_duration_limit = std::chrono::seconds(current_request.trial_duration_limit);

        // Set our walk command
        walk_command_velocity.x() = 0.0;
        walk_command_velocity.y() = current_request.parameters.real_params[11];
        walk_command_rotation     = 0.0;

        // Read the QuinticWalk config and overwrite the config parameters with the current individual's
        // parameters
        YAML::Node walk_config = YAML::LoadFile(current_request.task_config_path);

        // The mapping of parameters depends on how the config file was read by the optimiser
        auto walk                    = walk_config["walk"];
        walk["freq"]                 = current_request.parameters.real_params[0];
        walk["double_support_ratio"] = current_request.parameters.real_params[1];

        auto foot        = walk["foot"];
        foot["distance"] = current_request.parameters.real_params[2];
        foot["rise"]     = current_request.parameters.real_params[3];

        auto trunk        = walk["trunk"];
        trunk["height"]   = current_request.parameters.real_params[4];
        trunk["pitch"]    = current_request.parameters.real_params[5];
        trunk["x_offset"] = current_request.parameters.real_params[6];
        trunk["y_offset"] = current_request.parameters.real_params[7];

        trunk["swing"] = current_request.parameters.real_params[8];
        trunk["pause"] = current_request.parameters.real_params[9];

        auto pause        = walk["pause"];
        pause["duration"] = current_request.parameters.real_params[10];

        // Write the updated config to disk
        std::ofstream overwrite_file_stream(current_request.task_config_path);
        overwrite_file_stream << YAML::Dump(walk_config);
        overwrite_file_stream.close();

        // Write the config to keep for later
        NUClear::log<NUClear::DEBUG>(fmt::format("Saving as: gen{:03d}_ind{:03d}_task-{}.yaml",
                                                 current_request.generation,
                                                 current_request.id,
                                                 current_request.task));
        std::ofstream save_file_stream(fmt::format("gen{:03d}_ind{:03d}_task-{}.yaml",
                                                   current_request.generation,
                                                   current_request.id,
                                                   current_request.task));
        save_file_stream << YAML::Dump(walk_config);
        save_file_stream.close();

        // Get constant variables
        YAML::Node eval_config = YAML::LoadFile("config/NSGA2Evaluator.yaml");

        gravity_max = eval_config["gravity"]["MAX"].as<float>();
        gravity_min = eval_config["gravity"]["MIN"].as<float>();
    }

    void StrafeEvaluator::reset_simulation() {
        // Reset our local stateconst OptimisationRobotPosition& position
        trial_start_time       = 0.0;
        robot_position         = Eigen::Vector3d::Zero();
        initial_robot_position = Eigen::Vector3d::Zero();
        max_field_plane_sway   = 0.0;
    }

    void StrafeEvaluator::evaluating_state(size_t subsumption_id, NSGA2Evaluator* evaluator) {
        NUClear::log<NUClear::DEBUG>(fmt::format("Trialling with walk command: ({}, {}) {}",
                                                 walk_command_velocity.x(),
                                                 walk_command_velocity.y(),
                                                 walk_command_rotation));

        evaluator->emit(std::make_unique<WalkCommand>(
            subsumption_id,
            Eigen::Vector3d(walk_command_velocity.x(), walk_command_velocity.y(), walk_command_rotation)));
        evaluator->schedule_trial_expired_message(0, trial_duration_limit);
    }

    std::unique_ptr<NSGA2FitnessScores> StrafeEvaluator::calculate_fitness_scores(bool early_termination,
                                                                                  double sim_time,
                                                                                  int generation,
                                                                                  int individual) {
        auto scores      = calculate_scores();
        auto constraints = early_termination ? calculate_constraints(sim_time) : constraints_not_violated();

        double trial_duration = sim_time - trial_start_time;
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
    // I want a pattern here passed as an arg
    std::vector<double> StrafeEvaluator::calculate_scores() {
        auto robot_distance_travelled = std::fabs(initial_robot_position.y() - robot_position.y());
        return {
            max_field_plane_sway,           // For now, we want to reduce this
            1.0 / robot_distance_travelled  // 1/x since the NSGA2 optimiser is a minimiser
        };
    }

    std::vector<double> StrafeEvaluator::calculate_constraints(double sim_time) {
        // Convert trial duration limit to ms, add 1 for overhead
        const auto overhead = std::chrono::seconds(1);
        double max_trial_duration =
            (std::chrono::duration_cast<std::chrono::milliseconds>(trial_duration_limit + overhead)).count();
        double trial_duration = sim_time - trial_start_time;
        return {
            trial_duration - max_trial_duration,  // Punish for falling over, based on how long the trial took
                                                  // (more negative is worse)
            0.0                                   // Second constraint unused, fixed to 0
        };
    }

    std::vector<double> StrafeEvaluator::constraints_not_violated() {
        return {
            0,  // Robot didn't fall
            0   // Second constraint unused, fixed to 0
        };
    }


    bool StrafeEvaluator::check_for_fall(const RawSensors& sensors) {
        bool fallen        = false;
        auto accelerometer = sensors.accelerometer;

        if ((std::fabs(accelerometer.x()) > gravity_max || std::fabs(accelerometer.y()) > gravity_max)
            && std::fabs(accelerometer.z()) < gravity_min) {
            NUClear::log<NUClear::DEBUG>("Fallen!");
            NUClear::log<NUClear::DEBUG>("acc at fall (x y z):",
                                         std::fabs(accelerometer.x()),
                                         std::fabs(accelerometer.y()),
                                         std::fabs(accelerometer.z()));
            fallen = true;
        }
        return fallen;
    }

    void StrafeEvaluator::update_max_field_plane_sway(const RawSensors& sensors) {
        auto accelerometer = sensors.accelerometer;

        // Calculate the robot sway along the field plane (left/right, forward/backward)
        double field_plane_sway = std::pow(std::pow(accelerometer.x(), 2) + std::pow(accelerometer.y(), 2), 0.5);
        if (field_plane_sway > max_field_plane_sway) {
            max_field_plane_sway = field_plane_sway;
        }
    }

}  // namespace module::support::optimisation