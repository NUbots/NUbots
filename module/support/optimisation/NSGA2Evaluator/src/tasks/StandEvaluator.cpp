#include "StandEvaluator.hpp"

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

    // void StandEvaluator::process_raw_sensor_msg(const RawSensors& sensors, NSGA2Evaluator* evaluator) {
    //     double sim_time = evaluator->sim_time;
    //     sim_time++;

    //     update_max_field_plane_sway(sensors);
    //     current_sensors = sensors;
    // }

    bool StandEvaluator::has_fallen(const Sensors& sensors) {
        update_max_field_plane_sway(sensors);

        // Transform to torso {t} from world {w} space
        Eigen::Matrix4d Hwt = sensors.Htw.inverse().matrix();
        // Basis Z vector of torso {t} in world {w} space
        Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);

        // Check if angle between torso z axis and world z axis is greater than config value cfg.fallen_angle
        if (std::acos(Eigen::Vector3d::UnitZ().dot(uZTw)) > fallen_angle) {
            NUClear::log<NUClear::DEBUG>("Fallen!");
            return true;
        }
        return false;
    }

    void StandEvaluator::process_optimisation_robot_position(const OptimisationRobotPosition& position) {
        robot_position = position.value;
    }

    void StandEvaluator::set_up_trial(const NSGA2EvaluationRequest& currentRequest) {
        load_script(currentRequest.task_config_path);
        std::chrono::milliseconds limit_ms = std::chrono::milliseconds(0);
        for (size_t i = 0; i < currentRequest.parameters.real_params.size(); i++) {
            int frame_time            = currentRequest.parameters.real_params[i];
            limit_ms                  = limit_ms + std::chrono::milliseconds(frame_time);
            script.frames[i].duration = std::chrono::milliseconds(frame_time);
        }
        save_script(fmt::format("gen{:03d}_ind{:03d}_task-{}.yaml",
                                currentRequest.generation,
                                currentRequest.id,
                                currentRequest.task));

        auto overhead = std::chrono::seconds(2);  // Overhead tacked on to give the robot time to fall over if unstable
        trial_duration_limit = std::chrono::duration_cast<std::chrono::seconds>(limit_ms) + overhead;

        // Get constant variables
        YAML::Node config = YAML::LoadFile("config/NSGA2Evaluator.yaml");

        fallen_angle = config["fallen_angle"].as<float>();
    }

    void StandEvaluator::reset_simulation() {
        // Reset our local state
        trial_start_time     = 0.0;
        robot_position       = Eigen::Vector3d::Zero();
        max_field_plane_sway = 0.0;
    }

    void StandEvaluator::evaluating_state(size_t subsumption_id, NSGA2Evaluator* evaluator) {
        NUClear::log<NUClear::DEBUG>("Running Script");
        run_script(subsumption_id, evaluator);
        NUClear::log<NUClear::DEBUG>("schedule expire");
        evaluator->schedule_trial_expired_message(0, trial_duration_limit);
    }

    std::unique_ptr<NSGA2FitnessScores> StandEvaluator::calculate_fitness_scores(bool early_termination,
                                                                                 double sim_time,
                                                                                 int generation,
                                                                                 int individual) {
        double trial_duration = sim_time - trial_start_time;
        auto scores           = calculate_scores(trial_duration);
        auto constraints      = calculate_constraints(early_termination);

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

    std::vector<double> StandEvaluator::calculate_scores(double trial_duration) {
        return {max_field_plane_sway,  // For now, we want to reduce this
                trial_duration};
    }

    std::vector<double> StandEvaluator::calculate_constraints(bool early_termination) {
        double fallen_constraint = early_termination ? -1.0 : 0;
        return {
            fallen_constraint,
            0  // Second constraint unused, fixed to 0
        };
    }

    void StandEvaluator::load_script(std::string script_path) {
        if (utility::file::exists(script_path)) {
            NUClear::log<NUClear::DEBUG>("Loading script: ", script_path, '\n');
            script = YAML::LoadFile(script_path).as<::extension::Script>();
        }
        else {
            NUClear::log<NUClear::ERROR>("No script found at: ", script_path, '\n');
        }
    }

    void StandEvaluator::save_script(std::string script_path) {
        NUClear::log<NUClear::DEBUG>("Saving as: ", script_path);
        YAML::Node n(script);
        utility::file::writeToFile(script_path, n);
    }

    void StandEvaluator::run_script(size_t subsumption_id, NSGA2Evaluator* evaluator) {
        evaluator->emit(std::make_unique<extension::ExecuteScript>(subsumption_id, script, NUClear::clock::now()));
    }

    void StandEvaluator::update_max_field_plane_sway(const Sensors& sensors) {
        auto accelerometer = sensors.accelerometer;

        // Calculate the robot sway along the field plane (left/right, forward/backward)
        double field_plane_sway = std::pow(std::pow(accelerometer.x(), 2) + std::pow(accelerometer.y(), 2), 0.5);
        if (field_plane_sway > max_field_plane_sway) {
            max_field_plane_sway = field_plane_sway;
        }
    }

}  // namespace module::support::optimisation
