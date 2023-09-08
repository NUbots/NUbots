#include "StandEvaluator.hpp"

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "message/actuation/Limbs.hpp"
#include "message/skill/Walk.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

#include "utility/file/fileutil.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/skill/Script.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::support::optimisation {
    using message::actuation::LimbsSequence;
    using message::input::Sensors;
    using message::skill::Walk;
    using message::support::optimisation::NSGA2EvaluationRequest;
    using message::support::optimisation::NSGA2FitnessScores;
    using message::support::optimisation::NSGA2TrialExpired;

    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::support::Expression;

    void StandEvaluator::process_optimisation_robot_position(const OptimisationRobotPosition& position) {
        robot_position = position.value;
    }

    void StandEvaluator::set_up_trial(const NSGA2EvaluationRequest& current_request) {
        load_script(current_request.task_config_path);
        std::chrono::milliseconds limit_ms = std::chrono::milliseconds(0);
        for (size_t i = 0; i < current_request.parameters.real_params.size(); i++) {
            int frame_time = current_request.parameters.real_params[i];
            limit_ms       = limit_ms + std::chrono::milliseconds(frame_time);
            // script.frames[i].duration = std::chrono::milliseconds(frame_time);
        }
        save_script(fmt::format("gen{:03d}_ind{:03d}_task-{}.yaml",
                                current_request.generation,
                                current_request.id,
                                current_request.task));

        auto overhead = std::chrono::seconds(2);  // Overhead tacked on to give the robot time to fall over if unstable
        trial_duration_limit = std::chrono::duration_cast<std::chrono::seconds>(limit_ms) + overhead;

        // Get constant variables
        YAML::Node config = YAML::LoadFile("config/NSGA2Evaluator.yaml");

        cfg.fallen_angle = config["fallen_angle"].as<float>();

        fallen = false;
    }

    void StandEvaluator::reset_simulation() {
        // Reset our local state
        trial_start_time     = 0.0;
        robot_position       = Eigen::Vector3d::Zero();
        max_field_plane_sway = 0.0;
    }

    void StandEvaluator::evaluating_state(NSGA2Evaluator* evaluator) {
        NUClear::log<NUClear::DEBUG>("Running Script");
        run_script(evaluator);
        NUClear::log<NUClear::DEBUG>("schedule expire");
        evaluator->schedule_trial_expired_message(0, trial_duration_limit);
    }

    bool StandEvaluator::has_fallen(const Sensors& sensors) {
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

    void StandEvaluator::update_max_field_plane_sway(const Sensors& sensors) {
        auto accelerometer = sensors.accelerometer;

        // Calculate the robot sway along the field plane (left/right, forward/backward)
        double field_plane_sway = std::pow(std::pow(accelerometer.x(), 2) + std::pow(accelerometer.y(), 2), 0.5);
        if (field_plane_sway > max_field_plane_sway) {
            max_field_plane_sway = field_plane_sway;
        }
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
            script = script_path;
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

    void StandEvaluator::run_script(NSGA2Evaluator* evaluator) {
        evaluator->emit<Task>(utility::skill::load_script<LimbsSequence>(script));
    }

}  // namespace module::support::optimisation
