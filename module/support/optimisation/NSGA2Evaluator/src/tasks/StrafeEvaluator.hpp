#ifndef MODULE_SUPPORT_OPTIMISATION_STRAFEEVALUATOR_H
#define MODULE_SUPPORT_OPTIMISATION_STRAFEEVALUATOR_H

#include <Eigen/Core>
#include <nuclear>
#include <vector>

#include "EvaluatorTask.hpp"

#include "message/platform/RawSensors.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

namespace module::support::optimisation {
    using message::platform::RawSensors;
    using message::support::optimisation::NSGA2EvaluationRequest;
    using message::support::optimisation::NSGA2FitnessScores;

    class StrafeEvaluator : public EvaluatorTask {
    public:
        // Implementing the EvaluatorTask interface
        void process_raw_sensor_msg(const RawSensors& sensors, NSGA2Evaluator* evaluator);
        void process_optimisation_robot_position(const OptimisationRobotPosition& position);
        void set_up_trial(const NSGA2EvaluationRequest& request);
        void reset_simulation();
        void evaluating_state(size_t subsumption_id, NSGA2Evaluator* evaluator);
        std::unique_ptr<NSGA2FitnessScores> calculate_fitness_scores(bool early_termination,
                                                                     double sim_time,
                                                                     int generation,
                                                                     int individual);

        // Task-specific functions
        std::vector<double> calculate_scores();
        std::vector<double> calculate_constraints(double simTime);
        std::vector<double> constraints_not_violated();
        bool check_for_fall(const RawSensors& sensors);
        void update_max_field_plane_sway(const RawSensors& sensors);
        bool check_off_course();

    private:
        /// @brief Robot state for this evaluation, used during fitness and constraint calculation
        bool initial_position_set              = false;
        Eigen::Vector3d initial_robot_position = Eigen::Vector3d::Zero();
        Eigen::Vector3d robot_position         = Eigen::Vector3d::Zero();
        double max_field_plane_sway            = 0.0;

        /// @brief The amount of time to run a single trial, in seconds.
        std::chrono::seconds trial_duration_limit = std::chrono::seconds(0);

        /// @brief Keep track of when the trial started
        double trial_start_time = 0.0;

        /// @brief The walk command velocity.
        Eigen::Vector2d walk_command_velocity = Eigen::Vector2d(0.0, 0.0);

        /// @brief The walk command rotation.
        double walk_command_rotation = 0.0;

        /// @brief Configuration Min and Max values
        float gravity_max = 0.0;
        float gravity_min = 0.0;
    };

}  // namespace module::support::optimisation

#endif  // MODULE_SUPPORT_OPTIMISATION_STRAFEEVALUATOR_H