#ifndef MODULE_SUPPORT_OPTIMISATION_WALKEVALUATOR_HPP
#define MODULE_SUPPORT_OPTIMISATION_WALKEVALUATOR_HPP

#include <Eigen/Core>
#include <nuclear>
#include <vector>

#include "EvaluatorTask.hpp"

#include "message/input/Sensors.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

namespace module::support::optimisation {
    using message::input::Sensors;
    using message::support::optimisation::NSGA2EvaluationRequest;
    using message::support::optimisation::NSGA2FitnessScores;

    class WalkEvaluator : public EvaluatorTask {
    public:
        // Implementing the EvaluatorTask interface
        void process_optimisation_robot_position(const OptimisationRobotPosition& position);

        /// @brief Setup the next trial, updating the trial duration, velocity, and walk config parameters also saving
        /// them into a new file
        void set_up_trial(const NSGA2EvaluationRequest& request);

        /// @brief Reset the local state of the trial
        void reset_trial();

        /// @brief Starts an evaluation, running the walk and setting the duration
        /// @param evaluator Instance of the NSGA2Evaluator which this task is running from
        void evaluating_state(NSGA2Evaluator* evaluator);

        /// @brief Detects whether we have fallen
        bool has_fallen(const Sensors& sensors);

        /// @brief After an evaluation has completed calculates the scores for the individual
        /// @param early_termination Has the trial been forced to terminate early
        /// @param generation The generation that we have been evaluating
        /// @param individual The specific individual we were trialing
        std::unique_ptr<NSGA2FitnessScores> calculate_fitness_scores(bool early_termination,
                                                                     int generation,
                                                                     int individual);

        // Task-specific functions
        /// @brief Determine scores for the algorithm based on distance travelled and sway
        std::vector<double> calculate_scores();

        /// @brief Determine constraints for the algorithm, if we have fallen, based on the time that the trial ran for
        std::vector<double> calculate_constraints(double trial_duration);

        /// @brief Gives no constraints as a result of staying upright for the duration of the trial
        std::vector<double> constraints_not_violated();

        /// @brief Calculate the robot sway along the field plane (left/right, forward/backward)
        void update_max_field_plane_sway(const Sensors& sensors);

    private:
        struct Config {
            /// @brief Threshold angle for fallover detection, between torso z axis and world z axis
            float fallen_angle = 0.0f;
        } cfg;

        /// @brief Robot state for this evaluation, used during fitness and constraint calculation
        bool initial_position_set              = false;
        Eigen::Vector3d initial_robot_position = Eigen::Vector3d::Zero();
        Eigen::Vector3d robot_position         = Eigen::Vector3d::Zero();
        double max_field_plane_sway            = 0.0;

        /// @brief The amount of time to run a single trial, in seconds.
        std::chrono::seconds trial_duration_limit = std::chrono::seconds(0);

        /// @brief Keep track of when the trial started
        NUClear::clock::time_point trial_start_time{NUClear::clock::now()};

        /// @brief The walk command velocity.
        Eigen::Vector2d walk_command_velocity = Eigen::Vector2d(0.0, 0.0);

        /// @brief The walk command rotation.
        double walk_command_rotation = 0.0;

        /// @brief Indicates that a fall has occured.
        bool fallen = false;
    };

}  // namespace module::support::optimisation

#endif  // MODULE_SUPPORT_OPTIMISATION_WALKEVALUATOR_HPP
