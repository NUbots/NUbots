#ifndef MODULE_SUPPORT_OPTIMISATION_EVALUATORTASK_HPP
#define MODULE_SUPPORT_OPTIMISATION_EVALUATORTASK_HPP

#include <nuclear>
#include <vector>

#include "NSGA2Evaluator.hpp"

#include "extension/Behaviour.hpp"

#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"
#include "message/support/optimisation/OptimisationRobotPosition.hpp"

namespace module::support::optimisation {
    using message::input::Sensors;
    using message::platform::RawSensors;
    using message::support::optimisation::NSGA2EvaluationRequest;
    using message::support::optimisation::NSGA2FitnessScores;
    using message::support::optimisation::OptimisationRobotPosition;

    using ::extension::behaviour::Task;

    class NSGA2Evaluator;  // forward declaration

    class EvaluatorTask {
    public:
        /// @brief Determines if the robot has fallen over or not
        /// @param sensors Information on the robot's state, including IMU data
        /// @return True if the robot has fallen over, false otherwise
        virtual bool has_fallen(const Sensors& sensors) = 0;

        /// @brief Stores the position of the robot for use in the optimisation
        /// @param position The position of the robot in the world
        virtual void process_optimisation_robot_position(const OptimisationRobotPosition& position) = 0;

        /// @brief Set up the trial for the given optimisation problem and request
        /// @param request Specifics of the trial to run
        virtual void set_up_trial(const NSGA2EvaluationRequest& request) = 0;

        /// @brief Reset the trial for the given optimisation problem and request
        virtual void reset_trial() = 0;

        /// @brief Called when the trial is done to set up the robot for evaluation
        virtual void evaluating_state(NSGA2Evaluator* evaluator) = 0;

        /// @brief Calculates how well the individual performed
        /// @param early_termination If the individual stopped before the time had elapsed, eg the robot fell
        /// @param generation The generation this individual is from
        /// @param individual The individual's index within the generation
        /// @return The fitness scores for the individual
        virtual std::unique_ptr<NSGA2FitnessScores> calculate_fitness_scores(bool early_termination,
                                                                             int generation,
                                                                             int individual) = 0;
    };

}  // namespace module::support::optimisation

#endif  // MODULE_SUPPORT_OPTIMISATION_EVALUATORTASK_HPP
