#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_HPP
#define MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "extension/Behaviour.hpp"
#include "tasks/EvaluatorTask.hpp"

#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

namespace module::support::optimisation {

    using message::support::optimisation::NSGA2EvaluationRequest;

    class EvaluatorTask;  // Forward declaration

    class NSGA2Evaluator : public ::extension::behaviour::BehaviourReactor {

    public:
        /// @brief Called by the powerplant to build and setup the NSGA2Evaluator reactor.
        explicit NSGA2Evaluator(std::unique_ptr<NUClear::Environment> environment);
        void schedule_trial_expired_message(const int trial_stage, const std::chrono::seconds delay_time);

        void walk(Eigen::Vector3d vec3);

        enum State {
            UNKNOWN                = 0,
            WAITING_FOR_REQUEST    = 1,
            SETTING_UP_TRIAL       = 2,
            RESETTING_SIMULATION   = 3,
            EVALUATING             = 4,
            TERMINATING_EARLY      = 5,
            TERMINATING_GRACEFULLY = 6,
            FINISHED               = 7
        };

        enum Event {
            // From webots
            RESET_DONE = 0,

            // From optimiser
            CHECK_READY          = 1,
            EVALUATE_REQUEST     = 2,
            TERMINATE_EVALUATION = 3,

            // Internal
            TRIAL_SETUP_DONE    = 4,
            TERMINATE_EARLY     = 5,
            TRIAL_COMPLETED     = 6,
            FITNESS_SCORES_SENT = 7
        };

        /// @brief The current simulation time
        double sim_time = 0.0;

    private:
        /// @brief Pointer to an instance of the task being evaluated
        std::unique_ptr<EvaluatorTask> task;

        /// @brief Subsumption priority ID for this module
        size_t subsumption_id;

        /// @brief The number of the current generation
        int generation = 0;

        /// @brief The number of the current individual in the current generation
        int individual = 0;

        /// @brief Enables fallover checking when an evaluation is actually running
        bool evaluation_running = false;

        /// @brief Keeps track of the last messages we received
        NSGA2EvaluationRequest last_eval_request_msg;

        /// @brief Send the evaluated fitness scores
        /// @param scores The evaluated fitness scores for the current individual (sway, distance travelled)
        /// @param constraints A list of constraints for domination calculation. These can be used to encode one
        /// or more failure conditions. Here, we set the first constraint if the robot has fallen over, and the
        /// second one if it's swayed too too much. The more negative the constraint value is, the more it has
        /// violated the constriant, which means it will be dominated by other individuals that have violated
        /// less.
        void send_fitness_scores(std::vector<double> scores, std::vector<double> constraints);

        /// @brief Implementation of the fitness function (i.e. how well the individual did)
        std::vector<double> calculate_scores();

        /// @brief Implementation of the constraints function (i.e. how much to penalise for violations)
        std::vector<double> calculate_constraints();

        /// @brief Returns the appropriate values for the case where no constraints are violated
        std::vector<double> constraints_not_violated();

        /// @brief The current state of the evaluation
        State current_state = State::WAITING_FOR_REQUEST;

        /// @brief Get the next state to transition to given the current state and an event
        State handle_transition(State current_state, Event event);

        State transition_events(NSGA2Evaluator::Event event);

        /// @brief Handle the WAITING_FOR_REQUEST state
        void waiting_for_request();

        /// @brief Handle the SETTING_UP_TRIAL state
        void setting_up_trial();

        /// @brief Handle the RESETTING_SIMULATION state
        void resetting_simulation();

        /// @brief Handle the STANDING state
        void standing();

        /// @brief Handle the EVALUATING state
        void evaluating(Event event);

        /// @brief Handle the TERMINATING_EARLY state
        void terminating_early();

        /// @brief Handle the TERMINATING_GRACEFULLY state
        void terminating_gracefully();

        /// @brief Handle the FINISHED state
        void finished();
    };

}  // namespace module::support::optimisation

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_HPP
