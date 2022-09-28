#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_H
#define MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "tasks/EvaluatorTask.hpp"

#include "message/platform/RawSensors.hpp"
#include "message/support/optimisation/NSGA2EvaluatorMessages.hpp"
#include "message/support/optimisation/NSGA2OptimiserMessages.hpp"

namespace module {
    namespace support {
        namespace optimisation {

            using RawSensorsMsg = message::platform::RawSensors;
            using message::support::optimisation::NSGA2EvaluationRequest;
            using message::support::optimisation::NSGA2TrialExpired;

            class EvaluatorTask;  // Forward declaration

            class NSGA2Evaluator : public NUClear::Reactor {

            public:
                /// @brief Called by the powerplant to build and setup the NSGA2Evaluator reactor.
                explicit NSGA2Evaluator(std::unique_ptr<NUClear::Environment> environment);
                void ScheduleTrialExpiredMessage(const int trial_stage, const std::chrono::seconds delay_time);

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
                    ResetDone = 0,

                    // From optimiser
                    CheckReady          = 1,
                    EvaluateRequest     = 2,
                    TerminateEvaluation = 3,

                    // Internal
                    TrialSetupDone    = 4,
                    TerminateEarly    = 5,
                    TrialCompleted    = 6,
                    FitnessScoresSent = 7
                };

                /// @brief The current simulation time
                double simTime = 0.0;

            private:
                std::unique_ptr<EvaluatorTask> task;

                /// @brief Subsumption priority ID for this module
                size_t subsumptionId;

                // /// @brief The current simulation time
                // double simTime = 0.0;


                /// @brief The number of the current generation
                int generation = 0;

                /// @brief The number of the current individual in the current generation
                int individual = 0;

                /// @brief Keeps track of the last messages we received
                NSGA2EvaluationRequest lastEvalRequestMsg;

                /// @brief Send the evaluated fitness scores
                /// @param scores The evaluated fitness scores for the current individual (sway, distance travelled)
                /// @param constraints A list of constraints for domination calculation. These can be used to encode one
                /// or more failure conditions. Here, we set the first constraint if the robot has fallen over, and the
                /// second one if it's swayed too too much. The more negative the constraint value is, the more it has
                /// violated the constriant, which means it will be dominated by other individuals that have violated
                /// less.
                void SendFitnessScores(std::vector<double> scores, std::vector<double> constraints);

                /// @brief Implementation of the fitness function (i.e. how well the individual did)
                std::vector<double> CalculateScores();

                /// @brief Implementation of the constraints function (i.e. how much to penalise for violations)
                std::vector<double> CalculateConstraints();

                /// @brief Returns the appropriate values for the case where no constraints are violated
                std::vector<double> ConstraintsNotViolated();

                /// @brief The current state of the evaluation
                State currentState = State::WAITING_FOR_REQUEST;

                /// @brief Get the next state to transition to given the current state and an event
                State HandleTransition(State currentState, Event event);

                /// @brief Handle the WAITING_FOR_REQUEST state
                void WaitingForRequest();

                /// @brief Handle the SETTING_UP_TRIAL state
                void SettingUpTrial();

                /// @brief Handle the RESETTING_SIMULATION state
                void ResettingSimulation();

                /// @brief Handle the STANDING state
                void Standing();

                /// @brief Handle the EVALUATING state
                void Evaluating(Event event);

                /// @brief Handle the TERMINATING_EARLY state
                void TerminatingEarly();

                /// @brief Handle the TERMINATING_GRACEFULLY state
                void TerminatingGracefully();

                /// @brief Handle the FINISHED state
                void Finished();
            };

        }  // namespace optimisation
    }      // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_H
