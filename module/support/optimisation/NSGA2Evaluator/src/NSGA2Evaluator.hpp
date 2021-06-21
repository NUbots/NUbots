#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_H
#define MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

namespace module {
    namespace support {
        namespace optimisation {

            class NSGA2Evaluator : public NUClear::Reactor {

            public:
                /// @brief Called by the powerplant to build and setup the NSGA2Evaluator reactor.
                explicit NSGA2Evaluator(std::unique_ptr<NUClear::Environment> environment);

                enum State {
                    UNKNOWN,
                    WAITING_FOR_REQUEST,
                    RESETTING_SIMULATION,
                    STANDING,
                    WALKING,
                    TERMINATING_EARLY,
                    TERMINATING_GRACEFULLY,
                    SENDING_FITNESS_SCORES
                };

                enum Event {
                    TimeUpdate,
                    EvaluateRequest,
                    ResetDone,
                    RawSensors,
                    Fallen,
                    RobotLocation,
                    TrialTimeExpired,
                    CalculatedFitness,
                    FitnessScoresSent
                };

            private:
                /// @brief Evaluate the fitness of the current individual
                void CalculateFitness();

                ///  @brief Emits the evaluation results back to the Optimiser for advancement
                void SendFitnessScores();

                /// @brief Resets our state and the simulator world for the next evaluation
                void ResetWorld();

                /// @brief Reset the simulation time
                void ResetWorldTime();

                /// @brief Start ending the current evaluation
                void BeginTermination();

                /// @brief Subsumption priority ID for this module
                size_t subsumptionId;

                /// @brief The number of the current generation
                int generation = 0;

                /// @brief The number of the current individual in the current generation
                int id = 0;

                /// @brief The evaluated fitness scores for the current individual (sway, ball distance)
                std::vector<double> scores = {0.0, 0.0};

                /// @brief The current simulation time
                double simTime = 0.0;

                /// @brief Delta between this update and the last update
                double simTimeDelta = 0.0;

                /// @brief The amount of simulation time that has elapsed since we started terminating the current
                /// evaluation
                double timeSinceTermination;

                /// @brief The walk command velocity
                Eigen::Vector2d walk_command_velocity = Eigen::Vector2d(0.5, 0.0);

                /// @brief The walk command rotation
                double walk_command_rotation = 0.0;

                /// @brief A list of constraints for domination calculation. These can be used to encode one or more
                /// failure conditions. Here, we set the first constraint if the robot has fallen over, and the second
                /// one if it's swayed too too much. The more negative the constraint value is, the more it has violated
                /// the constriant, which means it will be dominated by other individuals that have violated less.
                std::vector<double> constraints = {0.0, 0.0};

                /// @brief Sensor readings from the simulator, used during fitness and constraint calculation
                Eigen::Vector3d gyroscope     = Eigen::Vector3d::Zero();
                Eigen::Vector3d accelerometer = Eigen::Vector3d::Zero();

                /// @brief Robot state for this evaluation, used during fitness and constraint calculation
                Eigen::Vector3d robotPosition      = Eigen::Vector3d::Zero();
                Eigen::Vector3d robotPositionDelta = Eigen::Vector3d::Zero();
                double robotDistanceTravelled      = 0.0;

                /// @brief How much the robot swayed for this evaluation
                Eigen::Vector3d sway = Eigen::Vector3d::Zero();

                /// @brief How much the robot swayed in the field plane (forward/backward, left/right, but not up/down)
                double fieldPlaneSway = 0.0;

                /// @brief The max field plane sway observed during the evaluation
                double maxFieldPlaneSway;

                NSGA2EvaluationRequest lastEvalRequest;

                State currentState = State::WAITING_FOR_REQUEST;

                /// @brief Get the next state to transition to given the current state and an event
                State HandleTransition(State currentState, Event event);

                /// @brief Handle the WAITING_FOR_REQUEST state
                void WaitingForRequest(State previousState, Event event, bool isReEntry);

                /// @brief Handle the RESETTING_SIMULATION state
                void ResettingSimulation(State previousState, Event event, bool isReEntry);

                /// @brief Handle the STANDING state
                void Standing(State previousState, Event event, bool isReEntry);

                /// @brief Handle the WALKING state
                void Walking(State previousState, Event event, bool isReEntry);

                /// @brief Handle the TERMINATING_EARLY state
                void TerminatingEarly(State previousState, Event event, bool isReEntry);

                /// @brief Handle the TERMINATING_GRACEFULLY state
                void TerminatingGracefully(State previousState, Event event, bool isReEntry);

                /// @brief Handle the SENDING_FITNESS_SCORES state
                void SendingFitnessScores(State previousState, Event event, bool isReEntry);
            };

        }  // namespace optimisation
    }      // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_H
