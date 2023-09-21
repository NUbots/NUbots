#include "NSGA2Evaluator.hpp"

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <yaml-cpp/yaml.h>

#include "tasks/RotationEvaluator.hpp"
// #include "tasks/StandEvaluator.hpp"
#include "tasks/StrafeEvaluator.hpp"
#include "tasks/WalkEvaluator.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Sensors.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"
#include "message/support/optimisation/OptimisationResetDone.hpp"
#include "message/support/optimisation/OptimisationTimeUpdate.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::support::optimisation {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using message::platform::webots::OptimisationCommand;
    using message::platform::webots::OptimisationRobotPosition;
    using message::skill::Walk;
    using message::strategy::FallRecovery;
    using message::support::optimisation::NSGA2Evaluating;
    using message::support::optimisation::NSGA2EvaluationRequest;
    using message::support::optimisation::NSGA2EvaluatorReadinessQuery;
    using message::support::optimisation::NSGA2EvaluatorReady;
    using message::support::optimisation::NSGA2FitnessScores;
    using message::support::optimisation::NSGA2Terminate;
    using message::support::optimisation::NSGA2TrialExpired;
    using message::support::optimisation::OptimisationResetDone;
    using message::support::optimisation::OptimisationTimeUpdate;

    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::support::Expression;

    NSGA2Evaluator::NSGA2Evaluator(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Startup>().then([this] {
            // On the lowest level, just stand
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 1);

            // At the start of the program, we should be standing
            // Without these emits, modules that need a Stability and WalkState messages may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));


            // // On the highest level, recover from falling
            // emit<Task>(std::make_unique<FallRecovery>(), 3);
        });

        // Handle a state transition event
        on<Trigger<Event>, Sync<NSGA2Evaluator>>().then([this](const Event& event) {
            State old_state            = current_state;
            State new_state            = handle_transition(current_state, event);
            const char* state_string[] = {"UNKNOWN",
                                          "WAITING_FOR_REQUEST",
                                          "SETTING_UP_TRIAL",
                                          "RESETTING_SIMULATION",
                                          "EVALUATING",
                                          "TERMINATING_EARLY",
                                          "TERMINATING_GRACEFULLY",
                                          "FINISHED"};
            const char* event_string[] = {"RESET_DONE",
                                          "CHECK_READY",
                                          "EVALUATE_REQUEST",
                                          "TERMINATE_EVALUATION",
                                          "TRIAL_SETUP_DONE",
                                          "TERMINATE_EARLY",
                                          "TRIAL_COMPLETED",
                                          "FITNESS_SCORES_SENT"};
            log<NUClear::DEBUG>("Transitioning on",
                                event_string[event],
                                ", from state",
                                state_string[old_state],
                                "to state",
                                state_string[new_state]);

            switch (new_state) {
                case State::WAITING_FOR_REQUEST:
                    current_state = new_state;
                    waiting_for_request();
                    break;
                case State::SETTING_UP_TRIAL:
                    current_state = new_state;
                    setting_up_trial();
                    break;
                case State::RESETTING_SIMULATION:
                    current_state = new_state;
                    resetting_simulation();
                    break;
                case State::EVALUATING:
                    current_state = new_state;
                    evaluating(event);
                    break;
                case State::TERMINATING_EARLY:
                    current_state = new_state;
                    terminating_early();
                    break;
                case State::TERMINATING_GRACEFULLY:
                    current_state = new_state;
                    terminating_gracefully();
                    break;
                case State::FINISHED:
                    current_state = new_state;
                    finished();
                    break;
                default: log<NUClear::WARN>("Unable to transition to unknown state from", current_state, "on", event);
            }
        });

        on<Trigger<NSGA2EvaluationRequest>, Single>().then([this](const NSGA2EvaluationRequest& request) {
            last_eval_request_msg = request;
            emit(std::make_unique<Event>(Event::EVALUATE_REQUEST));
        });

        on<Trigger<NSGA2TrialExpired>, Single>().then([this](const NSGA2TrialExpired& message) {
            // Only start terminating gracefully if the trial that just expired is the current one
            // (and not previous ones that have terminated early)
            if (message.generation == generation && message.individual == individual) {
                emit(std::make_unique<Event>(Event::TRIAL_COMPLETED));
            }
        });

        on<Trigger<Sensors>, Single>().then([this](const Sensors& sensors) {
            if (evaluation_running) {
                if (task->has_fallen(sensors)) {
                    emit(std::make_unique<Event>(Event::TERMINATE_EARLY));
                }
            }
        });

        on<Trigger<OptimisationResetDone>, Single>().then([this](const OptimisationResetDone&) {
            log<NUClear::INFO>("Reset done");
            emit(std::make_unique<Event>(Event::RESET_DONE));
        });

        on<Trigger<OptimisationTimeUpdate>, Single>().then(
            [this](const OptimisationTimeUpdate& update) { sim_time = update.sim_time; });

        on<Trigger<NSGA2Terminate>, Single>().then([this]() {
            // NSGA2Terminate is emitted when we've finished all generations and all individuals
            emit(std::make_unique<Event>(Event::TERMINATE_EVALUATION));
        });

        on<Trigger<NSGA2EvaluatorReadinessQuery>, Single>().then([this]() {
            // NSGA2EvaluatorReadinessQuery is the optimiser checking if we're ready
            emit(std::make_unique<Event>(Event::CHECK_READY));
        });

        on<Trigger<OptimisationRobotPosition>, Single>().then([this](const OptimisationRobotPosition& position) {
            if (current_state == State::EVALUATING) {
                task->process_optimisation_robot_position(position);
            }
        });
    }

    NSGA2Evaluator::State NSGA2Evaluator::handle_transition(NSGA2Evaluator::State current_state,
                                                            NSGA2Evaluator::Event event) {
        switch (current_state) {
            case State::WAITING_FOR_REQUEST: return transition_events(event);
            case State::SETTING_UP_TRIAL: return transition_events(event);
            case State::RESETTING_SIMULATION: return transition_events(event);
            case State::EVALUATING: return transition_events(event);
            case State::TERMINATING_EARLY: return transition_events(event);
            case State::TERMINATING_GRACEFULLY: return transition_events(event);
            case State::FINISHED:
                // Arguably this should return FINISHED regardless of event, unless we want to be able to
                // reset
                if (event == Event::FITNESS_SCORES_SENT) {
                    return State::FINISHED;
                }
                else {
                    return transition_events(event);
                }

            default: return State::UNKNOWN;
        }
    }

    NSGA2Evaluator::State NSGA2Evaluator::transition_events(NSGA2Evaluator::Event event) {
        switch (event) {
            case Event::EVALUATE_REQUEST: return State::SETTING_UP_TRIAL;
            case Event::CHECK_READY: return State::WAITING_FOR_REQUEST;
            case Event::TERMINATE_EVALUATION: return State::FINISHED;
            case Event::TRIAL_SETUP_DONE: return State::RESETTING_SIMULATION;
            case Event::RESET_DONE: return State::EVALUATING;
            case Event::TERMINATE_EARLY: return State::TERMINATING_EARLY;
            case Event::TRIAL_COMPLETED: return State::TERMINATING_GRACEFULLY;
            case Event::FITNESS_SCORES_SENT: return State::WAITING_FOR_REQUEST;

            default: return State::UNKNOWN;
        }
    }

    void NSGA2Evaluator::walk(Eigen::Vector3d vec3) {
        emit<Task>(std::make_unique<Walk>(vec3), 1);
    }

    /// @brief Handle the WAITING_FOR_REQUEST state
    void NSGA2Evaluator::waiting_for_request() {
        log<NUClear::DEBUG>("Waiting For Request");
        emit(std::make_unique<NSGA2EvaluatorReady>());  // Let the optimiser know we're ready
    }

    /// @brief Handle the SETTING_UP_TRIAL state
    void NSGA2Evaluator::setting_up_trial() {
        log<NUClear::DEBUG>("Setting Up Trial");

        generation = last_eval_request_msg.generation;
        individual = last_eval_request_msg.id;

        if (last_eval_request_msg.task == "walk") {
            task = std::make_unique<WalkEvaluator>();
        }
        else if (last_eval_request_msg.task == "strafe") {
            task = std::make_unique<StrafeEvaluator>();
        }
        else if (last_eval_request_msg.task == "rotation") {
            task = std::make_unique<RotationEvaluator>();
        }
        // else if (last_eval_request_msg.task == "stand") {
        //     task = std::make_unique<StandEvaluator>();
        // }
        else {
            log<NUClear::ERROR>("Unhandled task type:", last_eval_request_msg.task);
        }

        task->set_up_trial(last_eval_request_msg);

        emit(std::make_unique<Event>(Event::TRIAL_SETUP_DONE));
    }

    /// @brief Handle the RESETTING_SIMULATION state
    void NSGA2Evaluator::resetting_simulation() {
        log<NUClear::DEBUG>("Resetting Simulation");

        task->reset_simulation();

        // Tell Webots to reset the world
        std::unique_ptr<OptimisationCommand> reset = std::make_unique<OptimisationCommand>();
        reset->command                             = OptimisationCommand::CommandType::RESET_ROBOT;
        emit(reset);
    }

    /// @brief Handle the EVALUATING state
    void NSGA2Evaluator::evaluating(NSGA2Evaluator::Event event) {
        log<NUClear::DEBUG>("Evaluating");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (event == Event::RESET_DONE) {
            if (last_eval_request_msg.task == "walk" || last_eval_request_msg.task == "stand"
                || last_eval_request_msg.task == "strafe" || last_eval_request_msg.task == "rotation") {
                task->evaluating_state(this);
                evaluation_running = true;
            }
            else {
                log<NUClear::ERROR>("Unhandled task type:", last_eval_request_msg.task);
            }
        }
    }

    void NSGA2Evaluator::schedule_trial_expired_message(const int trial_stage, const std::chrono::seconds delay_time) {
        // Prepare the trial expired message
        std::unique_ptr<NSGA2TrialExpired> message = std::make_unique<NSGA2TrialExpired>();
        message->time_started                      = sim_time;
        message->generation                        = generation;
        message->individual                        = individual;
        message->trial_stage                       = trial_stage;

        // Schedule the end of the walk trial after the duration limit
        log<NUClear::DEBUG>("Scheduling expired message with time", delay_time.count());
        emit<Scope::DELAY>(message, delay_time);
    }

    /// @brief Handle the TERMINATING_EARLY state
    void NSGA2Evaluator::terminating_early() {
        log<NUClear::DEBUG>("Terminating Early");

        evaluation_running = false;

        // Send a zero walk command to stop walking
        emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 1);
        bool early_termination = true;
        auto fitness_scores    = task->calculate_fitness_scores(early_termination, sim_time, generation, individual);
        emit(fitness_scores);

        emit(std::make_unique<Event>(Event::FITNESS_SCORES_SENT));  // Go back to waiting for the next request
    }

    /// @brief Handle the TERMINATING_GRACEFULLY state
    void NSGA2Evaluator::terminating_gracefully() {
        log<NUClear::DEBUG>("Terminating Gracefully");

        // Send a zero walk command to stop walking
        emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 1);
        bool early_termination = false;
        auto fitness_scores    = task->calculate_fitness_scores(early_termination, sim_time, generation, individual);
        emit(fitness_scores);

        emit(std::make_unique<Event>(Event::FITNESS_SCORES_SENT));  // Go back to waiting for the next request
    }

    void NSGA2Evaluator::finished() {
        log<NUClear::INFO>("Finished");
    }
}  // namespace module::support::optimisation
