#include "NSGA2Evaluator.hpp"

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <yaml-cpp/yaml.h>

#include "tasks/RotationEvaluator.hpp"
#include "tasks/StandEvaluator.hpp"
#include "tasks/StrafeEvaluator.hpp"
#include "tasks/WalkEvaluator.hpp"

#include "extension/Configuration.hpp"
#include "extension/Script.hpp"

#include "message/behaviour/MotionCommand.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"
#include "message/support/optimisation/OptimisationResetDone.hpp"
#include "message/support/optimisation/OptimisationTimeUpdate.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/behaviour/MotionCommand.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::support::optimisation {

    using extension::Configuration;

    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::WalkCommand;
    using message::platform::RawSensors;
    using message::platform::webots::OptimisationCommand;
    using message::platform::webots::OptimisationRobotPosition;
    using message::support::optimisation::NSGA2EvaluationRequest;
    using message::support::optimisation::NSGA2EvaluatorReadinessQuery;
    using message::support::optimisation::NSGA2EvaluatorReady;
    using message::support::optimisation::NSGA2FitnessScores;
    using message::support::optimisation::NSGA2Terminate;
    using message::support::optimisation::NSGA2TrialExpired;
    using message::support::optimisation::OptimisationResetDone;
    using message::support::optimisation::OptimisationTimeUpdate;

    using utility::behaviour::RegisterAction;
    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::support::Expression;

    NSGA2Evaluator::NSGA2Evaluator(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), subsumption_id(size_t(this) * size_t(this) - size_t(this)) {
        log<NUClear::INFO>("Setting up the NSGA2 evaluator");

        emit<Scope::DIRECT>(std::make_unique<RegisterAction>(RegisterAction{
            subsumption_id,
            "NSGA2 Evaluator",
            {std::pair<float, std::set<LimbID>>(
                1,
                {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
            [this](const std::set<LimbID>& given_limbs) {
                if (given_limbs.find(LimbID::LEFT_LEG) != given_limbs.end()) {
                    // Enable the walk engine.
                    emit<Scope::DIRECT>(std::make_unique<EnableWalkEngineCommand>(subsumption_id));
                }
            },
            [this](const std::set<LimbID>& taken_limbs) {
                if (taken_limbs.find(LimbID::LEFT_LEG) != taken_limbs.end()) {
                    // Shut down the walk engine, since we don't need it right now.
                    emit<Scope::DIRECT>(std::make_unique<DisableWalkEngineCommand>(subsumption_id));
                }
            },
            [this](const std::set<ServoID>&) {}}));

        // Handle a state transition event
        on<Trigger<Event>, Sync<NSGA2Evaluator>>().then([this](const Event& event) {
            State old_state = current_state;
            State new_state = handle_transition(current_state, event);

            log<NUClear::DEBUG>("Transitioning on", event, ", from state", old_state, "to state", new_state);

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

        on<Trigger<RawSensors>, Single>().then([this](const RawSensors& sensors) {
            if (current_state == State::EVALUATING) {
                task->process_raw_sensor_msg(sensors, this);
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
        else if (last_eval_request_msg.task == "stand") {
            task = std::make_unique<StandEvaluator>();
        }
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
        reset->command                             = OptimisationCommand::CommandType::RESET_WORLD;
        emit(reset);
    }

    /// @brief Handle the EVALUATING state
    void NSGA2Evaluator::evaluating(NSGA2Evaluator::Event event) {
        log<NUClear::DEBUG>("Evaluating");

        if (event == Event::RESET_DONE) {
            if (last_eval_request_msg.task == "walk" || last_eval_request_msg.task == "stand"
                || last_eval_request_msg.task == "strafe" || last_eval_request_msg.task == "rotation") {
                task->evaluating_state(subsumption_id, this);
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
        log<NUClear::DEBUG>("Scheduling expired message with time ", delay_time.count());
        emit<Scope::DELAY>(message, delay_time);
    }

    /// @brief Handle the TERMINATING_EARLY state
    void NSGA2Evaluator::terminating_early() {
        log<NUClear::DEBUG>("Terminating Early");

        // Send a zero walk command to stop walking
        emit(std::make_unique<WalkCommand>(subsumption_id, Eigen::Vector3d(0.0, 0.0, 0.0)));
        bool early_termination = true;
        auto fitness_scores    = task->calculate_fitness_scores(early_termination, sim_time, generation, individual);
        emit(fitness_scores);

        emit(std::make_unique<Event>(Event::FITNESS_SCORES_SENT));  // Go back to waiting for the next request
    }

    /// @brief Handle the TERMINATING_GRACEFULLY state
    void NSGA2Evaluator::terminating_gracefully() {
        log<NUClear::DEBUG>("Terminating Gracefully");

        // Send a zero walk command to stop walking
        emit(std::make_unique<WalkCommand>(subsumption_id, Eigen::Vector3d(0.0, 0.0, 0.0)));
        bool early_termination = false;
        auto fitness_scores    = task->calculate_fitness_scores(early_termination, sim_time, generation, individual);
        emit(fitness_scores);

        emit(std::make_unique<Event>(Event::FITNESS_SCORES_SENT));  // Go back to waiting for the next request
    }

    void NSGA2Evaluator::finished() {
        log<NUClear::INFO>("Finished");
    }
}  // namespace module::support::optimisation
