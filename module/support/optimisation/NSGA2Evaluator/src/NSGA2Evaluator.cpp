/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "NSGA2Evaluator.hpp"

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <yaml-cpp/yaml.h>

#include "tasks/RotationEvaluator.hpp"
#include "tasks/StrafeEvaluator.hpp"
#include "tasks/WalkEvaluator.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"
#include "message/support/optimisation/OptimisationResetDone.hpp"
#include "message/support/optimisation/OptimisationRobotPosition.hpp"
#include "message/support/optimisation/OptimisationTimeUpdate.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::support::optimisation {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using message::skill::Walk;
    using message::strategy::FallRecovery;
    using message::support::optimisation::NSGA2Evaluating;
    using message::support::optimisation::NSGA2EvaluationRequest;
    using message::support::optimisation::NSGA2EvaluatorReadinessQuery;
    using message::support::optimisation::NSGA2EvaluatorReady;
    using message::support::optimisation::NSGA2FitnessScores;
    using message::support::optimisation::NSGA2Terminate;
    using message::support::optimisation::NSGA2TrialExpired;
    using message::support::optimisation::OptimisationCommand;
    using message::support::optimisation::OptimisationResetDone;
    using message::support::optimisation::OptimisationRobotPosition;
    using message::support::optimisation::OptimisationTimeUpdate;

    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::support::Expression;

    NSGA2Evaluator::NSGA2Evaluator(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Startup>().then([this] {
            // When starting up have the robot stand
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 1);
        });

        // Handle a state transition event
        on<Trigger<Event>, Sync<NSGA2Evaluator>>().then([this](const Event& event) {
            State old_state = current_state;
            State new_state = handle_transition(current_state, event);

            log<NUClear::DEBUG>("Transitioning on", event, ", from state", old_state, "to state", new_state);

            switch (new_state.value) {
                case State::Value::WAITING_FOR_REQUEST:
                    current_state = new_state;
                    waiting_for_request();
                    break;
                case State::Value::SETTING_UP_TRIAL:
                    current_state = new_state;
                    setting_up_trial();
                    break;
                case State::Value::RESETTING_TRIAL:
                    current_state = new_state;
                    resetting_trial();
                    break;
                case State::Value::EVALUATING:
                    current_state = new_state;
                    evaluating(event);
                    break;
                case State::Value::TERMINATING_EARLY:
                    current_state = new_state;
                    terminating_early();
                    break;
                case State::Value::TERMINATING_GRACEFULLY:
                    current_state = new_state;
                    terminating_gracefully();
                    break;
                case State::Value::FINISHED:
                    current_state = new_state;
                    finished();
                    break;
                default: log<NUClear::WARN>("Unable to transition to unknown state from", current_state, "on", event);
            }
        });

        on<Trigger<NSGA2EvaluationRequest>, Single>().then([this](const NSGA2EvaluationRequest& request) {
            last_eval_request_msg = request;
            emit(std::make_unique<Event>(Event(Event::Value::EVALUATE_REQUEST)));
        });

        on<Trigger<NSGA2TrialExpired>, Single>().then([this](const NSGA2TrialExpired& message) {
            // Only start terminating gracefully if the trial that just expired is the current one
            // (and not previous ones that have terminated early)
            if (message.generation == generation && message.individual == individual) {
                emit(std::make_unique<Event>(Event(Event::Value::TRIAL_COMPLETED)));
            }
        });

        on<Trigger<Sensors>, Single>().then([this](const Sensors& sensors) {
            // When an evaluaton is running, check to see if we have fallen
            if (evaluation_running) {
                if (task->has_fallen(sensors)) {
                    emit(std::make_unique<Event>(Event(Event::Value::TERMINATE_EARLY)));
                }
            }
        });

        on<Trigger<OptimisationResetDone>, Single>().then([this](const OptimisationResetDone&) {
            log<NUClear::INFO>("Reset done");
            task->reset_trial();
            emit(std::make_unique<Event>(Event(Event::Value::RESET_DONE)));
        });

        on<Trigger<NSGA2Terminate>, Single>().then([this]() {
            // NSGA2Terminate is emitted when we've finished all generations and all individuals
            emit(std::make_unique<Event>(Event(Event::Value::TERMINATE_EVALUATION)));
        });

        on<Trigger<NSGA2EvaluatorReadinessQuery>, Single>().then([this]() {
            // NSGA2EvaluatorReadinessQuery is the optimiser checking if we're ready
            emit(std::make_unique<Event>(Event(Event::Value::CHECK_READY)));
        });

        on<Trigger<OptimisationRobotPosition>, Single>().then([this](const OptimisationRobotPosition& position) {
            if (current_state.value == State::Value::EVALUATING) {
                task->process_optimisation_robot_position(position);
            }
        });
    }

    NSGA2Evaluator::State NSGA2Evaluator::handle_transition(NSGA2Evaluator::State current_state,
                                                            NSGA2Evaluator::Event event) {
        switch (current_state.value) {
            case State::Value::WAITING_FOR_REQUEST: return transition_events(event);
            case State::Value::SETTING_UP_TRIAL: return transition_events(event);
            case State::Value::RESETTING_TRIAL: return transition_events(event);
            case State::Value::EVALUATING: return transition_events(event);
            case State::Value::TERMINATING_EARLY: return transition_events(event);
            case State::Value::TERMINATING_GRACEFULLY: return transition_events(event);
            case State::Value::FINISHED:
                // Arguably this should return FINISHED regardless of event, unless we want to be able to
                // reset
                if (event.value == Event::Value::FITNESS_SCORES_SENT) {
                    return State(State::Value::FINISHED);
                }
                else {
                    return transition_events(event);
                }

            default: return State(State::Value::UNKNOWN);
        }
    }

    NSGA2Evaluator::State NSGA2Evaluator::transition_events(NSGA2Evaluator::Event event) {
        switch (event.value) {
            case Event::Value::EVALUATE_REQUEST: return State(State::Value::SETTING_UP_TRIAL);
            case Event::Value::CHECK_READY: return State(State::Value::WAITING_FOR_REQUEST);
            case Event::Value::TERMINATE_EVALUATION: return State(State::Value::FINISHED);
            case Event::Value::TRIAL_SETUP_DONE: return State(State::Value::RESETTING_TRIAL);
            case Event::Value::RESET_DONE: return State(State::Value::EVALUATING);
            case Event::Value::TERMINATE_EARLY: return State(State::Value::TERMINATING_EARLY);
            case Event::Value::TRIAL_COMPLETED: return State(State::Value::TERMINATING_GRACEFULLY);
            case Event::Value::FITNESS_SCORES_SENT: return State(State::Value::WAITING_FOR_REQUEST);

            default: return State(State::Value::UNKNOWN);
        }
    }

    void NSGA2Evaluator::walk(Eigen::Vector3d vec3) {
        emit<Task>(std::make_unique<Walk>(vec3), 1);
    }

    // Handle the WAITING_FOR_REQUEST state
    void NSGA2Evaluator::waiting_for_request() {
        log<NUClear::DEBUG>("Waiting For Request");
        emit(std::make_unique<NSGA2EvaluatorReady>());  // Let the optimiser know we're ready
    }

    // Handle the SETTING_UP_TRIAL state
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
        else {
            log<NUClear::ERROR>("Unhandled task type:", last_eval_request_msg.task);
        }

        task->set_up_trial(last_eval_request_msg);

        emit(std::make_unique<Event>(Event(Event::Value::TRIAL_SETUP_DONE)));
    }

    // Handle the RESETTING_TRIAL state
    void NSGA2Evaluator::resetting_trial() {
        log<NUClear::DEBUG>("Resetting Trial");

        // Tell Webots to reset the world
        std::unique_ptr<OptimisationCommand> reset = std::make_unique<OptimisationCommand>();
        reset->command                             = OptimisationCommand::CommandType::RESET_ROBOT;
        emit(reset);
    }

    // Handle the EVALUATING state
    void NSGA2Evaluator::evaluating(NSGA2Evaluator::Event event) {
        log<NUClear::DEBUG>("Evaluating");

        if (event.value == Event::Value::RESET_DONE) {
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
        message->generation                        = generation;
        message->individual                        = individual;
        message->trial_stage                       = trial_stage;

        // Schedule the end of the walk trial after the duration limit
        log<NUClear::DEBUG>("Scheduling expired message with time", delay_time.count());
        emit<Scope::DELAY>(message, delay_time);
    }

    // Handle the TERMINATING_EARLY state
    void NSGA2Evaluator::terminating_early() {
        log<NUClear::DEBUG>("Terminating Early");

        evaluation_running = false;

        // Send a zero walk command to stop walking
        emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 1);
        bool early_termination = true;
        auto fitness_scores    = task->calculate_fitness_scores(early_termination, generation, individual);
        emit(fitness_scores);

        emit(std::make_unique<Event>(
            Event(Event::Value::FITNESS_SCORES_SENT)));  // Go back to waiting for the next request
    }

    // Handle the TERMINATING_GRACEFULLY state
    void NSGA2Evaluator::terminating_gracefully() {
        log<NUClear::DEBUG>("Terminating Gracefully");

        // Send a zero walk command to stop walking
        emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 1);
        bool early_termination = false;
        auto fitness_scores    = task->calculate_fitness_scores(early_termination, generation, individual);
        emit(fitness_scores);

        emit(std::make_unique<Event>(
            Event(Event::Value::FITNESS_SCORES_SENT)));  // Go back to waiting for the next request
    }

    void NSGA2Evaluator::finished() {
        log<NUClear::INFO>("Finished");
    }
}  // namespace module::support::optimisation
