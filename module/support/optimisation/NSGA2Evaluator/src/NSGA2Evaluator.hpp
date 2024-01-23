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
#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_HPP
#define MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <nuclear>

#include "tasks/EvaluatorTask.hpp"

#include "extension/Behaviour.hpp"

#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

namespace module::support::optimisation {

    using message::support::optimisation::NSGA2EvaluationRequest;

    class EvaluatorTask;  // Forward declaration

    class NSGA2Evaluator : public ::extension::behaviour::BehaviourReactor {

    public:
        /// @brief Called by the powerplant to build and setup the NSGA2Evaluator reactor.
        explicit NSGA2Evaluator(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Sets up a message to emit when the maximum time for this trial has elapsed
        /// @param trial_stage The stage this expiration is for, if running multi-stage trials
        /// @param delay_time The amount of time the trial has before expiring
        void schedule_trial_expired_message(const int trial_stage, const std::chrono::seconds delay_time);

        /// @brief Call to make the robot walk
        /// @param vec3 The direction to walk in (x m/s, y m/s, theta radians/s)
        void walk(Eigen::Vector3d vec3);

        /// @brief The current state of the optimisation
        struct State {
            enum Value {
                UNKNOWN                = 0,
                WAITING_FOR_REQUEST    = 1,
                SETTING_UP_TRIAL       = 2,
                RESETTING_TRIAL        = 3,
                EVALUATING             = 4,
                TERMINATING_EARLY      = 5,
                TERMINATING_GRACEFULLY = 6,
                FINISHED               = 7
            };
            Value value = Value::UNKNOWN;

            // Constructors
            State() = default;
            State(Value const& v) : value(v) {}

            // Overload output operator
            friend std::ostream& operator<<(std::ostream& os, const State& state) {
                switch (state.value) {
                    case Value::UNKNOWN: return os << "UNKNOWN";
                    case Value::WAITING_FOR_REQUEST: return os << "WAITING_FOR_REQUEST";
                    case Value::SETTING_UP_TRIAL: return os << "SETTING_UP_TRIAL";
                    case Value::RESETTING_TRIAL: return os << "RESETTING_TRIAL";
                    case Value::EVALUATING: return os << "EVALUATING";
                    case Value::TERMINATING_EARLY: return os << "TERMINATING_EARLY";
                    case Value::TERMINATING_GRACEFULLY: return os << "TERMINATING_GRACEFULLY";
                    case Value::FINISHED: return os << "FINISHED";
                    default: throw std::runtime_error("enum State's value is corrupt, unknown value stored");
                }
            }
        };

        /// @brief The current event that is happening in the optimisation
        struct Event {
            enum Value {
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

            /// @brief The event that is happening
            Value value = Value::RESET_DONE;

            // Constructors
            Event() = default;
            Event(Value const& v) : value(v) {}

            // Overload output operator
            friend std::ostream& operator<<(std::ostream& os, const Event& event) {
                switch (event.value) {
                    case Value::RESET_DONE: return os << "RESET_DONE";
                    case Value::CHECK_READY: return os << "CHECK_READY";
                    case Value::EVALUATE_REQUEST: return os << "EVALUATE_REQUEST";
                    case Value::TERMINATE_EVALUATION: return os << "TERMINATE_EVALUATION";
                    case Value::TRIAL_SETUP_DONE: return os << "TRIAL_SETUP_DONE";
                    case Value::TERMINATE_EARLY: return os << "TERMINATE_EARLY";
                    case Value::TRIAL_COMPLETED: return os << "TRIAL_COMPLETED";
                    case Value::FITNESS_SCORES_SENT: return os << "FITNESS_SCORES_SENT";
                    default: throw std::runtime_error("enum Event's value is corrupt, unknown value stored");
                }
            }
        };

    private:
        /// @brief Pointer to an instance of the task being evaluated
        std::unique_ptr<EvaluatorTask> task;

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

        /// @brief Handle the RESETTING_TRIAL state
        void resetting_trial();

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
