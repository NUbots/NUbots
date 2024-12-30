/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
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
#ifndef MODULE_MOTION_SERVOS_HPP
#define MODULE_MOTION_SERVOS_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/input/Sensors.hpp"

#include "utility/actuation/ServoMap.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::actuation {
    using message::actuation::ServoTarget;
    using message::input::Sensors;
    using utility::input::ServoID;
    using utility::nusight::graph;

    class Servos : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Creates a reaction that sends a given servo command as a servo target command for the platform module
        /// to use
        // TODO(ysims): add capability to be Done when the servo reaches the target position
        template <typename Servo, ServoID::Value ID>
        void add_servo_provider() {
            on<Provide<Servo>, Every<90, Per<std::chrono::seconds>>, Priority::HIGH>().then(
                [this](const Servo& servo, const RunReason& run_reason) {
                    if (run_reason == RunReason::NEW_TASK) {
                        if (log_level <= DEBUG) {
                            emit(graph("Servo " + std::to_string(ID) + " (Position, Gain, Torque Enabled): ",
                                       servo.command.position,
                                       servo.command.state.gain,
                                       servo.command.state.torque));
                        }
                        emit(std::make_unique<ServoTarget>(servo.command.time,
                                                           ID,
                                                           servo.command.position,
                                                           servo.command.state.gain,
                                                           servo.command.state.torque));
                    }
                    // If the time to reach the position is over, then stop requesting the position
                    else if (NUClear::clock::now() >= servo.command.time) {
                        emit<Task>(std::make_unique<Done>());
                    }
                });
        }

        /// @brief Creates a reaction that takes a servo wrapper task (eg LeftLeg) and emits a task for each servo.
        /// Emits Done when all servo tasks are Done. If individual servos in that wrapper task were not provided, then
        /// it ignores them.
        /// @tparam Group is a servo wrapper task (eg LeftLeg)
        /// @tparam Elements is a template pack of Servos that Group uses
        template <typename Group, typename... Elements>
        void add_group_provider() {
            on<Provide<Group>, Needs<Elements>..., Priority::HIGH>().then(
                [this](const Group& group, const RunReason& run_reason, const Uses<Elements>... elements) {
                    // Check if any subtask is Done
                    if (run_reason == RunReason::SUBTASK_DONE) {
                        // If every servo task is done then emit Done (ignore servos that weren't included in the Task
                        // message)
                        if (((!group.servos.contains(utility::actuation::ServoMap<Elements>::value) || elements.done)
                             && ...)) {
                            emit<Task>(std::make_unique<Done>());
                            return;
                        }
                        // Emit Continue if all the servos are not Done yet
                        emit<Task>(std::make_unique<Continue>());
                        return;
                    }

                    (
                        // Runs an emit for each servo, if that servo was included in the map
                        [&] {
                            // Check if the Task includes this servo
                            if (group.servos.contains(utility::actuation::ServoMap<Elements>::value)) {
                                // Emit a Task for the servo if the map contains a value for it
                                emit<Task>(std::make_unique<Elements>(
                                    group.servos.at(utility::actuation::ServoMap<Elements>::value)));
                            }
                            else {  // if a servo was not filled in the map for this group, log it
                                log<TRACE>("Requested a Servo Group Task but did not provide values for Servo",
                                           ServoID(utility::actuation::ServoMap<Elements>::value));
                            }
                        }(),
                        ...);
                });
        }

        /// @brief Keeps track of which Group in the Sequence in add_sequence_provider should be emitted next
        /// @details Should be used carefully and may be bug-prone.
        /// @tparam Sequence Used to have unique counts for each Sequence Provider.
        template <typename Sequence>
        struct Count {
            long unsigned int count = 0;
        };

        /// @brief Creates a reaction that takes a vector of servo wrapper tasks (eg LeftLegs) and emits each servo
        /// wrapper task after the previous one is Done. Emits Done when the last group is Done.
        /// @tparam Sequence is a vector of servo wrapper tasks (eg LeftLegSequence)
        /// @tparam Group is a servo wrapper task (eg LeftLeg)
        template <typename Sequence, typename Group>
        void add_sequence_provider() {
            // Message to keep track of which position in the vector is to be emitted
            // Make an initial count message
            emit<Scope::INLINE>(std::make_unique<Count<Sequence>>(0));

            on<Provide<Sequence>, Needs<Group>, With<Count<Sequence>>, Priority::HIGH>().then(
                [this](const Sequence& sequence, const RunReason& run_reason, const Count<Sequence>& count) {
                    // If the user gave us nothing then we are done
                    if (sequence.frames.empty()) {
                        emit<Task>(std::make_unique<Done>());
                    }
                    // If this is a new task, run the first pack of servos and increment the counter
                    else if (run_reason == RunReason::NEW_TASK) {
                        emit<Task>(std::make_unique<Group>(sequence.frames[0]));
                        emit<Scope::INLINE>(std::make_unique<Count<Sequence>>(1));
                    }
                    // If the subtask is done, we are done if it is the last servo frames, otherwise use the count to
                    // determine the current frame to emit
                    else if (run_reason == RunReason::SUBTASK_DONE) {
                        if (count.count < sequence.frames.size()) {
                            emit<Task>(std::make_unique<Group>(sequence.frames[count.count]));
                            emit<Scope::INLINE>(std::make_unique<Count<Sequence>>(count.count + 1));
                        }
                        else {
                            emit<Task>(std::make_unique<Done>());
                        }
                    }
                    // Any other run reason shouldn't happen
                    else {
                        emit<Task>(std::make_unique<Continue>());
                    }
                });
        }

    public:
        /// @brief Called by the powerplant to build and setup the Servos reactor.
        explicit Servos(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::actuation

#endif  // MODULE_MOTION_SERVOS_HPP
