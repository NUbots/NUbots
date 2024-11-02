/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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

#ifndef EXTENSION_BEHAVIOUR_RUNREASON_HPP
#define EXTENSION_BEHAVIOUR_RUNREASON_HPP

#include <memory>
#include <typeindex>

#include "Lock.hpp"

namespace extension::behaviour {

    /**
     * Provides information about how and why this provider was executed.
     */
    enum class RunReason {
        /// Something other than the Director caused this reaction to execute
        OTHER_TRIGGER,
        /// A new task has been given to this provider
        NEW_TASK,
        /// The provider has been started (will happen on on<Started<X>>)
        STARTED,
        /// The provider has been stopped (will happen on on<Stopped<X>>)
        STOPPED,
        /// A subtask has finished and emitted a Done message
        SUBTASK_DONE,
        /// Another task requires a causing from this provider and pushed it to run
        PUSHED
    };

    namespace information {

        struct RunReasonStore {
        private:
            using ThreadStore = NUClear::dsl::store::ThreadStore<const RunReason>;

        public:
            /**
             * Set the RunReason for this thread and return a lock that when destroyed will default to OTHER_TRIGGER.
             *
             * @param info the RunReason to set
             *
             * @return a lock object that once destroyed will clear to nullptr
             */
            static Lock set(const RunReason& info) {
                auto lock          = Lock(&info, [](const void*) { ThreadStore::value = nullptr; });
                ThreadStore::value = &info;
                return lock;
            }

            /**
             * Retrieves a shared pointer to a current RunReason object.
             *
             * This function returns a shared pointer to the current RunReason object provided by Director.
             * If the RunReasonStore's value is null, it returns a shared pointer to a new RunReason  with
             * OTHER_TRIGGER. Otherwise, it returns a shared pointer to a new RunReason object that is a copy of the
             * current value in RunReasonStore.
             *
             * @return a shared pointer to the current RunReason.
             */
            static RunReason get() {
                return ThreadStore::value == nullptr ? RunReason::OTHER_TRIGGER : *ThreadStore::value;
            }
        };

    }  // namespace information

}  // namespace extension::behaviour

#endif  // EXTENSION_BEHAVIOUR_RUNREASON_HPP
