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

#ifndef EXTENSION_BEHAVIOUR_INFORMATIONSOURCE_HPP
#define EXTENSION_BEHAVIOUR_INFORMATIONSOURCE_HPP

#include <memory>
#include <typeindex>

namespace extension::behaviour {

    /**
     * Provides information about the current state of a provider group
     */
    template <typename T>
    struct GroupInfo {
        enum RunState {
            /// The group has not emitted the task
            NO_TASK,
            /// The group is running the task
            RUNNING,
            /// The group has the task queued
            QUEUED
        };

        /// The current run state of the group
        RunState run_state;

        /// Whether the task is done or not, regardless of if this provider group is running it
        bool done;
    };

    namespace information {

        template <typename T>
        struct GroupInfoStore {
        private:
            using ThreadStore = NUClear::dsl::store::ThreadStore<std::shared_ptr<GroupInfo<T>>>;
            using GlobalStore = NUClear::util::TypeMap<T, T, GroupInfo<T>>;

            static public : using Lock = std::unique_ptr<void, std::function<void(void*)>>;

            /**
             * Set the group info for this thread and return a lock that when destroyed will upgrade the data to the
             * global store.
             *
             * @param info the group info to set
             *
             * @return a lock object that once destroyed will upgrade the data to the global store and clear the thread store
             */
            static Lock set(const std::shared_ptr<const GroupInfo<T>>& info) {

                auto lock          = std::unique_ptr<void, std::function<void (void*)>>(something_non_null,
                                                                   [](void*) {
                                                                       GlobalStore::set(info);
                                                                       ThreadStore::value = nullptr;
                                                                   });
                ThreadStore::value = &info;
                return lock;
            }

            static std::shared_ptr<const GroupInfo<T>> get() {
                return ThreadStore::value == nullptr ? GlobalStore<GroupType>::get()
                                                              : *ThreadStore<GroupType>::value;
            }

        }

    }  // namespace information

}  // namespace extension::behaviour

#endif  // EXTENSION_BEHAVIOUR_INFORMATIONSOURCE_HPP
