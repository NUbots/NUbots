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

#ifndef EXTENSION_BEHAVIOUR_GROUPINFO_HPP
#define EXTENSION_BEHAVIOUR_GROUPINFO_HPP

#include <memory>
#include <typeindex>

#include "Lock.hpp"

namespace extension::behaviour {

    /**
     * Provides information about the current state of a provider group
     */
    struct GroupInfo {
        struct TaskInfo {
            /// The unique ID of the task
            NUClear::id_t id = 0;
            /// The type of the task
            std::type_index type = std::type_index(typeid(void));
            /// The ID of the provider that requested the task
            NUClear::id_t requester_id = 0;
        };

        /// The ID of the active provider
        NUClear::id_t active_provider_id = 0;
        /// The TaskInfo of the active task (zeroed if there is no active task)
        TaskInfo active_task = TaskInfo();

        /// Queue of tasks that are waiting to be run on this provider group
        std::vector<TaskInfo> watchers{};

        /// Whether the provider group is done or not
        bool done = false;
    };

    namespace information {

        template <typename T>
        struct GroupInfoStore {
        private:
            using ThreadStore = NUClear::dsl::store::ThreadStore<std::shared_ptr<const GroupInfo>>;
            using GlobalStore = NUClear::util::TypeMap<T, T, const std::shared_ptr<GroupInfo>>;

        public:
            /**
             * Set the group info for this thread and return a lock that when destroyed will upgrade the data to the
             * global store.
             *
             * @param info the group info to set
             *
             * @return a lock object that once destroyed will upgrade the data to the global store and clear the thread
             * store
             */
            static Lock set(const std::shared_ptr<GroupInfo>& info) {
                auto lock          = Lock(&info, [info](const void*) {
                    GlobalStore::set(info);
                    ThreadStore::value = nullptr;
                });
                ThreadStore::value = &info;
                return lock;
            }

            static std::shared_ptr<const GroupInfo> get() {
                return ThreadStore::value == nullptr ? GlobalStore::get() : *ThreadStore::value;
            }
        };

    }  // namespace information

}  // namespace extension::behaviour

#endif  // EXTENSION_BEHAVIOUR_GROUPINFO_HPP
