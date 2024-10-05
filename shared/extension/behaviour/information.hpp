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
     * Provides information about how and why this provider was executed.
     */
    struct RunInfo {
        enum RunReason {
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

        /// The reason we executed
        RunReason run_reason;
    };

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

}  // namespace extension::behaviour

namespace extension::behaviour::information {

    using RunInfoStore =        NUClear::dsl::store::ThreadStore<RunInfo>;

    template<typename GroupType>
    using LocalGroupInfoStore =      NUClear::dsl::store::ThreadStore<GroupInfo<GroupType>>;
    template <typename GroupType>
    using GlobalGroupInfoStore = NUClear::util::TypeMap<ProviderOrSomething, GroupType, GroupInfo<GroupType>>;

    inline std::shared_ptr<RunInfo> get_run_info() {
        return RunInfoStore::value == nullptr ? std::make_shared<RunInfo>({RunInfo::OTHER_TRIGGER})
                                              : std::make_shared<RunInfo>(*RunInfoStore::value);
    }

    template <typename GroupType>
    inline GroupInfo get_group_info() {
         return <std::shared_ptr<T>>::value == nullptr
                           ? GlobalGroupInfoStore<GroupType>::get()
                           : *LocalGroupInfoStore<GroupType>::value;
    }

    template <typename GroupType>
    inline std::shared_ptr<const GroupType> get_task_data(const uint64_t& reaction_id) {
        // Something similar in here but the store probably needs to also include which provider it's for so it can return nullptr when it's not the right one
    }

}  // namespace extension::behaviour::information

#endif  // EXTENSION_BEHAVIOUR_INFORMATIONSOURCE_HPP
