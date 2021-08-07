/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2021 NUbots <nubots@nubots.net>
 */
#ifndef MODULE_EXTENSION_DIRECTOR_PROVIDERGROUP_HPP
#define MODULE_EXTENSION_DIRECTOR_PROVIDERGROUP_HPP

#include <map>
#include <typeindex>
#include <vector>

#include "Provider.hpp"

namespace module::extension::provider {

    struct ProviderGroup {

        /// A task queue holds tasks in a provider that are waiting to be executed by that group
        using TaskQueue = std::vector<std::shared_ptr<const ::extension::behaviour::commands::DirectorTask>>;
        /// A task pack is the result of a set of tasks emitted by a provider that should be run together
        using TaskPack = std::vector<std::shared_ptr<const ::extension::behaviour::commands::DirectorTask>>;

        /**
         * The current state of the Provider group
         */
        enum State {
            /// This Provider group currently isn't doing anything
            IDLE,
            /// This Provider group is currently in an "Entering" state
            ENTERING,
            /// This Provider group is currently in a "Normal" state
            NORMAL,
            /// This Provider group is currently in a "Leaving" state
            LEAVING,
            /// This Provider group is not running because it has requirements that are not met, in order to meet
            /// these requirements it is taking control of another currently running Provider group
            PROXYING
        };

        /// The current state of this Provider group
        State state = State::IDLE;
        /// List of individual Providers that can service tasks for this type
        std::vector<std::shared_ptr<Provider>> providers;
        /// The current task that is running on this Provider
        std::shared_ptr<const ::extension::behaviour::commands::DirectorTask> active_task;
        /// The queue of tasks waiting to run if the situation changes
        TaskQueue task_queue;
        /// List of current subtasks that have been emitted by this provider group
        TaskPack subtasks;
    };

}  // namespace module::extension::provider

#endif  // MODULE_EXTENSION_DIRECTOR_PROVIDERGROUP_HPP
