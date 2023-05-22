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
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_EXTENSION_DIRECTOR_DIRECTORTASK_HPP
#define MODULE_EXTENSION_DIRECTOR_DIRECTORTASK_HPP

#include <map>
#include <typeindex>
#include <vector>

#include "Director.hpp"

#include "extension/Behaviour.hpp"

namespace module::extension::component {

    /**
     * A Task to use in the Director algorithm based on the BehaviourTask, with added state fields for the Director
     * algorithm
     */
    struct DirectorTask {

        /**
         * Construct a new Task object for use in the director algorithm using a given BehaviourTask
         *
         * @param task                  the BehaviourTask to use as a base
         */
        DirectorTask(const ::extension::behaviour::commands::BehaviourTask& task)
            : type(task.type)
            , requester_id(task.requester_id)
            , requester_task_id(task.requester_task_id)
            , data(task.data)
            , name(task.name)
            , priority(task.priority)
            , optional(task.optional)
            , dying(false) {}

        /// The Provider type this task is for
        std::type_index type;
        /// The Provider id of the requester
        uint64_t requester_id;
        /// The reaction task id of the requester (if it is a Provider later a ProviderDone will be emitted)
        uint64_t requester_task_id;
        /// The data for the command, (the data that will be given to the Provider) if null counts as no task
        std::shared_ptr<void> data;
        /// A name for this task to be shown in debugging systems
        std::string name;
        /// The priority with which this task is to be executed
        int priority;
        /// If this task is optional
        bool optional;
        /// Whether this task is currently about to be removed
        bool dying;
    };
}  // namespace module::extension::component

#endif  // MODULE_EXTENSION_DIRECTOR_DIRECTORTASK_HPP
