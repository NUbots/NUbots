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
         * @param optional_             whether this task is optional or not
         */
        DirectorTask(::extension::behaviour::commands::BehaviourTask task, bool dying_) : dying(dying_) {
            type              = task.type;
            root_type         = task.root_type;
            requester_id      = task.requester_id;
            requester_task_id = task.requester_task_id;
            data              = task.data;
            name              = task.name;
            priority          = task.priority;
            optional          = task.optional;
        }

        /**
         * Construct a new Task object for use in the director algorithm
         *
         * @param type_                 the type that this task is for
         * @param root_type_            a secondary type to use if this is a root task
         * @param requester_id_         the id of the reaction that emitted this task
         * @param requester_task_id_    the task_id of the reaction task that emitted this
         * @param data_                 the task data to be sent to the provider
         * @param name_                 a string name for this task for use in debugging
         * @param priority_             the priority that this task is to run with
         * @param optional_             whether this task is optional or not
         * @param dying_                whether this task is currently about to be removed
         */
        DirectorTask(std::type_index type_,
                     std::type_index root_type_,
                     uint64_t requester_id_,
                     uint64_t requester_task_id_,
                     std::shared_ptr<void> data_,
                     std::string name_,
                     int priority_,
                     bool optional_,
                     bool dying_)
            : type(type_)
            , root_type(root_type_)
            , requester_id(requester_id_)
            , requester_task_id(requester_task_id_)
            , data(data_)
            , name(name_)
            , priority(priority_)
            , optional(optional_)
            , dying(dying_) {}

        /// The Provider type this task is for
        std::type_index type;
        /// A secondary provider type to use if this is a root task
        std::type_index root_type;
        /// The reaction id of the requester (could be the id of a Provider)
        uint64_t requester_id;
        /// The reaction task id of the requester (if it is a Provider later a ProviderDone will be emitted)
        uint64_t requester_task_id;
        /// The data for the command, (the data that will be given to the Provider) if null counts as no task
        std::shared_ptr<void> data;
        /// A name for this task to be shown in debugging systems
        std::string name;
        /// The priority with which this task is to be executed
        int priority = 0;
        /// If this task is optional
        bool optional;
        /// Whether this task is currently about to be removed
        bool dying;
    };
}  // namespace module::extension::component

#endif  // MODULE_EXTENSION_DIRECTOR_DIRECTORTASK_HPP
