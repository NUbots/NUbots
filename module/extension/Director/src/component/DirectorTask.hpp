/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
