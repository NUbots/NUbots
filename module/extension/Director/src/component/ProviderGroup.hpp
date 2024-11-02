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

#ifndef MODULE_EXTENSION_DIRECTOR_PROVIDERGROUP_HPP
#define MODULE_EXTENSION_DIRECTOR_PROVIDERGROUP_HPP

#include <map>
#include <typeindex>
#include <vector>

#include "Provider.hpp"

#include "extension/behaviour/GroupInfo.hpp"
#include "extension/behaviour/commands.hpp"

namespace module::extension::component {

    struct ProviderGroup {

        using DataSetter = ::extension::behaviour::commands::ProvideReaction::DataSetter;
        using GroupInfo  = ::extension::behaviour::GroupInfo;

        /// A task list holds a list of tasks
        using TaskList = std::vector<std::shared_ptr<DirectorTask>>;

        ProviderGroup(const std::type_index& type_, const DataSetter& set_data) : type(type_), set_data(set_data) {}

        struct WatchHandle {
            WatchHandle(const std::function<void()>& deleter_) : deleter(deleter_) {}

            WatchHandle(WatchHandle&& other) {
                deleter = std::exchange(other.deleter, std::function<void()>());
            }

            WatchHandle& operator=(WatchHandle&& other) {
                if (this != &other) {
                    deleter = std::exchange(other.deleter, std::function<void()>());
                }
                return *this;
            }

            WatchHandle(const WatchHandle&)            = delete;
            WatchHandle& operator=(const WatchHandle&) = delete;
            ~WatchHandle() {
                if (deleter) {
                    deleter();
                }
            }

            std::function<void()> deleter;
        };

        std::shared_ptr<WatchHandle> add_watcher(const std::shared_ptr<DirectorTask>& task) {
            watchers.push_back(task);

            return std::make_shared<WatchHandle>([this, task] {
                auto it = std::find(watchers.begin(), watchers.end(), task);
                if (it != watchers.end()) {
                    watchers.erase(it);
                }
            });
        }

        GroupInfo get_group_info() {
            GroupInfo group_info;
            group_info.active_provider_id = active_provider != nullptr ? active_provider->id : 0;

            group_info.active_task.id           = active_task != nullptr ? active_task->requester_task_id : 0;
            group_info.active_task.type         = active_task != nullptr ? active_task->type : typeid(void);
            group_info.active_task.requester_id = active_task != nullptr ? active_task->requester_id : 0;

            for (auto& watcher : watchers) {
                group_info.watchers.emplace_back(GroupInfo::TaskInfo{
                    .id           = watcher->requester_task_id,
                    .type         = watcher->type,
                    .requester_id = watcher->requester_id,
                });
            }
            group_info.done = done;

            return group_info;
        }

        /// The type that this provider group manages
        std::type_index type;

        /// The data setter for this provider group
        DataSetter set_data;

        /// List of individual Providers that can service tasks for this type
        std::vector<std::shared_ptr<Provider>> providers;

        /// Stores if the last thing this group did was to emit a done task
        bool done = false;

        /// Stores if this group is in the process of being deleted and should be considered dead for challenges
        bool zombie = false;

        /// The current task that is running on this Provider
        std::shared_ptr<DirectorTask> active_task;
        /// The currently active provider that is executing
        std::shared_ptr<Provider> active_provider;
        /// The tasks who are interested in interacting with this provider. We use it to notify people in priority order
        /// when something changes they might want to know about.
        TaskList watchers;
        /// A list of handles for things we are watching, will be deleted when the list is cleared
        std::vector<std::shared_ptr<WatchHandle>> watch_handles;

        /// The task that is pushing this provider to run in a different way
        std::shared_ptr<DirectorTask> pushing_task;
        /// The provider that the pushing task wants to run on this provider
        std::shared_ptr<Provider> pushed_provider;

        /// List of current subtasks that have been emitted by this provider group
        TaskList subtasks;
    };

}  // namespace module::extension::component

#endif  // MODULE_EXTENSION_DIRECTOR_PROVIDERGROUP_HPP
