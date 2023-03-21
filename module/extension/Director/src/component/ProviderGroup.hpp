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
 * Copyright 2022 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_EXTENSION_DIRECTOR_PROVIDERGROUP_HPP
#define MODULE_EXTENSION_DIRECTOR_PROVIDERGROUP_HPP

#include <map>
#include <typeindex>
#include <vector>

#include "Provider.hpp"

namespace module::extension::component {

    struct ProviderGroup {

        /// A task list holds a list of tasks
        using TaskList = std::vector<std::shared_ptr<DirectorTask>>;

        ProviderGroup(const std::type_index& type_) : type(type_) {}

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

        /// The type that this provider group manages
        std::type_index type;

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
