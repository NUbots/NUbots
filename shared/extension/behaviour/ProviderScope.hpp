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
#ifndef EXTENSION_BEHAVIOUR_PROVIDER_SCOPE_HPP
#define EXTENSION_BEHAVIOUR_PROVIDER_SCOPE_HPP

#include <nuclear>

#include "RunReason.hpp"
#include "commands.hpp"
#include "unique_tasks.hpp"

namespace extension::behaviour {
    /**
     * This struct holds all the scope information for a provider.
     *
     * While running in a ProviderScope, the behaviour of task emits will be different.
     */
    struct ProviderScope {
        ProviderScope(const std::type_index& provider_type, NUClear::threading::ReactionTask& reaction_task)
            : provider_type(provider_type)
            , reaction_task(reaction_task)
            , previous_scope(std::exchange(current_scope, this)) {}

        ProviderScope(ProviderScope&& other)            = delete;
        ProviderScope& operator=(ProviderScope&& other) = delete;
        ProviderScope(const ProviderScope&)             = delete;
        ProviderScope& operator=(const ProviderScope&)  = delete;

        ~ProviderScope() {
            // Upon desctruction restore the previous scope as the current scope
            current_scope = previous_scope;

            // Sort the task pack so highest priority tasks come first
            // Optional tasks are always considered lower priority than non optional tasks
            std::stable_sort(tasks.begin(), tasks.end(), [](const auto& a, const auto& b) {
                return a.optional == b.optional ? a.priority > b.priority : b.optional;
            });

            // If there are any Continue tasks, do nothing else
            // If there are other tasks with Continue, they will be ignored
            for (const auto& task : tasks) {
                if (task.type == typeid(::extension::behaviour::Continue)) {
                    // Tell the user if there is a Continue task with other tasks
                    if (tasks.size() > 1) {
                        NUClear::log<NUClear::LogLevel::WARN>(
                            "Continue task was emitted with other tasks, the other tasks will be ignored");
                    }

                    // Do nothing else
                    return;
                }
            }

            // Emit all the tasks that were accumulated in this scope
            reaction_task.parent->reactor.emit(std::make_unique<commands::BehaviourTasks>(provider_type,
                                                                                          reaction_task.parent->id,
                                                                                          reaction_task.id,
                                                                                          false,
                                                                                          std::move(tasks)));
        }

        template <typename T>
        static void emit(NUClear::PowerPlant& powerplant,
                         const std::shared_ptr<T>& data,
                         const int priority,
                         const bool optional,
                         const std::string& name) {

            // Create the task object
            auto task = commands::BehaviourTask(typeid(T), data, name, priority, optional);

            // If we are in a scope (running in a provider) accumulate the task to be emitted later
            if (current_scope != nullptr) {
                current_scope->tasks.push_back(std::move(task));
            }
            /// Root tasks emit the task immediately as a single pack
            else {
                // Get the current reaction
                auto current_task = NUClear::threading::ReactionTask::get_current_task();
                auto reaction_id  = current_task != nullptr ? current_task->parent->id : 0;
                auto task_id      = current_task != nullptr ? current_task->id : 0;

                powerplant.emit(std::make_unique<commands::BehaviourTasks>(typeid(commands::RootProvider<T>),
                                                                           reaction_id,
                                                                           task_id,
                                                                           true,
                                                                           std::vector<commands::BehaviourTask>{task}));
            }
        }

    private:
        /// The type of the provider that this scope is for
        std::type_index provider_type;
        /// The reaction task object this scope is for
        NUClear::threading::ReactionTask& reaction_task;
        /// The previous scope that was active before this one so it can be restored
        /// This should only not be nullptr if somehow an Inline emit forces another provider to run while still in
        /// another provider
        ProviderScope* previous_scope = nullptr;
        /// The tasks that have been accumulated in this scope
        std::vector<commands::BehaviourTask> tasks;

        /// Holds the scope object which is currently active
        static thread_local ProviderScope* current_scope;
    };

}  // namespace extension::behaviour

#endif  // EXTENSION_BEHAVIOUR_PROVIDER_SCOPE_HPP
