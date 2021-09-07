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
#ifndef EXTENSION_BEHAVIOUR_HPP
#define EXTENSION_BEHAVIOUR_HPP

#include <memory>
#include <nuclear>

#include "behaviour/commands.hpp"

namespace extension::behaviour {

    /**
     * A TaskInfo object contains all the state that the Director provides to a running Provider.
     * This task information includes the task data itself, as well as details about why it was executed such as if it
     * was triggered by a done task from one of its tasks. This type works with the NUClear reaction system by providing
     * a dereference operator so that the users can either choose to receive a TaskInfo<T> object or a const T& object
     * in their callback if they don't care about the extra information the director provides.
     *
     * This object also ensures that the task will not run when the Director has no information for this Provider.
     * This can be caused when it should not be running but was triggered by another bind statement (e.g. Every).
     * In this case it has a bool operator that returns false which will stop the task from running.
     *
     * @tparam T the type of task that is stored in this task_info object
     */
    template <typename T>
    struct TaskInfo {

        /**
         * Default constructor, as data will be null NUClear will be told not to execute this task.
         */
        TaskInfo() = default;

        /**
         * Construct a new TaskInfo object for the provided task data
         *
         * @param data_ the data for this task
         */
        TaskInfo(std::shared_ptr<const T> data_) : data(data_) {}

        /**
         * Having a dereference operator allows NUClear to provide either the TaskInfo<T> object or a const T& object
         *
         * @return the data that is contained within this TaskInfo object
         */
        [[nodiscard]] const T& operator*() {
            return *data;
        }

        /**
         * This will control if NUClear should run the task
         *
         * @return true     there is data, NUClear should run the task
         * @return false    there is no data, NUClear should not run the task
         */
        [[nodiscard]] operator bool() {
            return data != nullptr;
        }

        /// The task data, this will be all that is returned if the user asks for a const T& rather than a TaskInfo
        std::shared_ptr<const T> data;
    };


    /**
     * This type is used as a base extension type for the different Provider DSL keywords (Provides, Entering, Leaving)
     * to handle their common code.
     *
     * @tparam T        The type that this Provider services
     * @tparam action   The action type that this Provider is
     */
    template <typename T, commands::ProviderClassification classification>
    struct ProviderBase {

        /**
         * Binds a Provider object, sending it to the Director so it can control its execution.
         *
         * @tparam DSL the NUClear DSL for the on statement
         *
         * @param reaction the reaction object that we are binding the Provider to
         */
        template <typename DSL>
        static inline void bind(const std::shared_ptr<NUClear::threading::Reaction>& reaction) {

            // Tell the Director
            reaction->reactor.powerplant.emit(
                std::make_unique<commands::ProvidesReaction>(reaction, typeid(T), classification));

            // Add our unbinder
            reaction->unbinders.emplace_back([](const NUClear::threading::Reaction& r) {
                r.reactor.emit<NUClear::dsl::word::emit::Direct>(
                    std::make_unique<NUClear::dsl::operation::Unbind<commands::ProvidesReaction>>(r.id));
            });
        }

        /**
         * Gets the information needed by the callbacks of the Provider.
         *
         * @tparam DSL the NUClear DSL for the on statement
         *
         * @param r the reaction object we are getting data for
         *
         * @return the information needed by the on statement
         */
        template <typename DSL>
        static inline TaskInfo<T> get(NUClear::threading::Reaction& /*r*/) {
            // TODO(@TrentHouliston) get the data from the director once it's algorithm is more fleshed out
            return TaskInfo<T>();
        }

        /**
         * Executes once a Provider has finished executing it's reaction so the Director knows which tasks it emitted
         *
         * @tparam DSL the NUClear dsl for the on statement
         *
         * @param task The reaction task object that just finished
         */
        template <typename DSL>
        static inline void postcondition(NUClear::threading::ReactionTask& task) {
            // Take the task id and send it to the Director to let it know that this Provider is done
            task.parent.reactor.emit(std::make_unique<commands::ProviderDone>(task.parent.id, task.id));
        }
    };

    /**
     * This DSL word used to define an entry transition into a Provider.
     * It should normally be coupled with a Causing DSL word.
     * When coupled with a Causing DSL word and a Provider that has a When condition, this entry will run in place of
     * the Provider until the When condition is fulfilled.
     * If it is used without a Causing DSL word it will run once as a Provider before the actual Provider is executed.
     *
     * @tparam T the Provider type that this transition function provides for
     */
    template <typename T>
    struct Entering : public ProviderBase<T, commands::ProviderClassification::ENTERING> {};

    /**
     * This DSL word is used to define a leaving transition from a Provider.
     * It should normally be combined with a Causing DSL word.
     * In this case when another Provider needs control and uses a When condition, this will run until that
     * condition is fulfilled. A Leaving condition will be preferred over an Entering condition that provides the
     * same Causing statement (the system will rather a module cleans up after itself than ask another module to do
     * so).
     *
     * @tparam T the Provider type that this transition function provides for
     */
    template <typename T>
    struct Leaving : public ProviderBase<T, commands::ProviderClassification::LEAVING> {};

    /**
     * This DSL word is used to define a Provider for a type.
     * It will execute in the same way as an on<Trigger<T>> statement except that when it executes will be
     * determined by the Director. This ensures that it will only run when it has permission to run and will
     * transition between Providers in a sensible way.
     *
     * @tparam T the Provider type that this transition function provides for
     */
    template <typename T>
    struct Provides : public ProviderBase<T, commands::ProviderClassification::NORMAL> {};

    /**
     * The When DSL word is used to limit access to a Provider unless a condition is true.
     * This will prevent a Provider from running if the state is false.
     * However if there is a Entering<T> or Leaving reaction that has a Causing relationship for this When, it will
     * be executed to ensure that the Provider can run.
     *
     * @tparam State    the smart enum that is being mointored for the when condition
     * @tparam expr     the function used for the comparison (e.g. std::less)
     * @tparam value    the value that the when condition is looking for
     */
    template <typename State, template <typename> class expr, enum State::Value value>
    struct When {

        /**
         * Bind a when expression in the Director.
         *
         * @tparam DSL the DSL from NUClear
         *
         * @param reaction the reaction that is having this when condition bound to it
         */
        template <typename DSL>
        static inline void bind(const std::shared_ptr<NUClear::threading::Reaction>& reaction) {

            // Tell the director about this when condition
            reaction->reactor.powerplant.emit(std::make_unique<commands::WhenExpression>(
                reaction,
                // typeindex of the enum value
                typeid(State),
                // Function that uses expr to determine if the passed value v is valid
                [](const int& v) -> bool { return expr<int>()(v, value); },
                // Function that uses get to get the current state of the reaction
                []() -> int {
                    // Check if there is cached data, and if not throw an exception
                    auto ptr = NUClear::dsl::operation::CacheGet<State>::get();
                    if (ptr == nullptr) {
                        throw std::runtime_error("The state requested has not been emitted yet");
                    }
                    return static_cast<int>(*ptr);
                },
                // Binder function that lets a reactor bind a function that is called when the state changes
                [](NUClear::Reactor& reactor,
                   const std::function<void(const int&)>& fn) -> NUClear::threading::ReactionHandle {
                    return reactor.on<NUClear::dsl::word::Trigger<State>>().then(
                        [fn](const State& s) { fn(static_cast<int>(s)); });
                }));
        }
    };

    /**
     * This DSL word is used to create a promise that at some point when running this reaction the state indicated
     * by T will be true. It may not be on any particular run of the Provider, in which case it will continue to run
     * that reaction when new tasks arrive until it is true, or a change elsewhere changes which Provider to run.
     *
     * @tparam State    the smart enum that this causing condition is going to manipulate
     * @tparam value    the value that will result when the causing is successful
     */
    template <typename State, enum State::Value value>
    struct Causing {

        /**
         * Bind a causing expression in the Director.
         *
         * @tparam DSL the DSL from NUClear
         *
         * @param reaction the reaction that is having this when condition bound to it
         */
        template <typename DSL>
        static inline void bind(const std::shared_ptr<NUClear::threading::Reaction>& reaction) {
            // Tell the director
            reaction->reactor.powerplant.emit(
                std::make_unique<commands::CausingExpression>(reaction, typeid(State), value));
        }
    };

    /**
     * A Task to be executed by the director and a priority with which to execute this task.
     * Higher priority tasks are preferred over lower priority tasks.
     *
     * There are two different types of tasks that can be created using this emit, root level tasks and subtasks.
     *
     * Root level tasks:
     * These are created when a reaction that is not a Provider (not a Provides, Entering or Leaving) emits the
     * task. These tasks form the root of the execution tree and their needs will be met on a highest priority first
     * basis. These tasks will persist until the Provider that they use emits a done task, or the task is re-emitted
     * with a priority of 0.
     *
     * Subtasks:
     * These are created when a Provider task emits a task to complete. These tasks must be emitted each time that
     * reaction is run to persist as if that reaction runs again and does not emit these tasks they will be
     * cancelled. Within these tasks the priority is used to break ties between two subtasks that share the same
     * root task. In these cases the subtask that requested the Provider with the highest priority will have its
     * task executed. The other subtask will be blocked until the active task is no longer in the call queue.
     *
     * If a subtask is emitted with optional then it is compared differently to other tasks when it comes to priority.
     * This task and all its descendants will be considered optional. If it is compared to a task that does not have
     * optional in it's parentage, the non optional task will win. However, descendants of this task that are not
     * optional will compare to each other as normal. If two tasks both have optional in their parentage they will be
     * compared as normal.
     *
     * @tparam T the Provider type that this task is for
     */
    template <typename T>
    struct Task {

        /**
         * Emits a new task for the Director to handle.
         *
         * @param powerplant the powerplant context provided by NUClear
         * @param data       the data element of the task
         * @param name       a string identifier to help with debugging
         * @param priority   the priority that this task is to be executed with
         * @param optional   if this task is an optional task and can be taken over by lower priority non optional tasks
         */
        static void emit(NUClear::PowerPlant& powerplant,
                         std::shared_ptr<T> data,
                         const std::string& name = "",
                         const int& priority     = 1,
                         const bool& optional    = false) {

            // Work out who is sending the task so we can determine if it's a subtask
            const auto* task     = NUClear::threading::ReactionTask::get_current_task();
            uint64_t reaction_id = (task != nullptr) ? task->parent.id : -1;
            uint64_t task_id     = (task != nullptr) ? task->id : -1;

            NUClear::dsl::word::emit::Direct<T>::emit(powerplant,
                                                      std::make_shared<commands::DirectorTask>(typeid(T),
                                                                                               reaction_id,
                                                                                               task_id,
                                                                                               data,
                                                                                               name,
                                                                                               priority,
                                                                                               optional));
        }
    };

    /**
     * This is a special task that should be emitted when a Provider finishes the task it was given.
     * When this is emitted the director will re-execute the Provider which caused this task to run.
     *
     * ```
     * emit<Task>(std::make_unique<Done>());
     * ```
     */
    struct Done {};

}  // namespace extension::behaviour

#endif  // EXTENSION_BEHAVIOUR_HPP
