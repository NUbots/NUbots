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

#ifndef EXTENSION_BEHAVIOUR_HPP
#define EXTENSION_BEHAVIOUR_HPP

#include <memory>
#include <nuclear>

#include "behaviour/InformationSource.hpp"
#include "behaviour/commands.hpp"

namespace extension::behaviour {

    /**
     * This type is used as a base extension type for the different Provider DSL keywords (Start, Stop, Provide)
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
            reaction->reactor.powerplant.emit<NUClear::dsl::word::emit::Direct>(
                std::make_unique<commands::ProvideReaction>(reaction, typeid(T), classification));

            // Add our unbinder
            reaction->unbinders.emplace_back([](const NUClear::threading::Reaction& r) {
                r.reactor.emit<NUClear::dsl::word::emit::Direct>(
                    std::make_unique<NUClear::dsl::operation::Unbind<commands::ProvideReaction>>(r.id));
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
        static inline std::tuple<std::shared_ptr<const T>, std::shared_ptr<const RunInfo>> get(
            NUClear::threading::Reaction& r) {

            auto run_info = std::make_shared<RunInfo>(information::InformationSource::get_run_info(r.id));
            auto run_data = std::static_pointer_cast<T>(information::InformationSource::get_task_data(r.id));
            return std::make_tuple(run_data, run_info);
        }

        /**
         * Executes once a Provider has finished executing its reaction so the Director knows which tasks it emitted
         *
         * @tparam DSL the NUClear dsl for the on statement
         *
         * @param task The reaction task object that just finished
         */
        template <typename DSL>
        static inline void postcondition(NUClear::threading::ReactionTask& task) {
            // Take the task id and send it to the Director to let it know that this Provider is done
            task.parent.reactor.emit<NUClear::dsl::word::emit::Direct>(
                std::make_unique<commands::ProviderDone>(task.parent.id, task.id));
        }
    };

    /**
     * Define a Provider for a type.
     * It will execute in the same way as an on<Trigger<T>> statement except that when it executes will be
     * determined by the Director. This ensures that it will only run when it has permission to run and will
     * transition between Providers in a sensible way.
     *
     * @tparam T the Provider type that this function provides for
     */
    template <typename T>
    struct Provide : public ProviderBase<T, commands::ProviderClassification::PROVIDE> {};

    /**
     * Define a Start provider for a type
     * It will execute before a task is first provided to the provider group.
     * It is not allowed to have any other words such as when/causing/needs and must not emit any tasks.
     *
     * @tparam T the Provider type that this transition function provides for
     */
    template <typename T>
    struct Start : public ProviderBase<T, commands::ProviderClassification::START> {};

    /**
     * Define a Stop provider for a type
     * It will execute after a task is no longer provided to the provider group.
     * It is not allowed to have any other words such as when/causing/needs and must not emit any tasks.
     *
     * @tparam T the Provider type that this transition function provides for
     */
    template <typename T>
    struct Stop : public ProviderBase<T, commands::ProviderClassification::STOP> {};

    /**
     * Limit access to this Provider unless a condition is true.
     * This will prevent a Provider from running if the state is false.
     * However if there is a `Provide` reaction that has a Causing relationship for this When, then
     * depending on the priority of this task, it can force a change in which provider will be run
     *
     * @tparam State    the smart enum that is being monitored for the when condition
     * @tparam expr     the function used for the comparison (e.g. std::less)
     * @tparam value    the value that the when condition is looking for
     */
    template <typename State, template <typename> class expr, State::Value value>
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
            reaction->reactor.emit<NUClear::dsl::word::emit::Direct>(std::make_unique<commands::WhenExpression>(
                reaction,
                // typeindex of the enum value
                typeid(State),
                // Function that uses expr to determine if the passed value v is valid
                [](const int& v) -> bool { return expr<int>()(v, value); },
                // Function that uses get to get the current state of the condition
                [reaction]() -> int {
                    // Check if there is cached data, and if not throw an exception
                    auto ptr = NUClear::dsl::operation::CacheGet<State>::template get<DSL>(*reaction);
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
     * Create a promise that at some point when running this reaction the state indicated by `State` will be `value`. It
     * may not be on any particular run of the Provider, in which case it will continue to run that reaction when new
     * tasks arrive until it is true, or a change elsewhere changes which Provider to run.
     *
     * @tparam State    the smart enum that this causing condition is going to manipulate
     * @tparam value    the value that will result when the causing is successful
     */
    template <typename State, State::Value value>
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
            reaction->reactor.emit<NUClear::dsl::word::emit::Direct>(
                std::make_unique<commands::CausingExpression>(reaction, typeid(State), value));
        }
    };

    /**
     * Adds a Uses object to the callback to allow access to information from the context of this provider.
     *
     * The uses object provides information about what would happen if you were to emit a specific task from this
     * provider as well as what has happened in previous calls.
     *
     * It has two main uses
     *   It allows you to see what would happen if you emitted a task (run, block proxy, etc)
     *   It allows you to see what has happened in previous calls, e.g. are we re-running because our previous task
     *   emitted that it was done
     *
     * @tparam Provider the type of the provider this uses is for
     */
    template <typename Provider>
    struct Uses {
        GroupInfo::RunState run_state;
        bool done;

        template <typename DSL>
        static inline std::shared_ptr<Uses<Provider>> get(NUClear::threading::Reaction& r) {

            auto group_info = information::InformationSource::get_group_info(r.id,
                                                                             typeid(Provider),
                                                                             typeid(commands::RootType<Provider>));

            auto data = std::make_shared<Uses<Provider>>();

            data->run_state = group_info.run_state;
            data->done      = group_info.done;

            return data;
        }
    };

    /**
     * Create a Needs relationship between this provider and the provider specified by `T`.
     *
     * A needs relationship ensures that this provider will only run if it is able to run the provider specified by `T`.
     * This relationship operates recursively, as if the provider specified by `T` needs another provider, this provider
     * will only run if it will be able to obtain those providers as well.
     *
     * @tparam Provider the provider that this provider needs
     */
    template <typename Provider>
    struct Needs : public Uses<Provider> {  // Needs implies uses

        /**
         * Bind a needs expression in the Director.
         *
         * @tparam DSL the DSL from NUClear
         *
         * @param reaction the reaction that is having this needs condition bound to it
         */
        template <typename DSL>
        static inline void bind(const std::shared_ptr<NUClear::threading::Reaction>& reaction) {
            reaction->reactor.emit<NUClear::dsl::word::emit::Direct>(
                std::make_unique<commands::NeedsExpression>(reaction, typeid(Provider)));
        }
    };

    /**
     * A Task to be executed by the director and a priority with which to execute this task.
     * Higher priority tasks are preferred over lower priority tasks.
     *
     * There are two different types of tasks that can be created using this emit, root level tasks and subtasks.
     *
     * Root level tasks:
     * These are created when a reaction that is not a Provider emits the task. These tasks form the root of the
     * execution tree and their needs will be met on a highest priority first basis. These tasks will persist until the
     * Provider that they use emits a done task, or the task is re-emitted from anywhere with nullptr as the data. They
     * will also be overridden if a new task is emitted anywhere in the system even if it is at a lower priority.
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
     * optional in its parentage, the non optional task will win. However, descendants of this task that are not
     * optional will compare to each other as normal. If two tasks both have optional in their parentage they will be
     * compared as normal.
     *
     * @tparam T the Provider type that this task is for
     */
    template <typename T>
    struct Task {

        /**
         * Emits a new task for the behaviour system to handle.
         *
         * @param powerplant the powerplant context provided by NUClear
         * @param data       the data element of the task
         * @param name       a string identifier to help with debugging
         * @param priority   the priority that this task is to be executed with
         * @param optional   if this task is an optional task and can be taken over by lower priority non optional tasks
         */
        static void emit(NUClear::PowerPlant& powerplant,
                         std::shared_ptr<T> data,
                         const int& priority     = 0,
                         const bool& optional    = false,
                         const std::string& name = "") {

            // Work out who is sending the task so we can determine if it's a subtask
            const auto* task     = NUClear::threading::ReactionTask::get_current_task();
            uint64_t reaction_id = (task != nullptr) ? task->parent.id : -1;
            uint64_t task_id     = (task != nullptr) ? task->id : -1;

            NUClear::dsl::word::emit::Direct<commands::BehaviourTask>::emit(
                powerplant,
                std::make_shared<commands::BehaviourTask>(typeid(T),
                                                          typeid(commands::RootType<T>),
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

    /**
     * This is a special task that should be emitted when a Provider doesn't want to change what it is doing.
     * When this is emitted the director will just continue with whatever was previously emitted by this provider.
     *
     * ```
     * emit<Task>(std::make_unique<Idle>());
     * ```
     */
    struct Idle {};

    /**
     * A reactor subtype that can be used when making a behaviour reactor.
     *
     * It exposes the additional DSL words that are added by the Behaviour DSL so they can be used without the need for
     * using statements
     */
    class BehaviourReactor : public NUClear::Reactor {
    public:
        using NUClear::Reactor::Reactor;

    protected:
        template <typename T>
        using Provide = ::extension::behaviour::Provide<T>;
        template <typename T>
        using Start = ::extension::behaviour::Start<T>;
        template <typename T>
        using Stop = ::extension::behaviour::Stop<T>;
        template <typename State, template <typename> class expr, State::Value value>
        using When = ::extension::behaviour::When<State, expr, value>;
        template <typename State, State::Value value>
        using Causing = ::extension::behaviour::Causing<State, value>;
        template <typename T>
        using Needs = ::extension::behaviour::Needs<T>;
        template <typename T>
        using Uses = ::extension::behaviour::Uses<T>;
        template <typename T>
        using Task      = ::extension::behaviour::Task<T>;
        using RunInfo   = ::extension::behaviour::RunInfo;
        using GroupInfo = ::extension::behaviour::GroupInfo;
        using Done      = ::extension::behaviour::Done;
        using Idle      = ::extension::behaviour::Idle;
    };

}  // namespace extension::behaviour

#endif  // EXTENSION_BEHAVIOUR_HPP
