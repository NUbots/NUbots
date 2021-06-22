#ifndef EXTENSION_BEHAVIOUR_HPP
#define EXTENSION_BEHAVIOUR_HPP

#include <memory>
#include <nuclear>

namespace extension::behaviour {

    // This namespace holds the messages and communication elements that operate between the Director module and the DSL
    // the users of the behaviour system.
    namespace commands {

        /**
         * The type of action that the provider performs
         */
        enum class ProviderAction {
            /// NORMAL providers are the usual provideres that execute in a loop if needed
            NORMAL,
            /// ENTERING providers are the first point of entry for a type before the main NORMAL reaction runs
            ENTERING,
            /// LEAVING providers are executed as a module is relinquishing control
            LEAVING,
        };

        /**
         * Message used to tell the Director about a new Provides reaction
         */
        struct ProvidesReaction {
            ProvidesReaction(const std::shared_ptr<NUClear::threading::Reaction>& reaction,
                             const std::type_index& type,
                             const ProviderAction& action)
                : reaction(reaction), type(type), action(action) {}

            /// The reaction for this provider
            std::shared_ptr<NUClear::threading::Reaction> reaction;
            /// The data type that this provider services
            std::type_index type;
            /// The action type that this provider is using (normal, entering, leaving)
            ProviderAction action;
        };

        /**
         * Message used to tell the Director about a new When condition on a provider
         */
        struct WhenExpression {
            WhenExpression(const std::shared_ptr<NUClear::threading::Reaction>& reaction,
                           const std::type_index& type,
                           bool (*validator)(const int&),
                           int (*current)())
                : reaction(reaction), type(type), validator(validator), current(current) {}

            /// The provider reaction that is contingent on this when condition
            std::shared_ptr<NUClear::threading::Reaction> reaction;
            /// The enum type that this when expression is looking at
            std::type_index type;
            /// Function to determine if the passed state is valid
            bool (*validator)(const int&);
            /// Function to get the current state from the global cache
            int (*current)();
            /// Function to bind a reaction to monitor when this state changes
            NUClear::threading::ReactionHandle (*binder)(NUClear::Reactor&, std::function<void(const int&)>);
        };

        /**
         * Message used to tell the director about a new Causing relationship for this provider
         */
        struct CausingExpression {
            CausingExpression(const std::shared_ptr<NUClear::threading::Reaction>& reaction,
                              const std::type_index& type,
                              const int& resulting_state)
                : reaction(reaction), type(type), resulting_state(resulting_state) {}
            /// The provider reaction that will cause this state
            std::shared_ptr<NUClear::threading::Reaction> reaction;
            /// The enum type that this when expression claiming to manipulate
            std::type_index type;
            /// The resulting state this provider is claiming to provide if it is executed
            int resulting_state;
        };

        /**
         * Message to tell the Director that a provider task has finished executing
         */
        struct ProviderDone {
            ProviderDone(const uint64_t& requester_id, const uint64_t& requester_task_id)
                : requester_id(requester_id), requester_task_id(requester_task_id) {}
            /// The reaction_id of the provider that finished
            uint64_t requester_id;
            /// The specific task_id of the provider that finished
            uint64_t requester_task_id;
        };

        /**
         * A task message that is to be sent to the Director for execution
         */
        struct DirectorTask {
            DirectorTask(std::type_index type,
                         uint64_t requester_id,
                         uint64_t requester_task_id,
                         std::shared_ptr<void> command,
                         int priority,
                         const std::string& name)
                : type(type)
                , requester_id(requester_id)
                , requester_task_id(requester_task_id)
                , command(command)
                , priority(priority)
                , name(name) {}

            /// The provider type this director task is for
            std::type_index type;
            /// The reaction id of the requester (could be the id of a provider)
            uint64_t requester_id;
            /// The reaction task id of the requester (if it is a provider later a ProviderDone will be emitted)
            uint64_t requester_task_id;
            /// The data for the command, (the data that will be given to the provider)
            std::shared_ptr<void> command;
            /// The priority with which this task is to be executed
            int priority;
            /// A name for this task to be shown in debugging systems
            std::string name;
        };

        /**
         * This type is used as a base type for the different provider DSL keywords (Provides, Entering, Leaving) to
         * handle their common code.
         *
         * @tparam T        The type that this provider services
         * @tparam action   The action type that this provider is
         */
        template <typename T, ProviderAction action>
        struct ProviderBase {

            template <typename DSL>
            static inline void bind(const std::shared_ptr<NUClear::threading::Reaction>& reaction) {

                // Tell the director
                reaction->reactor.powerplant.emit(std::make_unique<ProvidesReaction>(reaction, typeid(T), action));

                // Add our unbinder
                reaction->unbinders.emplace_back([](const NUClear::threading::Reaction& r) {
                    r.reactor.emit<NUClear::dsl::word::emit::Direct>(
                        std::make_unique<NUClear::dsl::operation::Unbind<commands::ProvidesReaction>>(r.id));
                });
            }

            template <typename DSL>
            static inline std::shared_ptr<T> get(NUClear::threading::Reaction& r) {

                // TODO here we need to get task information from the director
                // TODO using r.id we can query the director if this provider is supposed to be running right now and
                // also get the data for this provider
                // If this provider is not supposed to be running right now we return a nullptr
                // By returning nullptr that evaluates to something "falsy" it will prevent this reaction from running
                // (unless the provider has an optional wrapper around it but then it's the users job to deal with that)
                return nullptr;
            }

            template <typename DSL>
            static inline void postcondition(NUClear::threading::ReactionTask& task) {
                // Take the task id and send it to the director to let it know that this provider is done
                task.parent.reactor.emit(std::make_unique<ProviderDone>(task.parent.id, task.id));
            }
        };
    }  // namespace commands

    /**
     * This DSL word used to define an entry transition into a provider.
     * It should normally be coupled with a Causing DSL word.
     * When coupled with a Causing DSL word and a provider that has a When condition, this entry will run in place of
     * the provider until the When condition is fulfilled.
     * If it is used without a Causing DSL word it will run once as a provider before the actual provider is executed.
     *
     * @tparam T the provider type that this transition function provides for
     */
    template <typename T>
    struct Entering : public commands::ProviderBase<T, commands::ProviderAction::ENTERING> {};

    /**
     * This DSL word is used to define a leaving transition from a provider.
     * It should normally be combined with a Causing DSL word.
     * In this case when another provider needs control and uses a When condition, this will run until that
     * condition is fulfilled. A Leaving condition will be preferred over an Entering condition that provides the
     * same Causing statement (the system will rather a module cleans up after itself than ask another module to do
     * so).
     *
     * @tparam T the provider type that this transition function provides for
     */
    template <typename T>
    struct Leaving : public commands::ProviderBase<T, commands::ProviderAction::LEAVING> {};

    /**
     * This DSL word is used to define a provider for a type.
     * It will execute in the same way as an on<Trigger<T>> statement except that when it executes will be
     * determined by the Director. This ensures that it will only run when it has permission to run and will
     * transition between providers in a sensible way.
     *
     * @tparam T the provider type that this transition function provides for
     */
    template <typename T>
    struct Provides : public commands::ProviderBase<T, commands::ProviderAction::NORMAL> {};

    /**
     * The When DSL word is used to limit access to a provider unless a condition is true.
     * This will prevent a provider from running if the state is false.
     * However if there is a Entering<T> or Leaving reaction that has a Causing relationship for this When, it will
     * be executed to ensure that the Provider can run.
     *
     * @tparam T the condition expression that must be true in order to execute this task.
     */
    template <typename State, template <typename> class expr, enum State::Value value>
    struct When {

        template <typename DSL>
        static inline void bind(const std::shared_ptr<NUClear::threading::Reaction>& reaction) {

            // Tell the director about this when condition
            reaction->reactor.powerplant.emit(std::make_unique<commands::WhenExpression>(
                reaction,
                typeid(State),
                [](const int& v) { return expr<int>()(v, value); },
                []() { return NUClear::dsl::operation::CacheGet<State>::get(); },
                [](NUClear::Reactor& reactor,
                   std::function<void(const int&)> fn) -> NUClear::threading::ReactionHandle {
                    return reactor.on<NUClear::dsl::word::Trigger<State>>().then(
                        [fn](const State& s) { fn(static_cast<int>(s)); });
                }));
        }
    };

    /**
     * This DSL word is used to create a promise that at some point when running this reaction the state indicated
     * by T will be true. It may not be on any particular run of the reaction, in which case it will continue to run
     * that reaction until it is true, or another graph jump becomes necessary.
     *
     * @tparam T the condition expression that this reaction will make true before leaving.
     */
    template <typename State, enum State::Value value>
    struct Causing {

        template <typename DSL>
        static inline void bind(const std::shared_ptr<NUClear::threading::Reaction>& reaction) {
            // Tell the director
            reaction->reactor.powerplant.emit(
                std::make_unique<commands::CausingExpression>(reaction, typeid(State), value));
        }
    };

    /**
     * A Task to be executed by the director, and a priority with which to execute this task.
     *
     * There are two different types of tasks that can be created using this emit, root level tasks and subtasks.
     *
     * Root level tasks:
     * These are created when a reaction that is not a provider (not a Provides, Entering or Leaving) emits the
     * task. These tasks form the root of the execution tree and their needs will be met on a highest priority first
     * basis. These tasks will persist until the provider that they use emits a done task, or the task is re-emitted
     * with a priority of 0.
     *
     * Subtasks:
     * These are created when a provider task emits a task to complete. These tasks must be emitted each time that
     * reaction is run to persist as if that reaction runs again and does not emit these tasks they will be
     * cancelled. Within these tasks the priority is used to break ties between two subtasks that share the same
     * root task. In these cases the subtask that requested the provider with the highest priority will have its
     * task executed. The other subtask will be blocked until that task is no longer in the call queue.
     *
     * If a subtask is emitted with a negative priority then it will be considered an "optional" task for this tree.
     * In this case any positive priority from another root task will be preferred over this task and can kick it and
     * its descendents off the behaviour tree.
     *
     * @tparam T the provider type that this task is for
     */
    template <typename T>
    struct Task {

        /**
         * This is a special task that should be emitted when a provider finishes the task it was given.
         * When this is emitted the director will re-execute the provider which caused this task to run.
         *
         * ```
         * emit<Task>(std::make_unique<Task::Done>());
         * ```
         */
        struct Done {};

        static void emit(NUClear::PowerPlant& powerplant,
                         std::shared_ptr<T> data,
                         int priority            = 1,
                         const std::string& name = "") {

            // Work out who is sending the task so we can determine if it's a subtask
            auto* task           = NUClear::threading::ReactionTask::get_current_task();
            uint64_t reaction_id = task ? task->parent.id : -1;
            uint64_t task_id     = task ? task->id : -1;

            NUClear::dsl::word::emit::Direct<T>::emit(
                powerplant,
                std::make_shared<commands::DirectorTask>(typeid(T), reaction_id, task_id, data, priority, name));
        }
    };

}  // namespace extension::behaviour

#endif  // EXTENSION_BEHAVIOUR_H
