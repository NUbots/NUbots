#ifndef EXTENSION_BEHAVIOUR_H
#define EXTENSION_BEHAVIOUR_H

namespace extension {
namespace behaviour {

    namespace commands {
        struct EnteringReaction {
            EnteringReaction(const std::shared_ptr<NUClear::threading::Reaction>& reaction) : reaction(reaction) {}
            std::shared_ptr<threading::Reaction> reaction;
        };

        struct LeavingReaction {
            LeavingReaction(const std::shared_ptr<NUClear::threading::Reaction>& reaction) : reaction(reaction) {}
            std::shared_ptr<threading::Reaction> reaction;
        };

        struct ProvidesReaction {
            ProvidesReaction(const std::shared_ptr<NUClear::threading::Reaction>& reaction) : reaction(reaction) {}
            std::shared_ptr<threading::Reaction> reaction;
        };

        struct WhenExpression {
            WhenExpression(const std::shared_ptr<NUClear::threading::Reaction>& reaction,
                           std::function<void()> expression)
                : reaction(reaction), expression(expression) {}
            std::shared_ptr<threading::Reaction> reaction;
            std::function<void()> expression;
        };

        struct CausingExpression {
            CausingExpression(const std::shared_ptr<NUClear::threading::Reaction>& reaction,
                              std::function<void()> expression)
                : reaction(reaction), expression(expression) {}
            std::shared_ptr<threading::Reaction> reaction;
            std::function<void()> expression;
        };

        struct DirectorTask {
            DirectorTask(std::type_index type,
                         uint64_t cause_id,
                         std::shared_ptr<void> command,
                         int priority,
                         const std::string& name)
                : type(type), cause_id(cause_id), command(command), priority(priority), name(name) {}
            std::type_index type;
            uint64_t cause_id;
            std::shared_ptr<void> command;
            int priority;
            std::string name;
        };
    }  // namespace commands

    /**
     * This DSL word used to define an entry transition into a provider.
     * It should normally be coupled with a Causing DSL word.
     * When coupled with a Causing DSL word and a provider that has a When condition,
     * this entry will run in place of the provider until the When condition is fulfilled.
     * If it is used without a Causing DSL word it will run once as a provider before the actual provider is executed.
     *
     * @tparam T the provider type that this transition function provides for
     */
    template <typename T>
    struct Entering {

        template <typename DSL>
        static inline void bind(const std::shared_ptr<threading::Reaction>& reaction) {

            // Tell the director
            reaction->reactor.powerplant.emit(std::make_unique<commands::EnteringReaction>(reaction));

            // Add our unbinder
            reaction->unbinders.emplace_back([](const threading::Reaction& r) {
                r.reactor.emit<word::emit::Direct>(
                    std::make_unique<operation::Unbind<commands::EnteringReaction>>(r.id));
            });
        }
    };

    /**
     * This DSL word is used to define a leaving transition from a provider.
     * It should normally be combined with a Causing DSL word.
     * In this case when another provider needs control and uses a When condition, this will run until that condition is
     * fulfilled.
     * A Leaving condition will be preferred over an Entering condition that provides the same Causing statement.
     *
     * @tparam T the provider type that this transition function provides for
     */
    template <typename T>
    struct Leaving {

        template <typename DSL>
        static inline void bind(const std::shared_ptr<threading::Reaction>& reaction) {

            // Tell the director
            reaction->reactor.powerplant.emit(std::make_unique<commands::LeavingReaction>(reaction));

            // Add our unbinder
            reaction->unbinders.emplace_back([](const threading::Reaction& r) {
                r.reactor.emit<word::emit::Direct>(
                    std::make_unique<operation::Unbind<commands::LeavingReaction>>(r.id));
            });
        }
    };

    /**
     * This DSL word is used to define a provider for a type.
     * It will execute in the same way as an on<Trigger<T>> statement except that when it executes will be determined
     * by the Director. This ensures that it will only run when it has permission to run and will transition between
     * providers in a sensible way.
     *
     * @tparam T the provider type that this transition function provides for
     */
    template <typename T>
    struct Provides {

        template <typename DSL>
        static inline void bind(const std::shared_ptr<threading::Reaction>& reaction) {

            // Tell the director
            reaction->reactor.powerplant.emit(std::make_unique<commands::ProvidesReaction>(reaction));

            // Add our unbinder
            reaction->unbinders.emplace_back([](const threading::Reaction& r) {
                r.reactor.emit<word::emit::Direct>(
                    std::make_unique<operation::Unbind<commands::ProvidesReaction>>(r.id));
            });
        }

        template <typename DSL>
        static inline std::shared_ptr<T> get(threading::Reaction& t) {

            // TODO get the instance of the task command data from the director
            return nullptr;
        }
    };

    /**
     * The When DSL word is used to limit access to a provider unless a condition is true.
     * This will prevent a provider from running if the state is false.
     * However if there is a Entering<T> or Leaving reaction that has a Causing relationship for this When, it will be
     * executed to ensure that the Provider can run.
     *
     * @tparam T the condition expression that must be true in order to execute this task.
     */
    template <typename T>
    struct When {

        template <typename DSL>
        static inline void bind(const std::shared_ptr<threading::Reaction>& reaction) {

            // Tell the director
            // TODO find a way to encode these expressions in the DSL
            reaction->reactor.powerplant.emit(std::make_unique<commands::WhenExpression>(reaction, [] {}));

            // Add our unbinder
            reaction->unbinders.emplace_back([](const threading::Reaction& r) {
                r.reactor.emit<word::emit::Direct>(std::make_unique<operation::Unbind<command::WhenExpression>>(r.id));
            });
        }
    };

    /**
     * This DSL word is used to create a promise that at some point when running this reaction the state indicated by T
     * will be true. It may not be on any particular run of the reaction, in which case it will continue to run that
     * reaction until it is true, or another graph jump becomes necessary.
     *
     * @tparam T the condition expression that this reaction will make true before leaving.
     */
    template <typename T>
    struct Causing {

        template <typename DSL>
        static inline void bind(const std::shared_ptr<threading::Reaction>& reaction) {

            // Tell the director
            // TODO find a way to encode these expressions in the DSL
            reaction->reactor.powerplant.emit(std::make_unique<commands::CausingExpression>(reaction, [] {}));

            // Add our unbinder
            reaction->unbinders.emplace_back([](const threading::Reaction& r) {
                r.reactor.emit<word::emit::Direct>(
                    std::make_unique<operation::Unbind<command::CausingExpression>>(r.id));
            });
        }
    };

    /**
     * A Task to be executed by the director, and a priority with which to execute this task.
     * The scope of the priority is contained within the reaction that emitted it.
     * So if a provider emits two tasks it will execute the one with higher priority.
     * The exception to this rule is task that are emitted from reactions that are not providers.
     * When a task is emitted by one of these general reactions it exists in a pool along with all other non provider
     * reactions. This is used to provide a list of all the tasks that must be executed by the system.
     *
     * When emitting a task, there is always only a single task for a given provider per reaction.
     * This task will persist until a new task is emitted from the reaction for the same provider and then this
     * task will be used instead.
     *
     * If the provided priority is zero, this task will instead be removed from the task list.
     *
     * @tparam T the provider type that this task is for
     */
    template <typename T>
    struct Task {
        static void emit(PowerPlant& powerplant, std::shared_ptr<T> data, int priority, const std::string& name = "") {

            // Work out who is sending the task
            auto* task  = NUClear::threading::ReactionTask::get_current_task();
            uint64_t id = task ? task->reaction->id : -1;

            NUClear::dsl::word::emit::Direct::emit(
                powerplant, std::make_shared<commands::DirectorTask>(typeid(T), id, data, priority, name));
        }
    };

}  // namespace behaviour
}  // namespace extension

#endif  // EXTENSION_BEHAVIOUR_H
