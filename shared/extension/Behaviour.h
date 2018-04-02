#ifndef EXTENSION_BEHAVIOUR_H
#define EXTENSION_BEHAVIOUR_H

namespace extension {
namespace behaviour {

    /**
     * This DSL word used to define an entry transition into a provider.
     * It should normally be coupled with a Causing DSL word.
     * When coupled with a Causing DSL word and a provider that has a When condition,
     * this entry will run in place of the provider until the When condition is fulfilled.
     * If it is used without a Causing DSL word it will run once as a provider before the actual provider is executed.
     *
     * @tparam T the type that this transition function provides for
     */
    template <typename T>
    struct Entering {

        template <typename DSL>
        static inline void bind(const std::shared_ptr<threading::Reaction>& reaction) {
            // Emit an entering condition for this reaction

            // TODO SEND TO DIRECTOR

            // Add our unbinder
            reaction->unbinders.emplace_back([](const threading::Reaction& r) {
                r.reactor.emit<word::emit::Direct>(std::make_unique<operation::Unbind<Entering<T>>>(r.id));
            });
        }
    };

    /**
     * This DSL word is used to define a leaving transition from a provider.
     * It should normally be combined with a Causing DSL word.
     * In this case when another provider needs control and uses a When condition, this will run until that condition is
     * fulfilled
     *
     * @tparam T the type that this transition function provides for
     */
    template <typename T>
    struct Leaving {

        template <typename DSL>
        static inline void bind(const std::shared_ptr<threading::Reaction>& reaction) {
            // Emit a Leaving condition for this reaction

            // TODO SEND TO DIRECTOR

            // Add our unbinder
            reaction->unbinders.emplace_back([](const threading::Reaction& r) {
                r.reactor.emit<word::emit::Direct>(std::make_unique<operation::Unbind<Leaving<T>>>(r.id));
            });
        }
    };

    /**
     * This DSL word is used to define a provider for a type.
     * It will execute in the same way as an on<Trigger<T>> statement except that when it executes will be determined
     * by the Director. This ensures that it will only run when it has permission to run and will transition between
     * providers in a sensible way.
     *
     * @tparam T the type that this transition function provides for
     */
    template <typename T>
    struct Provides {

        template <typename DSL>
        static inline void bind(const std::shared_ptr<threading::Reaction>& reaction) {
            // Emit a provides condition for this guy

            // TODO SEND TO DIRECTOR

            // Add our unbinder
            reaction->unbinders.emplace_back([](const threading::Reaction& r) {
                r.reactor.emit<word::emit::Direct>(std::make_unique<operation::Unbind<Provides<T>>>(r.id));
            });
        }
    };

    /**
     * The When DSL word is used to limit access to a provider unless a condition is true.
     * This will prevent a provider from running if the state is false.
     * However if there is a Entering<T> or Leaving reaction that has a Causing relationship for this When, it will be
     * executed to ensure that the Provider can run.
     *
     * @tparam T the type that encapsulates the state that must be true in order to execute this task.
     */
    template <typename T>
    struct When {

        template <typename DSL>
        static inline void bind(const std::shared_ptr<threading::Reaction>& reaction) {
            // Add this when relationship for this reaction

            // TODO SEND TO DIRECTOR

            // Add our unbinder
            reaction->unbinders.emplace_back([](const threading::Reaction& r) {
                r.reactor.emit<word::emit::Direct>(std::make_unique<operation::Unbind<When<T>>>(r.id));
            });
        }
    };

    /**
     * This DSL word is used to create a promise that at some point when running this reaction the state indicated by T
     * will be true. It may not be on any particular run of the reaction, in which case it will continue to run that
     * reaction until it is true, or another graph jump becomes necessary.
     */
    template <typename T>
    struct Causing {

        template <typename DSL>
        static inline void bind(const std::shared_ptr<threading::Reaction>& reaction) {
            // Add this Causing relationship for this reaction

            // TODO SEND TO DIRECTOR

            // Add our unbinder
            reaction->unbinders.emplace_back([](const threading::Reaction& r) {
                r.reactor.emit<word::emit::Direct>(std::make_unique<operation::Unbind<Causing<T>>>(r.id));
            });
        }
    };

    /**
     * This DSL word is used to indicate relationships between providers.
     * In order for some providers to run they must use the services of other providers.
     * These relationships create the structure for the execution of tasks.
     * When a task is scheduled to run all its dependencies must be recursively obtained at once.
     * The entire tree that is executing will always be based on these Uses relationships.
     * For example, if a tree contains a using for a Provider of A, even when leaving this whole tree will execute until
     * the Leaves<A> if it exists is done, and then the entire tree will transition at once.
     */
    template <typename T>
    struct Uses {

        template <typename DSL>
        static inline void bind(const std::shared_ptr<threading::Reaction>& reaction) {
            // Add this Uses relationship for this reaction

            // TODO SEND TO DIRECTOR

            // Add our unbinder
            reaction->unbinders.emplace_back([](const threading::Reaction& r) {
                r.reactor.emit<word::emit::Direct>(std::make_unique<operation::Unbind<Uses<T>>>(r.id));
            });
        }
    };

    template <typename T>
    struct Task {
        T command;
        int priority;
    };

}  // namespace behaviour
}  // namespace extension

#endif  // EXTENSION_BEHAVIOUR_H
