#ifndef EXTENSION_FINITE_STATE_MACHINE
#define EXTENSION_FINITE_STATE_MACHINE

#include <memory>
#include <nuclear>
#include <string>
#include <vector>
#include <atomic>
#include <typeinfo>

namespace extension::finite_state {

    template <typename T>
    std::string type_name(){
        return NUClear::demangle(typeid(T).name());
    }

    template <typename T>
    void recursive_demangle(std::vector<std::string>& destination){
        destination.emplace_back(type_name<T>());
    }

    template <typename T, typename... U>
    void recursive_demangle(std::vector<std::string>& destination){
        destination.emplace_back(type_name<T>());
        recursive_demangle<U...>(destination);
    }

    template <int I, typename T>
    bool recursive_check(const std::vector<std::string>& current_state){
        return destination[I] == type_name<T>();
    }

    template <int I, typename T, typename... U>
    bool recursive_check(const std::vector<std::string>& current_state){
        return recursive_check<I + 1, U...>(current_state) && destination[I] == type_name<T>();
    }

    struct initial{};

    std::vector<std::string> current_state = initial();

    std::atomic_bool transitioning = false;

    template <typename Transition, typename... State>
    struct Entering : NUClear::dsl::word::Trigger<Transition> {
        template <typename DSL>
        static inline bool precondition(NUClear::threading::reaction& reaction){
            if(transitioning.load || !recursive_check<0, State...>(current_state)){
                return false;
            }

            transitioning.store(true);
            reaction.reactor.powerplant.emit<NUClear::dsl::word::emit::Direct>(std::make_unique<run_leaving<Transition, State...>>());
        }

        template <typename DSL>
        static inline void postcondition(NUClear::threading::ReactionTask& task){
            transitioning.store(false);
            task.parent.reactor.powerplant.emit(std::make_unique<run_state<Transition, State...>>());
        }
    }

    template <typename T, typename... S>
    struct run_state {};

    template <typename Transition, typename... State>
    struct Running : NUClear::dsl::operation::TypeBind<run_state<Transition, State...>>, NUClear::dsl::operation::CacheGet<Transition> {
        template <typename DSL>
        static inline void bind(const std::shared_ptr<threading::Reaction>& reaction){
            // We do want to run bind from Trigger
            NUClear::dsl::word::Trigger<Transition>::bind();

            //TODO(cameron) Somehow store the templated types, so we can check 
        }

        template <typename DSL>
        static inline void postcondition(NUClear::threading::ReactionTask& task){
            // We will wait in the queue, so we don't use up a whole thread.
            task.parent.reactor.powerplant.emit(std::make_unique<run_state<Transition, State...>>());
        }
    }

    template <typename T, typename... S>
    struct run_leaving {};

    template <typename Transition, typename... State>
    struct Leaving :  NUClear::dsl::operation::TypeBind<run_state<Transition, State...>>, NUClear::dsl::operation::CacheGet<Transition>{}

    namespace trait {
        // TODO(cameron) Work out transience
    }
}

#endif //EXTENSION_FINITE_STATE_MACHINE
