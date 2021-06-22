#ifndef EXTENSION_FINITE_STATE_MACHINE
#define EXTENSION_FINITE_STATE_MACHINE

#include <memory>
#include <tuple>
#include <nuclear>
#include <typeinfo>
#include <tuple>

namespace extension::finite_state {

    template <typename T>
    struct has_leaving {
        //TODO(cameron)
    };

    class State {
        public:
            virtual void entering(){}

            virtual void running() = 0;

            virtual void leaving(){}

            template <typename... Triggers>
            struct Transition {
                using triggers = std::tuple<Triggers...>;

                virtual State get_new_state() = 0;

                virtual bool transition_gaurd(){
                    return true;
                }
            };
    };

    template <typename Name>
    class FiniteStateMachineRunner : NUClear::Reactor {
        private:
            /*template <int I, typename Tuple, typename DSL>
            void call_bind(const std::shared_ptr<NUClear::threading::reaction>& reaction){
                NUClear::dsl::operation::TypeBind<std::tuple_element<I, Tuple>>::bind<DSL>(reaction);
                if constexpr (I < std::tuple_size_v<Tuple>){
                    call_bind<I+1, Tuple, DSL>(reaction);
                }
            }

            template <typename... Triggers>
            struct TriggersHandler {
                template <typename DSL>
                static inline void bind(const std::shared_ptr<NUClear::threading::reaction>& reaction){
                    call_bind<0, State, 
                }
            }*/

        public:
        FiniteStateMachineRunner(std::unique_ptr<NUClear::Environment> environment): NUClear::Reactor(environment){}

        static void install(NUClear::PowerPlant power_plant){
            power_plant.install<FiniteStateMachineRunner<Name>>();
        }
    };
}

#endif //EXTENSION_FINITE_STATE_MACHINE
