#ifndef MESSAGES_CONFIGURATION_H_
#define MESSAGES_CONFIGURATION_H_

#include <NUClear.h>
#include "ConfigurationNode.h"

namespace messages {

    using namespace NUClear::Internal::Magic::MetaProgramming;

    // Anonymous namespace to hide details
    namespace {
        /*
         * This uses SFINAE to work out if the CONFIGURATION_PATH operator exists. If it does then doTest(0) will match
         * the int variant (as it is a closer match) but only so long as T::CONFIGURATION_PATH is defined (otherwise
         * substitution will fail. It will then fallback to the char verison (who's returntype is not void)
         */
        template<class T>
        static auto doTest(int) -> decltype(T::CONFIGURATION_PATH, void());
        template<class>
        static char doTest(char);

        /**
         * @brief Tests if the passed type's CONFIGURATION_PATH variable can be assigned to a string
         *
         * @details
         *  TODO
         */
        template<typename T>
        struct ConfigurationIsString :
        public Meta::If<
            std::is_assignable<std::string, decltype(T::CONFIGURATION_PATH)>,
            std::true_type,
            std::false_type
        > {};

        template<typename T>
        struct HasConfiguration :
        public Meta::If<std::is_void<decltype(doTest<T>(0))>, ConfigurationIsString<T>, std::false_type> {};
    }

    template <typename TType>
    struct Configuration : public ConfigurationNode {
        static_assert(HasConfiguration<TType>::value, "The passed type does not have a CONFIGURATION_PATH variable");
    };

    // TODO this is used to tell the config system what to do
    struct ConfigurationConfiguration {
        std::type_index requester;
        std::string configPath;
        std::function<void (NUClear::Reactor*, messages::ConfigurationNode*)> emitter;
    };
}

// Our extension
namespace NUClear {
    template <typename TConfiguration>
    struct NUClear::Reactor::Exists<messages::Configuration<TConfiguration>> {
        static void exists(NUClear::Reactor* context) {

            // Build our lambda we will use to trigger this reaction
            std::function<void (Reactor*, messages::ConfigurationNode*)> emitter =
            [](Reactor* configReactor, messages::ConfigurationNode* node) {

                // We cast our node to be the correct type (to trigger the correct reaction) and emit it
                configReactor->emit(static_cast<messages::Configuration<TConfiguration>*>(node));
            };

            // Emit it from our reactor to the config system
            context->emit<Scope::DIRECT>(new messages::ConfigurationConfiguration{typeid(TConfiguration),
                                                                                  TConfiguration::CONFIGURATION_PATH,
                                                                                  emitter});
        }
    };
}

#endif
