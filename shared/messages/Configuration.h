#ifndef MESSAGES_CONFIGURATION_H_
#define MESSAGES_CONFIGURATION_H_

#include <NUClear.h>

namespace Messages {

    class ConfigurationNode {

        /// This holds our data
        std::unique_ptr<void> data;
        
        enum class DataType {
            STRING,
            INTEGER,
            FLOAT,
            ARRAY,
            OBJECT,
            NULLPTR
        } datatype;

        template <typename TType>
        operator TType();

        template <typename TType>
        ConfigurationNode operator=(TType from);
    };

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
}

#endif