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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGES_SUPPORT_CONFIGURATION_H_
#define MESSAGES_SUPPORT_CONFIGURATION_H_

#include <nuclear>
#include <yaml-cpp/yaml.h>
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

namespace messages {
    namespace support {

        using namespace NUClear::metaprogramming;

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

        /**
         * TODO document
         *
         * @author Trent Houliston
         */
        template <typename TType>
        struct Configuration {
            static_assert(HasConfiguration<TType>::value, "The passed type does not have a CONFIGURATION_PATH variable");

            std::string name;
            YAML::Node config;

            Configuration(const std::string& name, YAML::Node config) : name(name), config(config) {};

            YAML::Node operator [] (const std::string& key) {
                return config[key];
            }

            const YAML::Node operator [] (const std::string& key) const {
                return config[key];
            }

            YAML::Node operator [] (const char* key) {
                return config[key];
            }

            const YAML::Node operator [] (const char* key) const {
                return config[key];
            }

            YAML::Node operator [] (size_t index) {
                return config[index];
            }

            const YAML::Node operator [] (size_t index) const {
                return config[index];
            }
        };

        /**
         * TODO document
         *
         * @author Trent Houliston
         */
        struct SaveConfiguration {
            std::string path;
            YAML::Node config;
        };

        /**
         * TODO document
         *
         * @author Trent Houliston
         */
        struct ConfigurationConfiguration {
            std::type_index requester;
            std::string configPath;
            std::function<void (NUClear::Reactor*, const std::string&, const YAML::Node&)> emitter;
            std::function<void (NUClear::Reactor*, const std::string&, const YAML::Node&)> initialEmitter;
        };

    }  // support
}  // messages

// Our extension
namespace NUClear {

    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    template <typename TConfiguration>
    struct NUClear::Reactor::Exists<messages::support::Configuration<TConfiguration>> {
        static void exists(NUClear::Reactor& context) {

            // Build our lambda we will use to trigger this reaction
            std::function<void (Reactor*, const std::string&, const YAML::Node&)> emitter =
            [](Reactor* configReactor, const std::string& name, const YAML::Node& node) {
                // Cast our node to be the correct type (and wrap it in a unique pointer)
                try {
                    configReactor->emit(std::make_unique<messages::support::Configuration<TConfiguration>>(name, node));
                }

                catch (...) {
                    NUClear::log<NUClear::FATAL>("Config error in file: ", name);
                    throw;
                }
            };

            // We need to emit our initial configuration directly in order to avoid race conditions where
            // a main reactor tries to load configuration information before the configurations are loaded.
            std::function<void (Reactor*, const std::string&, const YAML::Node&)> initialEmitter =
            [](Reactor* configReactor, const std::string& name, const YAML::Node& node) {
                // Cast our node to be the correct type (and wrap it in a unique pointer)
                try {
                    configReactor->emit<Scope::DIRECT>(std::make_unique<messages::support::Configuration<TConfiguration>>(name, node));
                }

                catch (...) {
                    NUClear::log<NUClear::FATAL>("Config error in file: ", name);
                    throw;
                }
            };

            // Emit it from our reactor to the config system
            context.emit<Scope::INITIALIZE>(std::unique_ptr<messages::support::ConfigurationConfiguration>(
                new messages::support::ConfigurationConfiguration {
                    typeid(TConfiguration),
                    TConfiguration::CONFIGURATION_PATH,
                    emitter,
                    initialEmitter
            }));
        }
    };
}  // NUClear

#endif
