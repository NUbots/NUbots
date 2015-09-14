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

#include <cstdlib>
#include <nuclear>
#include <yaml-cpp/yaml.h>

#include "messages/support/FileWatch.h"

namespace messages {
    namespace support {

        /**
         * TODO document
         *
         * @author Trent Houliston
         */
        struct Configuration {
            std::string path;
            YAML::Node config;

            Configuration(const std::string& path, YAML::Node config) : path(path), config(config) {};

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

        struct SaveConfiguration {
            std::string path;
            YAML::Node config;
        };

    }  // support
}  // messages

// NUClear configuration extension
namespace NUClear {
    namespace dsl {
        namespace operation {
            template <>
            struct DSLProxy<messages::support::Configuration> {

                template <typename DSL, typename TFunc>
                static inline threading::ReactionHandle bind(Reactor& reactor, const std::string& label, TFunc&& callback, const std::string& path) {
                    return DSLProxy<messages::support::FileWatch>::bind<DSL>(reactor, label, callback, "config/" + path,
                                                                             messages::support::FileWatch::ATTRIBUTES
                                                                             | messages::support::FileWatch::CREATE
                                                                             | messages::support::FileWatch::MODIFY
                                                                             | messages::support::FileWatch::MOVED_TO);
                }

                template <typename DSL>
                static inline std::shared_ptr<messages::support::Configuration> get(threading::ReactionTask& t) {

                    // Get the file watch event
                    messages::support::FileWatch watch = DSLProxy<messages::support::FileWatch>::get<DSL>(t);

                    // TODO test which events fired and if it's relevant to us

                    // Return our yaml file
                    return std::make_shared<messages::support::Configuration>(watch.path, YAML::LoadFile(watch.path));
                }
            };
        }
    }
}

#endif
