/*
 * This file is part of ConfigSystem.
 *
 * ConfigSystem is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ConfigSystem is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ConfigSystem.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#ifndef MODULES_CONFIGSYSTEM_H_
#define MODULES_CONFIGSYSTEM_H_

#include <map>
#include <NUClear.h>
#include "messages/Configuration.h"

struct AddConfiguration {
    std::type_index requester;
    std::string path;
    std::function<void (NUClear::Reactor*, Messages::ConfigurationNode*)> emitter;
};

namespace NUClear {
    template <typename TConfiguration>
    struct NUClear::Reactor::Exists<Messages::Configuration<TConfiguration>> {
        static void exists(NUClear::Reactor* context) {

            // Build our lambda we will use to trigger this reaction
            std::function<void (Reactor*, Messages::ConfigurationNode*)> emitter =
            [](Reactor* configReactor, Messages::ConfigurationNode* node) {

                // We cast our node to be the correct type (to trigger the correct reaction) and emit it
                configReactor->emit(static_cast<Messages::Configuration<TConfiguration>*>(node));
            };

            // Emit it from our reactor
            context->emit(new AddConfiguration(typeid(TConfiguration), TConfiguration::CONFIGURATION_PATH, emitter));
        }
    };
}

namespace modules {

    std::map<std::string, std::map<std::type_index,
    std::function<void (NUClear::Reactor*, Messages::ConfigurationNode*)>>> configurations;

    std::map<int, std::string> fileDescriptors;

    int watcherFd;

    class ConfigSystem : public NUClear::Reactor {
    public:
        explicit ConfigSystem(NUClear::PowerPlant* plant);
    };
}
#endif

