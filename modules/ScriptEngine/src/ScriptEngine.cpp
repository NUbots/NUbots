/*
 * This file is part of ScriptEngine.
 *
 * ScriptEngine is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ScriptEngine is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with ScriptEngine.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include "ScriptEngine.h"
#include "messages/Configuration.h"

// TODO extend the ConfigurationEditor in order to make ServoWaypoint objects

namespace modules {

    struct Scripts {
        // For scripts we want updates on the whole scripts directory
        static constexpr const char* CONFIGURATION_PATH = "scripts/";
    };

    ScriptEngine::ScriptEngine(NUClear::PowerPlant* plant) : Reactor(plant) {

        on<Trigger<messages::Configuration<Scripts>>>([this](const messages::Configuration<Scripts>& scripts) {
            // Add this script to our list of scripts
        });
    }
}
