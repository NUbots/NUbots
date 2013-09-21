/*
 * This file is part of DanceEngine.
 *
 * DanceEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DanceEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DanceEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "DanceEngine.h"

#include "messages/Configuration.h"
#include "messages/Beat.h"

namespace modules {
    struct DanceScripts {
        // For scripts we want updates on the whole scripts directory
        static constexpr const char* CONFIGURATION_PATH = "scripts/dance/";
    };

    ScriptEngine::ScriptEngine(NUClear::PowerPlant* plant) : Reactor(plant) {

        on<Trigger<messages::Configuration<DanceScripts>>>([this](const messages::Configuration<Scripts>& script) {
            // Add this script to our list of scripts
            scripts.insert(std::make_pair(script.name, script.config));
        });

        on<Trigger<messages::ExecuteScriptByName>>([this](const messages::Beat& beat) {

            // If we are currently not dancing
            // Pick a dance script to execute
            // Scale that dance script to a number of beats
            // Emit an execute script command that will finish on a beat
        });
    }
}
