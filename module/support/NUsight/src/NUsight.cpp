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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "NUsight.h"

#include "extension/Configuration.h"
#include "message/support/SaveConfiguration.h"
#include "message/support/nusight/Command.h"
#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/nusight/NUhelpers.h"

namespace module {
namespace support {

    using extension::Configuration;

    using message::support::SaveConfiguration;
    using message::support::nusight::Command;

    using utility::nusight::graph;

    NUsight::NUsight(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , max_image_duration()
        , max_classified_image_duration()
        , handles()
        , actionRegisters() {

        // These go first so the config can do things with them
        provideOverview();
        provideDataPoints();
        provideSubsumption();
        provideGameController();
        provideLocalisation();
        provideReactionStatistics();
        provideSensors();
        provideVision();

        on<Configuration>("NUsight.yaml").then([this](const Configuration& config) {
            using namespace std::chrono;
            max_image_duration = duration_cast<NUClear::clock::duration>(
                duration<double>(1.0 / config["output"]["network"]["max_image_fps"].as<double>()));
            max_classified_image_duration = duration_cast<NUClear::clock::duration>(
                duration<double>(1.0 / config["output"]["network"]["max_classified_image_fps"].as<double>()));

            for (auto& setting : config["reaction_handles"].config) {
                // Lowercase the name
                std::string name = setting.first.as<std::string>();
                std::transform(name.begin(), name.end(), name.begin(), ::tolower);

                bool enabled = setting.second.as<bool>();

                bool found = false;
                for (auto& handle : handles[name]) {
                    if (enabled && !handle.enabled()) {
                        handle.enable();
                        found = true;
                    }

                    else if (!enabled && handle.enabled()) {
                        handle.disable();
                        found = true;
                    }
                }

                if (found) {
                    if (enabled) {
                        log<NUClear::INFO>("Enabled:", name);
                    }
                    else {
                        log<NUClear::INFO>("Disabled:", name);
                    }
                }
            }
        });


        on<Network<Command>>().then("Network Command", [this](const Command& message) {
            std::string command = message.command;
            log<NUClear::INFO>("Received command:", command);
            if (command == "get_subsumption") {
                sendSubsumption();
            }
        });
    }

}  // namespace support
}  // namespace module
