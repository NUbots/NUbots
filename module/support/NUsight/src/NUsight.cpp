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
#include "message/support/nusight/ReactionHandles.h"
#include "message/vision/LookUpTable.h"

#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/time/time.h"

namespace module {
namespace support {

    using extension::Configuration;

    using message::support::SaveConfiguration;
    using message::support::nusight::Command;
    using message::support::nusight::ReactionHandles;
    using message::vision::LookUpTable;
    using message::vision::SaveLookUpTable;

    using utility::nusight::graph;
    using utility::time::durationFromSeconds;

    // Flag struct to upload a lut
    struct UploadLUT {};

    NUsight::NUsight(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , max_image_duration()
        , max_classified_image_duration()
        , handles()
        , actionRegisters() {

        // These go first so the config can do things with them
        provideOverview();
        provideDataPoints();
        provideDrawObjects();
        provideSubsumption();
        provideGameController();
        provideLocalisation();
        provideReactionStatistics();
        provideSensors();
        provideVision();

        on<Configuration>("NUsight.yaml").then([this](const Configuration& config) {
            max_image_duration = durationFromSeconds(1.0 / config["output"]["network"]["max_image_fps"].as<double>());
            max_classified_image_duration =
                durationFromSeconds(1.0 / config["output"]["network"]["max_classified_image_fps"].as<double>());

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

        on<Trigger<UploadLUT>, With<LookUpTable>>().then([this](std::shared_ptr<const LookUpTable> lut) {
            powerplant.emit_shared<Scope::NETWORK>(std::move(lut), "nusight", true);
        });

        on<Network<Command>>().then("Network Command", [this](const Command& message) {
            std::string command = message.command;
            log<NUClear::INFO>("Received command:", command);
            if (command == "download_lut") {

                // Emit something to make it upload the lut
                emit<Scope::DIRECT>(std::make_unique<UploadLUT>());
            }
            else if (command == "get_reaction_handles") {
                sendReactionHandles();
            }
            else if (command == "get_subsumption") {
                sendSubsumption();
            }
        });

        on<Network<LookUpTable>>().then([this](const LookUpTable& lut) {
            log<NUClear::INFO>("Loading LUT");
            emit<Scope::DIRECT>(std::make_unique<LookUpTable>(lut));

            log<NUClear::INFO>("Saving LUT to file");
            emit<Scope::DIRECT>(std::make_unique<SaveLookUpTable>());
        });

        on<Network<ReactionHandles>>().then(
            [this](const NetworkSource& /*source*/, const ReactionHandles& /*command*/) {
                // auto config = std::make_unique<SaveConfiguration>();
                // config->config = currentConfig->config;

                // for (const auto& command : message.reaction_handles().handles()) {

                //     Message::Type type = command.type();
                //     bool enabled = command.enabled();
                //     std::string key = getStringFromMessageType(type);

                //     std::transform(key.begin(), key.end(), key.begin(), ::tolower);

                //     config->path = CONFIGURATION_PATH;
                //     config->config["reaction_handles"][key] = enabled;
                //     for (auto& handle : handles[type]) {
                //         handle.enable(enabled);
                //     }
                // }

                // emit(std::move(config));
            });

        sendReactionHandles();

        on<Trigger<SaveConfiguration>>().then("Save Config", [this](const SaveConfiguration& config) {
            saveConfigurationFile(config.path, config.config);
        });
    }

    void NUsight::sendReactionHandles() {

        auto reactionHandles = std::make_unique<ReactionHandles>();

        for (auto& handle : handles) {
            ReactionHandles::Handle objHandle;
            objHandle.type    = handle.first;
            auto& value       = handle.second;
            objHandle.enabled = (value.empty()) ? true : value.front().enabled();
            reactionHandles->handles.push_back(objHandle);
        }

        emit<Scope::NETWORK>(reactionHandles, "nusight", true);
    }

}  // namespace support
}  // namespace module
