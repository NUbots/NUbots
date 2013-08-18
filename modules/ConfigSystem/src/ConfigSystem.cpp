/*
 * This file is part of ConfigSystem.
 *
 * ConfigSystem is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ConfigSystem is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with ConfigSystem.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include "ConfigSystem.h"

#include <sys/inotify.h>
#include <fcntl.h>
#include <unistd.h>

namespace modules {

    Messages::ConfigurationNode* buildConfigurationNode(int fileDescriptor) {
        
        // TODO using the file descriptor provided, read it and build a configurationnode tree

        return nullptr;
    }

    ConfigSystem::ConfigSystem(NUClear::PowerPlant* plant) : Reactor(plant) {

        watcherFd = inotify_init();

        on<Trigger<AddConfiguration>>([this](const AddConfiguration& command) {

            // Attempt to open our path as a file (before we get carried away adding it)
            int fd = open(command.path.c_str(), O_RDWR);

            if (fd < 0) {
                // TODO error that this config is unable to be found emit a bad node (nullptr node)
            }
            else {
                // Find or add our path
                auto& path = configurations[command.path];

                // Find our type and add our emitter if it's not already there
                auto type = path.find(command.requester);
                if (type == std::end(path)) {
                    path.insert(std::make_pair(command.requester, command.emitter));
                }

                // Insert our file descriptor into our list of files
                fileDescriptors.insert(std::make_pair(fd, command.path));

                // TODO read our configuration file and then emit it using the emitter
                Messages::ConfigurationNode* data = buildConfigurationNode(fd);
                command.emitter(this, data);

                // Add our file descriptor to the selector
                //FD_SET(fd, &selectorSet);



                // Open the file descriptor shown by command.path

                // Parse the file and use the emitter to emit it

                // Add this to our "Select" list

                // We need a map that maps paths to a vector of emittiers

                // We need to only add a new emitter when we don't have that typeindex
            }
        });
    }
}
