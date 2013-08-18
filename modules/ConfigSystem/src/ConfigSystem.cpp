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

    Messages::ConfigurationNode* buildConfigurationNode(std::string filePath) {
		
		Messages::ConfigurationNode* node = new Messages::ConfigurationNode();
		
		// Read the file as JSON using some lib or something
		
        
        // TODO using the file descriptor provided, read it and build a configurationnode tree

        return node;
    }

    ConfigSystem::ConfigSystem(NUClear::PowerPlant* plant) : Reactor(plant), watcherFd(inotify_init()) {

        on<Trigger<Messages::ConfigurationConfiguration>>([this](const Messages::ConfigurationConfiguration& command) {

			// Check if we already have this path loaded
			if (configurations.find(command.configPath) == std::end(configurations)) {
				
				// We are interested when a file is modified or deleted
				int wd = inotify_add_watch(watcherFd, command.configPath.c_str(), IN_MODIFY | IN_DELETE_SELF);
				
				// Store our watch descriptor so we can look it up later
				wdMap[wd] = command.configPath;
			}
			
			// Check if we have loaded this type
			if (configurations[command.configPath].find(command.requester) == std::end(configurations[command.configPath])) {
				
				// Add this emitter to the list
				configurations[command.configPath][command.requester] = command.emitter;
				
				// Run our emitter to emit the initial object
				command.emitter(this, buildConfigurationNode(command.configPath));
			}
        });
    }
}
