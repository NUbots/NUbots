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
#include "json/Parser.h"

#include <sys/inotify.h>
#include <sys/eventfd.h>
#include <fcntl.h>
#include <unistd.h>
#include <fstream>

namespace modules {
    
    // Lots of space for events (definitely more then needed)
    const size_t MAX_EVENT_LEN = sizeof (inotify_event) * 1024 + 16;

    messages::ConfigurationNode buildConfigurationNode(std::string filePath) {
        
        std::cout << "I Totally entered this function " << filePath << std::endl;

        // Read the data from the file into a string.
        std::ifstream data(filePath, std::ios::in);
        
        std::cout << "Reading file " << filePath << std::endl;

        // There are lots of nice ways to read a file into a string but this is one of the quickest.
        // See: http://stackoverflow.com/a/116220
        std::stringstream stream;
        stream << data.rdbuf();

        return json::Parser::parse(stream.str());
    }

    ConfigSystem::ConfigSystem(NUClear::PowerPlant* plant) : Reactor(plant), watcherFd(inotify_init()), killFd(eventfd(0, EFD_NONBLOCK)) {

        // TODO get the directories file descriptor here and put it in
        inotify_add_watch(watcherFd, BASE_CONFIGURATION_PATH, IN_ATTRIB | IN_MODIFY | IN_CREATE | IN_DELETE_SELF);

        on <Trigger<messages::ConfigurationConfiguration>>([this](const messages::ConfigurationConfiguration& command) {

            // Check if we have already loaded this type's handler
            if (loaded.find(command.requester) == std::end(loaded)) {
                // We have now loaded the type
                loaded.insert(command.requester);

                // Add this emitter to our handler for this path
                handler[command.configPath].push_back(command.emitter);

                // Run our emitter to emit the initial object
                std::string fullPath = BASE_CONFIGURATION_PATH + command.configPath;

                command.emitter(this, new messages::ConfigurationNode(buildConfigurationNode(fullPath)));
            }
        });

        std::function<void ()> run = std::bind(std::mem_fn(&ConfigSystem::run), this);
        std::function<void ()> kill = std::bind(std::mem_fn(&ConfigSystem::kill), this);

        powerPlant->addServiceTask(NUClear::Internal::ThreadWorker::ServiceTask(run, kill));
    }

    void ConfigSystem::run() {

        // Build our descriptor set with the watcherFd and an event FD to terminate on demand
        fd_set fdset;

        // Zero our set
        FD_ZERO(&fdset);
        
        while (running) {
            // Add our two File descriptors to the set
            FD_SET(watcherFd, &fdset);
            FD_SET(killFd, &fdset);
            
            // This means wait indefinitely until something happens
            int result = select(killFd + 1, &fdset, nullptr, nullptr, nullptr);
            uint8_t buffer[MAX_EVENT_LEN];
            
            // We have events (0 if timeout, n (one for ready sockets) and -1 for error)
            if(result > 0) {
                // If we have been told to die then killFd will be set
                if(!FD_ISSET(killFd, &fdset)) {
                    
                    // Read events into our buffer
                    int length = read(watcherFd, buffer, MAX_EVENT_LEN);
                    
                    // Read our events
                    for (int i = 0; i < length;) {
                        inotify_event* event = reinterpret_cast<inotify_event*>(&buffer[i]);
                        
                        // If a config file was modified touched or created (not a directory)
                        if (event->mask & (IN_ATTRIB | IN_MODIFY | IN_CREATE) && !(event->mask & IN_ISDIR)) {
                            
                            std::string path = std::string(event->name);
                            std::string fullPath = BASE_CONFIGURATION_PATH + path;
                            
                            std::cout << "Reloaded " << fullPath << std::endl;
                            
                            for(auto& emitter : handler[path]) {
                                emitter(this, new messages::ConfigurationNode(buildConfigurationNode(fullPath)));
                            }
                        }
                        // If our directory was deleted then you broke the config system!
                        if (event->mask & IN_DELETE_SELF) {
                            // TODO throw an error of some sort here, the configuration system will no longer respond to events!
                        }
                        
                        i += sizeof(inotify_event) + event->len;
                    }
                }
            }
        }
    }

    void ConfigSystem::kill() {
        // Stop running
        running = false;

        // Signal the event file descriptor
        eventfd_t val = 1337;
        write(killFd, &val, sizeof (eventfd_t));

        // Close our file descriptors
        close(watcherFd);
        close(killFd);
    }
}