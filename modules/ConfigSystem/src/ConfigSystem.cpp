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

extern "C" {
    #include <sys/inotify.h>
    #include <sys/eventfd.h>
    #include <sys/stat.h>
    #include <fcntl.h>
    #include <dirent.h>
    #include <unistd.h>
}

#include <fstream>
#include <system_error>

#include "utility/strutil/strutil.h"
#include "utility/configuration/ConfigurationNode.h"
#include "utility/configuration/json/parse.h"
#include "utility/configuration/json/serialize.h"
#include "utility/file/fileutil.h"

namespace modules {

    // Lots of space for events (definitely more then needed)
    const size_t MAX_EVENT_LEN = sizeof (inotify_event) * 1024 + 16;

    // Test if a passed path is a directory
    bool isDir(const std::string& path) {

        int status;
        struct stat st_buf;

        // Get the status of the file system object.
        status = stat(path.c_str(), &st_buf);
        if (status != 0) {
            throw std::system_error(errno, std::system_category(), "Error checking if path is file or directory");
        }

        // Return if our varible is a directory
        return S_ISDIR(st_buf.st_mode);
    }

    bool isDir(dirent* ent) {
        return ent->d_type & DT_DIR;
    }

    std::vector<std::string> listContents(const std::string& path) {

        auto dir = opendir(path.c_str());
        std::vector<std::string> result;

        if(dir != nullptr) {
            for(dirent* ent = readdir(dir); ent != nullptr; ent = readdir(dir)) {

                auto file = std::string(ent->d_name);

                if(file == "." || file == "..") {
                    continue;
                }

                if(isDir(ent) || utility::strutil::endsWith(file, ".json")) {
                    result.push_back(std::string(ent->d_name));
                }
            }

            closedir(dir);
        }
        else {
            // TODO Throw an error or something
        }

        return result;
    }

    messages::ConfigurationNode buildConfigurationNode(const std::string& filePath) {
        return utility::configuration::json::parse(utility::file::loadFromFile(filePath));
    }

    ConfigSystem::ConfigSystem(NUClear::PowerPlant* plant) : Reactor(plant), running(true), watcherFd(inotify_init()), killFd(eventfd(0, EFD_NONBLOCK)) {

        // TODO get the directories file descriptor here and put it in
        int rWd = inotify_add_watch(watcherFd, BASE_CONFIGURATION_PATH, IN_ATTRIB | IN_MODIFY | IN_CREATE | IN_DELETE_SELF);

        // TODO recurse the base configuration path grabbing all directories and adding them to the watch

        on<Trigger<messages::ConfigurationConfiguration>>([this](const messages::ConfigurationConfiguration& command) {

            // Check if we have already loaded this type's handler
            if (loaded.find(command.requester) == std::end(loaded)) {
                // We have now loaded the type
                loaded.insert(command.requester);

                // Add this emitter to our handler for this path
                handler[command.configPath].push_back(command.emitter);

                // Run our emitter to emit the initial object
                std::string fullPath = BASE_CONFIGURATION_PATH + command.configPath;

                // Check if the configuration request is a directory
                if(isDir(fullPath)) {
                    auto elements = listContents(fullPath);

                    for(const auto& element : elements) {
                        if(!isDir(fullPath + element)) {
                            command.initialEmitter(this, element, messages::ConfigurationNode(buildConfigurationNode(fullPath + "/" + element)));
                        }
                    }
                }
                else {
                    auto lastSlashIndex = fullPath.rfind('/');
                    auto fileName = fullPath.substr(lastSlashIndex == std::string::npos ? 0 : lastSlashIndex);
                    command.initialEmitter(this, fileName, messages::ConfigurationNode(buildConfigurationNode(fullPath)));
                }
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

                            // todo get our relative path

                            std::string path = std::string(event->name);
                            std::string fullPath = BASE_CONFIGURATION_PATH + path;

                            log("Reloaded ", fullPath, '\n');

                            for(auto& emitter : handler[path]) {
                                emitter(this, path, messages::ConfigurationNode(buildConfigurationNode(fullPath)));
                            }
                        }
                        // If a directory is created
                        if (event->mask & (IN_CREATE | IN_ISDIR)) {
                            // TODO Add it to our watcher
                        }
                        if (event->mask & (IN_DELETE_SELF | IN_ISDIR)) {
                            // TODO If our main directory was deleted then you broke config! otherwise just the elements in it
                            // TODO in which case remove it from our WD map
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

        // Close our file descriptors
        close(watcherFd);
        close(killFd);
    }

    void ConfigSystem::kill() {
        // Stop running
        running = false;

        // Signal the event file descriptor
        eventfd_t val = 1337;

        // This hides the fact we don't use this variable (if it doesn't work who cares, we're trying to kill the system)
        int written = write(killFd, &val, sizeof (eventfd_t));
        assert(written == sizeof(eventfd_t));
        (void) written; // Stop gcc from complaining about it being unused when NDEBUG is set
    }
}
