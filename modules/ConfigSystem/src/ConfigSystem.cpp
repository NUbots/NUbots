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

#include "json/Parser.h"
#include "utility/strutil/strutil.h"

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

                if(isDir(ent)) {
                    result.push_back(file + "/");
                }
                else if (utility::strutil::endsWith(file, ".json")) {
                    result.push_back(file);
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

        // Read the data from the file into a string.
        std::ifstream data(filePath, std::ios::in);

        // There are lots of nice ways to read a file into a string but this is one of the quickest.
        // See: http://stackoverflow.com/a/116220
        std::stringstream stream;
        stream << data.rdbuf();

        return json::Parser::parse(stream.str());
    }

    ConfigSystem::ConfigSystem(NUClear::PowerPlant* plant) : Reactor(plant), running(true), watcherFd(inotify_init()), killFd(eventfd(0, EFD_NONBLOCK)) {

        int wd = inotify_add_watch(watcherFd, BASE_CONFIGURATION_PATH, IN_ATTRIB | IN_MODIFY | IN_CREATE | IN_DELETE_SELF | IN_MOVE_SELF | IN_MOVED_TO);
        watchPath[wd] = BASE_CONFIGURATION_PATH;

        on<Trigger<messages::ConfigurationConfiguration>>([this](const messages::ConfigurationConfiguration& command) {

            // Check if we have already loaded this type's handler
            if (loaded.find(command.requester) == std::end(loaded)) {
                // We have now loaded the type
                loaded.insert(command.requester);

                // Recursively load and watch all config files
                watch(BASE_CONFIGURATION_PATH + command.configPath, command.emitter, command.initialEmitter);
            }
        });

        std::function<void ()> run = std::bind(std::mem_fn(&ConfigSystem::run), this);
        std::function<void ()> kill = std::bind(std::mem_fn(&ConfigSystem::kill), this);

        powerPlant->addServiceTask(NUClear::Internal::ThreadWorker::ServiceTask(run, kill));
    }

    void ConfigSystem::watch(const std::string& filePath, ConfigSystem::HandlerFunction emitter, ConfigSystem::HandlerFunction emitNow) {
        // Add this emitter to our handler for this path
        auto& handlers = handler[filePath];
        handlers.push_back(emitter); 

        // Check if the configuration request is a directory
        if(isDir(filePath)) {
            auto elements = listContents(filePath);

            for(const auto& element : elements) {
                watch(filePath + element, emitter, emitNow);
            }

            // Add a watch on this directory if we're the first handler to be added to it
            if(handlers.size() == 1) {
                int wd = inotify_add_watch(watcherFd, filePath.c_str(), IN_ATTRIB | IN_MODIFY | IN_CREATE | IN_DELETE_SELF | IN_MOVE_SELF | IN_MOVED_TO);
                watchPath[wd] = filePath;
            }
        }
        else {
            auto lastSlashIndex = filePath.rfind('/');
            auto fileName = filePath.substr(lastSlashIndex == std::string::npos ? 0 : lastSlashIndex);
            emitNow(this, fileName, messages::ConfigurationNode(buildConfigurationNode(filePath)));
        }
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
                        if (event->mask & (IN_ATTRIB | IN_CREATE | IN_MODIFY | IN_MOVED_TO) && !(event->mask & IN_ISDIR)) {
                            std::string name = std::string(event->name);
                            if (utility::strutil::endsWith(name, ".json")) {
                                const std::string& parent = watchPath[event->wd];
                                std::string fullPath = parent + name;

                                std::cout << "Reloaded " << fullPath << std::endl;

                                auto handlers = handler.find(fullPath);
                                // if this is a new file it may not have a handler registered,
                                // use the parent directory's handler instead
                                if (handlers == std::end(handler)) {
                                    handlers = handler.find(parent);
                                }
                                // if there's still no handler then this is an unknown JSON file in
                                // the root of the config dir, just ignore it

                                if (handlers != std::end(handler)) {
                                    try {
                                        for (auto& emitter : handlers->second) {
                                            emitter(this, name, messages::ConfigurationNode(buildConfigurationNode(fullPath)));
                                        }
                                    }
                                    catch(const std::exception& e) {
                                        // so that an error reading/applying config doesn't crash
                                        // the whole config thread
                                        std::cout << "Exception thrown while configuring " << fullPath << ": " << e.what() << std::endl;
                                    }
                                }
                            }
                        }
                        // If a directory is created
                        else if (event->mask & (IN_CREATE | IN_MOVED_TO) && event->mask & IN_ISDIR) {
                            const std::string& parent = watchPath[event->wd];
                            std::string fullPath = parent + event->name + "/";

                            // make sure we have a handler for the new directory
                            auto handlers = handler.find(parent);
                            if (handlers != std::end(handler)) {
                                // add the directory to the watch list, load any configs in it
                                for (auto& emitter : handlers->second) {
                                    watch(fullPath, emitter, emitter);
                                }
                            }
                        }
                        // If a watched directory is deleted
                        else if (event->mask & (IN_DELETE_SELF | IN_MOVE_SELF)) {
                            auto pathIter = watchPath.find(event->wd);
                            const std::string& path = pathIter->second;

                            // throw an error and stop watching if the root config directory was deleted
                            if (path == BASE_CONFIGURATION_PATH) {
                                throw std::runtime_error("config directory deleted!");
                            }

                            // Delete all the handlers registered for this dir and its files
                            for (auto handlerIter = std::begin(handler); handlerIter != std::end(handler);) {
                                if (utility::strutil::startsWith(handlerIter->first, path)) {
                                    handler.erase(handlerIter++);
                                }
                                else {
                                    ++handlerIter;
                                }
                            }

                            watchPath.erase(pathIter);
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
