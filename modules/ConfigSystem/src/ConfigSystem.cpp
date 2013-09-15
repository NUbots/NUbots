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
 * Copyright 2013 NUBots <nubots@nubots.net>
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

#include "utility/strutil/strutil.h"
#include "utility/configuration/ConfigurationNode.h"
#include "utility/configuration/json/parse.h"
#include "utility/configuration/json/serialize.h"
#include "utility/file/fileutil.h"

namespace modules {

    // Lots of space for events (definitely more then needed)
    const size_t MAX_EVENT_LEN = sizeof (inotify_event) * 1024 + 16;

    messages::ConfigurationNode ConfigSystem::buildConfigurationNode(const std::string& filePath) {
        std::cout << "Loading config " << filePath << std::endl;
        timestamp[filePath] = NUClear::clock::now();
        return utility::configuration::json::parse(utility::file::loadFromFile(filePath));
    }

    ConfigSystem::ConfigSystem(NUClear::PowerPlant* plant) : Reactor(plant), running(true), watcherFd(inotify_init()), killFd(eventfd(0, EFD_NONBLOCK)) {

        int wd = inotify_add_watch(watcherFd, BASE_CONFIGURATION_PATH, IN_ATTRIB | IN_MODIFY | IN_CREATE | IN_DELETE_SELF | IN_MOVE);
        watchPath[wd] = BASE_CONFIGURATION_PATH;

        on<Trigger<messages::ConfigurationConfiguration>>([this](const messages::ConfigurationConfiguration& command) {

            // Check if we have already loaded this type's handler
            if (loaded.find(command.requester) == std::end(loaded)) {
                // We have now loaded the type
                loaded.insert(command.requester);

                std::string fullPath = BASE_CONFIGURATION_PATH + command.configPath;
                if (utility::file::isDir(fullPath)) {
                    // Recursively load and watch all config files
                    watchDir(fullPath, command.emitter, command.initialEmitter);
                }
                else {
                    // Register and load just the one file
                    handler[fullPath].push_back(command.emitter);

                    auto lastSlashIndex = command.configPath.rfind('/');
                    auto fileName = command.configPath.substr(lastSlashIndex == std::string::npos ? 0 : lastSlashIndex);
                    command.initialEmitter(this, fileName, messages::ConfigurationNode(buildConfigurationNode(fullPath)));
                }
            }
        });

        std::function<void ()> run = std::bind(std::mem_fn(&ConfigSystem::run), this);
        std::function<void ()> kill = std::bind(std::mem_fn(&ConfigSystem::kill), this);

        powerPlant->addServiceTask(NUClear::Internal::ThreadWorker::ServiceTask(run, kill));
    }

    void ConfigSystem::watchDir(const std::string& path, ConfigSystem::HandlerFunction emitter, ConfigSystem::HandlerFunction emitNow) {
        // Add a handler for all json files in this directory...
        auto& handlers = handler[path];
        if (handlers.empty()) {
            // Add a watch on this directory
            int wd = inotify_add_watch(watcherFd, path.c_str(), IN_ATTRIB | IN_MODIFY | IN_CREATE | IN_DELETE_SELF | IN_MOVE);
            watchPath[wd] = path;
            std::cout << "Watching " << path << std::endl;
        }
        handlers.push_back(emitter); 

        // List the directory and recursively walk its tree
        auto elements = utility::file::listDir(path);

        for (const auto& element : elements) {
            if (utility::strutil::endsWith(element, "/")) { // listDir always adds / to dirs
                watchDir(path + element, emitter, emitNow);
            }
            else if (utility::strutil::endsWith(element, ".json")) {
                emitNow(this, element, messages::ConfigurationNode(buildConfigurationNode(path + element)));
            }
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
            // If we have been told to die then killFd will be set
            if(result > 0 && !FD_ISSET(killFd, &fdset)) {

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

                            if (event->mask & IN_ATTRIB) {
                                std::cout << "Attrib ";
                            } else if (event->mask & IN_CREATE) {
                                std::cout << "Create ";
                            } else if(event->mask & IN_MODIFY) {
                                std::cout << "Modify ";
                            } else {
                                std::cout << "Move to ";
                            }
                            std::cout << fullPath << std::endl;

                            // Depending on which text editor is being used, and whether
                            // a file is new or has just been modified, several events
                            // may occur on the same file in quick succession. To prevent
                            // reloading the file several times, only allow reloading it
                            // once per second.
                            auto timeDiff = std::chrono::duration_cast<std::chrono::seconds>(NUClear::clock::now() - timestamp[fullPath]);
                            if (timeDiff.count() >= 1) {
                                auto handlers = handler.find(fullPath);
                                // If there was no handler for this specific file, check
                                // if there's one for its parent directory
                                if (handlers == std::end(handler)) {
                                    handlers = handler.find(parent);
                                }
                                // if there's still no handler then this is an unknown
                                // file in the root of the config dir, just ignore it

                                if (handlers != std::end(handler)) {
                                    try {
                                        for (auto& emitter : handlers->second) {
                                            emitter(this, name, messages::ConfigurationNode(buildConfigurationNode(fullPath)));
                                        }
                                    }
                                    catch(const std::exception& e) {
                                        // so that an error reading/applying config
                                        // doesn't crash the whole config thread
                                        std::cout << "Exception thrown while configuring " << fullPath << ": " << e.what() << std::endl;
                                    }
                                }
                            }
                        }
                    }
                    // If a directory is created or moved/renamed
                    else if (event->mask & (IN_CREATE | IN_MOVED_TO) && event->mask & IN_ISDIR) {
                        const std::string& parent = watchPath[event->wd];
                        std::string fullPath = parent + event->name + "/";

                        std::cout << "Created dir " << fullPath << std::endl;

                        // give the new dir the same handlers as its parents, or ignore
                        // it if there are none
                        auto handlers = handler.find(parent);
                        if (handlers != std::end(handler)) {
                            // as above, catch exceptions so that a bad config file won't
                            // crash the whole thread
                            try {
                                // add the directory to the watch list, load any configs in it
                                for (auto& emitter : handlers->second) {
                                    watchDir(fullPath, emitter, emitter);
                                }
                            }
                            catch(const std::exception& e) {
                                std::cout << "Exception thrown while configuring " << fullPath << ": " << e.what() << std::endl;
                            }
                        }
                    }
                    // If a watched directory is moved/renamed
                    else if (event->mask & IN_MOVED_FROM && event->mask & IN_ISDIR) {
                        std::string fullPath = watchPath[event->wd] + event->name + "/";
                        std::cout << "Moving away " << fullPath << std::endl;

                        // Deregister all handlers for the directory and its subdirs
                        for (auto handlerIter = std::begin(handler); handlerIter != std::end(handler);) {
                            if (utility::strutil::startsWith(handlerIter->first, fullPath)) {
                                handler.erase(handlerIter++);
                            }
                            else {
                                ++handlerIter;
                            }
                        }

                        // and unwatch them too
                        for (auto wdIter = std::begin(watchPath); wdIter != std::end(watchPath);) {
                            if (utility::strutil::startsWith(wdIter->second, fullPath)) {
                                inotify_rm_watch(watcherFd, wdIter->first);
                                watchPath.erase(wdIter++);
                            }
                            else {
                                ++wdIter;
                            }
                        }
                    }
                    // If a watched directory is deleted
                    else if (event->mask & IN_DELETE_SELF) {
                        auto pathIter = watchPath.find(event->wd);
                        const std::string& path = pathIter->second;

                        std::cout << "Deleted dir " << path << std::endl;
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

                        inotify_rm_watch(watcherFd, event->wd);
                        watchPath.erase(pathIter);
                    }

                    i += sizeof(inotify_event) + event->len;
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
