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
    #include <fcntl.h>
    #include <dirent.h>
    #include <unistd.h>
}

#include "utility/strutil/strutil.h"
#include "utility/configuration/ConfigurationNode.h"
#include "utility/configuration/json/parse.h"
#include "utility/configuration/json/serialize.h"
#include "utility/file/fileutil.h"
#include "utility/idiom/pimpl_impl.h"

#include "messages/support/Configuration.h"

namespace modules {
    namespace support {
        namespace configuration {

            class ConfigSystem::impl {
            public:
                using HandlerFunction = std::function<void (NUClear::Reactor*, const std::string&, const messages::support::ConfigurationNode&)>;

                std::set<std::type_index> loaded;
                std::map<std::string, std::vector<HandlerFunction>> handler;
                std::map<int, std::string> watchPath;
                std::map<std::string, NUClear::clock::time_point> timestamp;
                int watcherFd;
                int killFd;
                ConfigSystem* reactor;

                volatile bool running;

                void run();
                void kill();
                void loadDir(const std::string& path, HandlerFunction emit);
                void watchDir(const std::string& path);
                messages::support::ConfigurationNode buildConfigurationNode(const std::string& filePath);

                // Lots of space for events (definitely more then needed)
                static constexpr size_t MAX_EVENT_LEN = sizeof (inotify_event) * 1024 + 16;
                static constexpr const char* BASE_CONFIGURATION_PATH = "config/";
            };

            ConfigSystem::ConfigSystem(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

                m->running = true;
                m->watcherFd = inotify_init();
                m->killFd = eventfd(0, EFD_NONBLOCK);
                m->reactor = this;
                m->watchDir(impl::BASE_CONFIGURATION_PATH);

                on<Trigger<messages::support::ConfigurationConfiguration>>([this](const messages::support::ConfigurationConfiguration& command) {

                    // Check if we have already loaded this type's handler
                    if (m->loaded.find(command.requester) == std::end(m->loaded)) {
                        // We have now loaded the type
                        m->loaded.insert(command.requester);

                        std::string fullPath = impl::BASE_CONFIGURATION_PATH + command.configPath;
                        auto& handlers = m->handler[fullPath];

                        if (utility::file::isDir(fullPath)) {
                            // Make sure fullPath has a trailing /
                            if (!utility::strutil::endsWith(fullPath, "/")) {
                                fullPath += "/";
                            }

                            // If this is the first type watching this config dir, add a watch
                            // on the directory
                            if (handlers.empty()) {
                                m->watchDir(fullPath);
                            }

                            // Load all the config files in the given directory
                            m->loadDir(fullPath, command.initialEmitter);
                        }
                        else {
                            auto lastSlashIndex = command.configPath.rfind('/');
                            auto fileName = command.configPath.substr(lastSlashIndex == std::string::npos ? 0 : lastSlashIndex);
                            command.initialEmitter(this, fileName, messages::support::ConfigurationNode(m->buildConfigurationNode(fullPath)));
                        }

                        handlers.push_back(command.emitter);
                    }
                });

                std::function<void ()> run = std::bind(std::mem_fn(&impl::run), &*m);
                std::function<void ()> kill = std::bind(std::mem_fn(&impl::kill), &*m);

                powerPlant->addServiceTask(NUClear::threading::ThreadWorker::ServiceTask(run, kill));
            }

            messages::support::ConfigurationNode ConfigSystem::impl::buildConfigurationNode(const std::string& filePath) {
                timestamp[filePath] = NUClear::clock::now();
                return utility::configuration::json::parse(utility::file::loadFromFile(filePath));
            }

            void ConfigSystem::impl::loadDir(const std::string& path, ConfigSystem::impl::HandlerFunction emit) {
                // List the directory and recursively walk its tree
                auto elements = utility::file::listDir(path);

                for (const auto& element : elements) {
                    if (utility::strutil::endsWith(element, ".json")) {
                        emit(reactor, element, messages::support::ConfigurationNode(buildConfigurationNode(path + element)));
                    }
                }
            }

            void ConfigSystem::impl::watchDir(const std::string& path) {
                int wd = inotify_add_watch(watcherFd, path.c_str(), IN_ATTRIB | IN_MODIFY | IN_CREATE | IN_DELETE_SELF | IN_MOVE);
                watchPath[wd] = path;
            }

            void ConfigSystem::impl::run() {

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
                                            std::cout << "Loading " << fullPath << " with handler for " << handlers->first << std::endl;
                                            try {
                                                for (auto& emitter : handlers->second) {
                                                    emitter(reactor, name, messages::support::ConfigurationNode(buildConfigurationNode(fullPath)));
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
                                std::string fullPath = watchPath[event->wd] + event->name + "/";

                                // give the new dir the same handlers as its parents, or ignore
                                // it if there are none
                                auto handlers = handler.find(fullPath);
                                if (handlers != std::end(handler)) {
                                    watchDir(fullPath);

                                    // as above, catch exceptions so that a bad config file won't
                                    // crash the whole thread
                                    try {
                                        // add the directory to the watch list, load any configs in it
                                        for (auto& emitter : handlers->second) {
                                            loadDir(fullPath, emitter);
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

                                // find the moved directory in the path list and unwatch it
                                for (auto wdIter = std::begin(watchPath); wdIter != std::end(watchPath); ++wdIter) {
                                    if (wdIter->second == fullPath) {
                                        inotify_rm_watch(watcherFd, wdIter->first);
                                        watchPath.erase(wdIter);
                                        break;
                                    }
                                }
                            }
                            // If a watched directory is deleted
                            else if (event->mask & IN_DELETE_SELF) {
                                auto pathIter = watchPath.find(event->wd);
                                const std::string& path = pathIter->second;

                                // throw an error and stop watching if the root config directory was deleted
                                if (path == BASE_CONFIGURATION_PATH) {
                                    throw std::runtime_error("config directory deleted!");
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

            void ConfigSystem::impl::kill() {
                // Stop running
                running = false;

                // Signal the event file descriptor
                eventfd_t val = 1337;

                // This hides the fact we don't use this variable (if it doesn't work who cares, we're trying to kill the system)
                int written = write(killFd, &val, sizeof (eventfd_t));
                assert(written == sizeof(eventfd_t));
                (void) written; // Stop gcc from complaining about it being unused when NDEBUG is set
            }

        }  // configuration
    }  // support
}  // modules