/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#include "FileWatcher.h"

#include <unistd.h>
#include <fcntl.h>
#include <libgen.h>
#include <sys/inotify.h>

#include "messages/support/FileWatch.h"
#include "utility/file/fileutil.h"

namespace modules {
namespace support {
namespace extension {

    using messages::support::FileWatch;
    using messages::support::FileWatchRequest;

    FileWatcher::FileWatcher(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , watcherFd(inotify_init()) {

        fcntl (watcherFd, F_SETFL, fcntl (watcherFd, F_GETFL) | O_NONBLOCK);

        on<Trigger<FileWatchRequest>>().then([this](const FileWatchRequest& req) {

            char buff[1024];

            // Get our absolute path, folder and filename
            std::string path     = ::realpath(req.path.c_str(), buff);
            std::string dir      = ::dirname(buff);
            ::realpath(req.path.c_str(), buff);
            std::string filename = ::basename(buff);

            if(utility::file::isDir(path)) {

                // Check if we are already watching this path
                bool watchingPath = false;
                for(auto& pair : watchPaths) {
                    if(pair.second == path) {
                        watchingPath = true;
                        break;
                    }
                }

                // Check if we are already watching this path
                if(!watchingPath) {
                    // We are not watching this path so start watching
                    int wd = inotify_add_watch(watcherFd, path.c_str()
                                               , IN_ATTRIB | IN_MODIFY | IN_CREATE | IN_DELETE_SELF | IN_MOVE);

                    // Insert our watch path
                    watchPaths.insert(std::make_pair(wd, path));
                }

                // Put the reaction into the correct spot
                handlers[path][""].push_back(std::make_pair(req.reaction, req.events));

                // Emit for each of our files in this folder
                for (const auto& element : utility::file::listDir(path)) {
                    // Set our thread local event details
                    FileWatch watch;
                    watch.path = path + "/" + element;
                    watch.events = 0;

                    // Store our watch value in the local cache
                    FileWatch::FileWatchStore::value = &watch;

                    // Submit our reaction here
                    powerplant.submit(req.reaction->getTask());

                    // Clear our local cache
                    FileWatch::FileWatchStore::value = nullptr;
                }
            }
            else {
                // Check if we are already watching this path
                bool watchingPath = false;
                for(auto& pair : watchPaths) {
                    if(pair.second == dir) {
                        watchingPath = true;
                        break;
                    }
                }

                if(!watchingPath) {
                    // We are not watching this path so start watching
                    int wd = inotify_add_watch(watcherFd, dir.c_str(), IN_ALL_EVENTS | IN_EXCL_UNLINK);

                    // Insert our watch path
                    watchPaths.insert(std::make_pair(wd, dir));
                }

                handlers[dir][filename].push_back(std::make_pair(req.reaction, req.events));

                // Set our thread local event details
                FileWatch watch;
                watch.path = path;
                watch.events = 0;

                // Store our watch value in the local cache
                FileWatch::FileWatchStore::value = &watch;

                // Submit our reaction here
                powerplant.submit(req.reaction->getTask());

                // Clear our local cache
                FileWatch::FileWatchStore::value = nullptr;
            }
        });

        on<IO>(watcherFd, IO::READ).then([this] (const IO::Event& event) {

            // Read our file system changes
            uint8_t buffer[1024];

            // Our map of results
            std::map<std::string, std::map<std::string, int>> results;

            // Loop through the events
            for(int len = read(event.fd, buffer, sizeof(buffer));
                len > 0;
                len = read(event.fd, buffer, sizeof(buffer))) {
                for(int i = 0; i < len;) {

                    // Get the current event
                    inotify_event& event = *reinterpret_cast<inotify_event*>(&buffer[i]);
                    std::string path(watchPaths[event.wd]);
                    std::string file(event.name);

                    results[path][file] |= (event.mask & IN_ACCESS)        ? FileWatch::ACCESS        : 0;
                    results[path][file] |= (event.mask & IN_ATTRIB)        ? FileWatch::ATTRIBUTES    : 0;
                    results[path][file] |= (event.mask & IN_CLOSE_WRITE)   ? FileWatch::CLOSE_WRITE   : 0;
                    results[path][file] |= (event.mask & IN_CLOSE_NOWRITE) ? FileWatch::CLOSE_NOWRITE : 0;
                    results[path][file] |= (event.mask & IN_CREATE)        ? FileWatch::CREATE        : 0;
                    results[path][file] |= (event.mask & IN_DELETE)        ? FileWatch::DELETE        : 0;
                    results[path][file] |= (event.mask & IN_DELETE_SELF)   ? FileWatch::DELETE_SELF   : 0;
                    results[path][file] |= (event.mask & IN_MODIFY)        ? FileWatch::MODIFY        : 0;
                    results[path][file] |= (event.mask & IN_MOVE_SELF)     ? FileWatch::MOVE_SELF     : 0;
                    results[path][file] |= (event.mask & IN_MOVED_FROM)    ? FileWatch::MOVED_FROM    : 0;
                    results[path][file] |= (event.mask & IN_MOVED_TO)      ? FileWatch::MOVED_TO      : 0;
                    results[path][file] |= (event.mask & IN_OPEN)          ? FileWatch::OPEN          : 0;
                    results[path][file] |= (event.mask & IN_IGNORED)       ? FileWatch::IGNORED       : 0;
                    results[path][file] |= (event.mask & IN_ISDIR)         ? FileWatch::ISDIR         : 0;
                    results[path][file] |= (event.mask & IN_UNMOUNT)       ? FileWatch::UNMOUNT       : 0;

                    // Move to the next event
                    i += sizeof(inotify_event) + event.len;
                }

                // Sleeping for one millisecond here bunches up the events
                // So they all come through in a single call
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            for(auto& result : results) {
                for(auto& el : result.second) {

                    // Check if we have this path
                    if(handlers.find(result.first) != handlers.end()) {
                        auto& files = handlers[result.first];

                        // TODO search for the empty file set too
                        // that one will have stuff as well

                        if(files.find("") != files.end()) {
                            auto& reactions = files[""];

                            // Loop through the reactions for this file
                            for(auto& reaction : reactions) {

                                // If this reaction is interested in the event
                                if((reaction.second & el.second) > 0) {

                                    // Set our thread local event details
                                    FileWatch watch;
                                    watch.path = result.first + "/" + el.first;
                                    watch.events = el.second;

                                    // Store our watch value in the local cache
                                    FileWatch::FileWatchStore::value = &watch;

                                    // Submit the task (which should run the get)
                                    powerplant.submit(reaction.first->getTask());

                                    // Clear our local cache
                                    FileWatch::FileWatchStore::value = nullptr;
                                }
                            }
                        }

                        // Check if we have the file
                        if(files.find(el.first) != files.end()) {
                            auto& reactions = files[el.first];

                            // Loop through the reactions for this file
                            for(auto& reaction : reactions) {

                                // If this reaction is interested in the event
                                if((reaction.second & el.second) > 0) {

                                    // Set our thread local event details
                                    FileWatch watch;
                                    watch.path = result.first + "/" + el.first;
                                    watch.events = el.second;

                                    // Store our watch value in the local cache
                                    FileWatch::FileWatchStore::value = &watch;

                                    // Submit the task (which should run the get)
                                    powerplant.submit(reaction.first->getTask());

                                    // Clear our local cache
                                    FileWatch::FileWatchStore::value = nullptr;
                                }
                            }
                        }
                    }
                }
            }
        });
    }
}
}
}
