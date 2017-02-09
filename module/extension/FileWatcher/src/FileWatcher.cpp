/*
 * Copyright (C) 2013-2016 Trent Houliston <trent@houliston.me>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "FileWatcher.h"

#include "extension/FileWatch.h"
#include "utility/file/fileutil.h"

namespace module {
namespace extension {

    using ::extension::FileWatch;
    using ::extension::FileWatchRequest;
    using Unbind = NUClear::dsl::operation::Unbind<FileWatch>;

    void fswatchCallback(const std::vector<fsw::event>& events, void* r) {
        FileWatcher& reactor = *reinterpret_cast<FileWatcher*>(r);

        // Look through our filesystem events
        for (auto& event : events) {

            // Get our path
            auto path = event.get_path();

            // Split our path into events
            std::string dir;
            std::string filename;
            std::tie(dir, filename) = utility::file::pathSplit(path);

            // See if we can find this directory
            auto handler = reactor.handlers.find(dir);
            if (reactor.handlers.find(dir) != reactor.handlers.end()) {
                auto reactions = handler->second.find(filename);

                if(reactions != handler->second.end()) {
                    for (auto& reaction : reactions->second) {
                        uint flags = 0;
                        for (auto& flag : event.get_flags()) {
                            flags |= flag;
                        }

                        if (flags & reaction.second) {
                            // Set our thread local event details
                            FileWatch watch;
                            watch.path = path;
                            watch.events = flags;

                            // Store our watch value in the local cache
                            FileWatch::FileWatchStore::value = &watch;

                            // Directly execute our reaction here
                            auto task = reaction.first->get_task();
                            if(task) {
                                reactor.powerplant.submit(std::move(task));
                            }

                            // Clear our local cache
                            FileWatch::FileWatchStore::value = nullptr;
                        }
                    }
                }
            }
        }
    }

    FileWatcher::FileWatcher(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , handlers()
    , runMutex ()
    , monitor(nullptr) {

        // We use an on always here as we can't use on IO due to the library
        on<Always>().then("File Watcher", [this] {

            // Lock so that while we are changing things this won't run
            std::lock_guard<std::mutex> lock(runMutex);

            std::vector<std::string> watchPaths;
            watchPaths.reserve(handlers.size());

            // Create our list of watch paths
            for(auto& path : handlers) {
                watchPaths.push_back(path.first);
            }

            // Replace our old watcher with a new shiny one
            // TODO in the future this may not be needed if there
            //      is a way to add paths dynamically
            monitor.reset(fsw::monitor_factory::create_monitor(
                    fsw_monitor_type::system_default_monitor_type
                  , watchPaths
                  , fswatchCallback
                  , this
            ));

            // This will execute until it is told to stop
            monitor->start();
        });

        // Shutdown with the system
        on<Shutdown>().then([this] {
            if (monitor) {
                monitor->stop();
            };
        });

        on<Trigger<Unbind>>().then([this](const Unbind& fw) {

            // Lock our mutex and stop our monitor
            std::lock_guard<std::mutex> lock(runMutex);
            monitor->stop();

            // Find the reaction to unbind
            for(auto pIt = handlers.begin(); pIt != handlers.end(); ++pIt) {
                for(auto fIt = pIt->second.begin(); fIt != pIt->second.end(); ++fIt) {
                    for(auto rIt = fIt->second.begin(); rIt != fIt->second.end(); ++rIt) {
                        if (rIt->first->id == fw.id) {

                            // Erase this reaction
                            fIt->second.erase(rIt);

                            // If erasing this reaction got rid of this file, erase this file
                            if(fIt->second.empty()) {
                                pIt->second.erase(fIt);
                            }

                            // If erasing this file got rid of this path, unwatch this path
                            if(pIt->second.empty()) {
                                handlers.erase(pIt);
                            }

                            // We are done no need to look further
                            return;
                        }
                    }
                }
            }

            // The monitor should now restart itself
        });

        on<Trigger<FileWatchRequest>>().then([this](const FileWatchRequest& req) {

            // Get the real path with a unique ptr to ensure it is freed properly
            std::unique_ptr<char, void(*)(void*)> realPath {
                ::realpath(req.path.c_str(), nullptr),
                std::free
            };

            // If this happens then the config file does not exist
            if(!realPath) {

            }

            std::string path = realPath.get();
            std::string dir;
            std::string filename;

            std::tie(dir, filename) = utility::file::pathSplit(path);

            if(utility::file::isDir(path)) {

                // Add our reaction here into the correct spot
                handlers[path][""].push_back(std::make_pair(req.reaction, req.events));

                // Initial emit for each of our files in this folder
                for (const auto& element : utility::file::listDir(path)) {
                    // Set our thread local event details
                    FileWatch watch;
                    watch.path = path + "/" + element;
                    watch.events = 0;

                    // Store our watch value in the local cache
                    FileWatch::FileWatchStore::value = &watch;

                    // Directly execute our reaction here
                    auto task = req.reaction->get_task();
                    if(task) {
                        task->run(std::move(task));
                    }

                    // Clear our local cache
                    FileWatch::FileWatchStore::value = nullptr;
                }
            }
            else {

                // Add our reaction here into the correct spot
                handlers[dir][filename].push_back(std::make_pair(req.reaction, req.events));

                // Set our thread local event details
                FileWatch watch;
                watch.path = path;
                watch.events = 0;

                // Store our watch value in the local cache
                FileWatch::FileWatchStore::value = &watch;

                // Directly execute our reaction here
                auto task = req.reaction->get_task();
                if(task) {
                    task->run(std::move(task));
                }

                // Clear our local cache
                FileWatch::FileWatchStore::value = nullptr;
            }
        });
//
//        on<IO>(watcherFd, IO::READ).then([this] (const IO::Event& event) {
//
//            // Read our file system changes
//            uint8_t buffer[1024];
//
//            // Our map of results
//            std::map<std::string, std::map<std::string, int>> results;
//
//            // Loop through the events
//            for(int len = read(event.fd, buffer, sizeof(buffer));
//                len > 0;
//                len = read(event.fd, buffer, sizeof(buffer))) {
//                for(int i = 0; i < len;) {
//
//                    // Get the current event
//                    inotify_event* event = reinterpret_cast<inotify_event*>(buffer + i);
//                    std::string path(watchPaths[event->wd]);
//                    std::string file(event->name);
//
//                    results[path][file] |= (event->mask & IN_ACCESS)        ? FileWatch::ACCESS        : 0;
//                    results[path][file] |= (event->mask & IN_ATTRIB)        ? FileWatch::ATTRIBUTES    : 0;
//                    results[path][file] |= (event->mask & IN_CLOSE_WRITE)   ? FileWatch::CLOSE_WRITE   : 0;
//                    results[path][file] |= (event->mask & IN_CLOSE_NOWRITE) ? FileWatch::CLOSE_NOWRITE : 0;
//                    results[path][file] |= (event->mask & IN_CREATE)        ? FileWatch::CREATE        : 0;
//                    results[path][file] |= (event->mask & IN_DELETE)        ? FileWatch::DELETE        : 0;
//                    results[path][file] |= (event->mask & IN_DELETE_SELF)   ? FileWatch::DELETE_SELF   : 0;
//                    results[path][file] |= (event->mask & IN_MODIFY)        ? FileWatch::MODIFY        : 0;
//                    results[path][file] |= (event->mask & IN_MOVE_SELF)     ? FileWatch::MOVE_SELF     : 0;
//                    results[path][file] |= (event->mask & IN_MOVED_FROM)    ? FileWatch::MOVED_FROM    : 0;
//                    results[path][file] |= (event->mask & IN_MOVED_TO)      ? FileWatch::MOVED_TO      : 0;
//                    results[path][file] |= (event->mask & IN_OPEN)          ? FileWatch::OPEN          : 0;
//                    results[path][file] |= (event->mask & IN_IGNORED)       ? FileWatch::IGNORED       : 0;
//                    results[path][file] |= (event->mask & IN_ISDIR)         ? FileWatch::ISDIR         : 0;
//                    results[path][file] |= (event->mask & IN_UNMOUNT)       ? FileWatch::UNMOUNT       : 0;
//
//                    // Move to the next event
//                    i += sizeof(inotify_event) + event->len;
//                }
//
//                // Sleeping for one millisecond here bunches up the events
//                // So they all come through in a single call
//                std::this_thread::sleep_for(std::chrono::milliseconds(1));
//            }
//
//            for(auto& result : results) {
//                for(auto& el : result.second) {
//
//                    // Check if we have this path
//                    if(handlers.find(result.first) != handlers.end()) {
//                        auto& files = handlers[result.first];
//
//                        // TODO search for the empty file set too
//                        // that one will have stuff as well
//
//                        if(files.find("") != files.end()) {
//                            auto& reactions = files[""];
//
//                            // Loop through the reactions for this file
//                            for(auto& reaction : reactions) {
//
//                                // If this reaction is interested in the event
//                                if((reaction.second & el.second) > 0) {
//
//                                    // Set our thread local event details
//                                    FileWatch watch;
//                                    watch.path = result.first + "/" + el.first;
//                                    watch.events = el.second;
//
//                                    // Store our watch value in the local cache
//                                    FileWatch::FileWatchStore::value = &watch;
//
//                                    // Submit the task (which should run the get)
//                                    auto task = reaction.first->get_task();
//                                    if(task) {
//                                        powerplant.submit(std::move(task));
//                                    }
//
//                                    // Clear our local cache
//                                    FileWatch::FileWatchStore::value = nullptr;
//                                }
//                            }
//                        }
//
//                        // Check if we have the file
//                        if(files.find(el.first) != files.end()) {
//                            auto& reactions = files[el.first];
//
//                            // Loop through the reactions for this file
//                            for(auto& reaction : reactions) {
//
//                                // If this reaction is interested in the event
//                                if((reaction.second & el.second) > 0) {
//
//                                    // Set our thread local event details
//                                    FileWatch watch;
//                                    watch.path = result.first + "/" + el.first;
//                                    watch.events = el.second;
//
//                                    // Store our watch value in the local cache
//                                    FileWatch::FileWatchStore::value = &watch;
//
//                                    // Submit the task (which should run the get)
//                                    auto task = reaction.first->get_task();
//                                    if(task) {
//                                        powerplant.submit(std::move(task));
//                                    }
//
//                                    // Clear our local cache
//                                    FileWatch::FileWatchStore::value = nullptr;
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        });
    }
}
}
