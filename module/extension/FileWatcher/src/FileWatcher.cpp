/*
 * MIT License
 *
 * Copyright (c) 2017 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "FileWatcher.hpp"

#include <filesystem>
#include <fmt/format.h>
#include <vector>

#include "extension/FileWatch.hpp"

namespace module::extension {

    using ::extension::FileWatch;
    using ::extension::FileWatchRequest;
    using Unbind = NUClear::dsl::operation::Unbind<FileWatch>;

    void FileWatcher::file_watch_callback(uv_fs_event_t* handle, const char* filename, int events, int /* status */) {

        struct FileExecTask {
            std::shared_ptr<NUClear::threading::Reaction> reaction;
            std::string path;
            int events;
        };

        // Regain our reactor
        FileWatcher& reactor = *reinterpret_cast<FileWatcher*>(handle->data);

        // A list of tasks that we will execute after we release the mutex to prevent deadlocks
        std::vector<FileExecTask> exec_queue;

        auto exec = [&reactor](NUClear::threading::Reaction& r, const std::string& p, const int& events) {
            // Set our thread local event details
            FileWatch watch;
            watch.path   = p;
            watch.events = events;

            // Store our watch value in the local cache
            FileWatch::FileWatchStore::value = &watch;

            // Directly execute our reaction here
            auto task = r.get_task();

            // Clear our local cache
            FileWatch::FileWatchStore::value = nullptr;

            if (task) {
                reactor.powerplant.submit(std::move(task));
            }
        };

        /* mutex scope */ {
            // Lock our mutex as we are editing our datastructure
            std::lock_guard<std::mutex> lock(reactor.paths_mutex);

            // Work out what path we are watching
            std::array<char, 512> pathbuff{};
            size_t size = pathbuff.size();
            uv_fs_event_getpath(handle, pathbuff.data(), &size);
            std::string path(pathbuff.data());

            std::string fullpath = fmt::format("{}/{}", path, filename);

            if (reactor.paths.find(path) != reactor.paths.end()) {
                auto& p = reactor.paths[path];

                // If someone is watching this directory
                if (p.files.find("") != p.files.end()) {
                    for (auto& r : p.files[""].reactions) {

                        // If there are events we are interested in
                        if ((r.events & events) != 0) {
                            exec_queue.push_back(FileExecTask{r.reaction, fullpath, events});
                        }
                    }
                }

                // If someone is watching for this file
                if (p.files.find(filename) != p.files.end()) {
                    for (auto& r : p.files[filename].reactions) {

                        // If there are events we are interested in
                        if ((r.events & events) != 0) {
                            exec_queue.push_back(FileExecTask{r.reaction, fullpath, events});
                        }
                    }
                }
            }
        }

        for (auto& task : exec_queue) {
            exec(*task.reaction, task.path, task.events);
        }
    }

    FileWatcher::FileWatcher(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , loop(std::make_unique<uv_loop_t>())
        , add_watch(std::make_unique<uv_async_t>())
        , remove_watch(std::make_unique<uv_async_t>())
        , shutdown(std::make_unique<uv_async_t>()) {

        // Initialise our UV loop
        uv_loop_init(loop.get());

        // Initialise our shutdown event to stop uv
        uv_async_init(loop.get(), shutdown.get(), [](uv_async_t* handle) {
            // Stop the loop
            uv_stop(handle->loop);
        });

        // Initialise our add_watch event to add new watches
        uv_async_init(loop.get(), add_watch.get(), [](uv_async_t* async_handle) {
            // Grab our reactor context back
            FileWatcher& reactor = *reinterpret_cast<FileWatcher*>(async_handle->data);

            // Lock our mutex as we are editing our datastructure
            std::lock_guard<std::mutex> lock(reactor.paths_mutex);

            for (auto it = reactor.add_queue.begin(); it != reactor.add_queue.end();) {
                auto& map = *it;
                uv_fs_event_init(async_handle->loop, map->handle.get());
                uv_fs_event_start(map->handle.get(),
                                  &FileWatcher::file_watch_callback,
                                  map->path.c_str(),
                                  UV_RENAME | UV_CHANGE);
                it = reactor.add_queue.erase(it);
            }
        });
        add_watch->data = this;

        // Initialise our remove_watch event to remove watches
        uv_async_init(loop.get(), remove_watch.get(), [](uv_async_t* async_handle) {
            // Grab our reactor context back
            FileWatcher& reactor = *reinterpret_cast<FileWatcher*>(async_handle->data);

            // Lock our mutex as we are editing our datastructure
            std::lock_guard<std::mutex> lock(reactor.paths_mutex);

            for (auto it = reactor.remove_queue.begin(); it != reactor.remove_queue.end();) {
                uv_close(reinterpret_cast<uv_handle_t*>(it->get()), [](uv_handle_t* /* handle */) {});
                it = reactor.remove_queue.erase(it);
            }
        });
        remove_watch->data = this;

        on<Always>().then("FileWatcher", [this] {
            if (first_loop) {
                // The first time run with no wait so if there are no events we won't get stuck
                uv_run(loop.get(), UV_RUN_NOWAIT);
                first_loop = false;
                emit(std::make_unique<::extension::FileWatcherReady>());
            }
            else {
                // Run our event loop
                uv_run(loop.get(), UV_RUN_DEFAULT);
            }
        });

        // Shutdown with the system
        on<Shutdown>().then("Shutdown FileWatcher", [this] {
            // Send an event to shutdown
            uv_async_send(shutdown.get());
        });

        on<Trigger<Unbind>>().then("Unbind FileWatch", [this](const Unbind& fw) {
            // Lock our mutex as we are editing our datastructure
            std::lock_guard<std::mutex> lock(paths_mutex);

            // Find the reaction to unbind
            for (auto pIt = paths.begin(); pIt != paths.end(); ++pIt) {
                for (auto fIt = pIt->second.files.begin(); fIt != pIt->second.files.end(); ++fIt) {
                    for (auto rIt = fIt->second.reactions.begin(); rIt != fIt->second.reactions.end(); ++rIt) {
                        if (rIt->reaction->id == fw.id) {

                            // Erase this reaction
                            fIt->second.reactions.erase(rIt);

                            // If erasing this reaction got rid of this file, erase this file
                            if (fIt->second.reactions.empty()) {
                                pIt->second.files.erase(fIt);
                            }

                            // If erasing this file got rid of this path, unwatch this path
                            if (pIt->second.files.empty()) {

                                // TODO(thouliston) unwatch the path
                                remove_queue.push_back(std::move(pIt->second.handle));

                                paths.erase(pIt);
                            }

                            // We are done no need to look further
                            // NOTE this also means it's fine to just ++it rather than setting them from erase
                            // Normally this would be bad but we will never use these iterators again
                            return;
                        }
                    }
                }
            }
        });

        on<Trigger<FileWatchRequest>>().then("Add FileWatch", [this](const FileWatchRequest& req) {
            // Get the real path
            std::filesystem::path path = std::filesystem::absolute(req.path);

            std::string filename = path.filename();
            std::string dir      = path.parent_path();

            // If it's a directory then there is no filename to watch
            if (std::filesystem::is_directory(path)) {
                dir      = path;
                filename = "";
            }

            /* mutex scope */ {
                // Lock our mutex as we are editing our datastructure
                std::lock_guard<std::mutex> lock(paths_mutex);

                // If this is a new path to watch
                if (paths.find(dir) == paths.end()) {
                    auto& p        = paths[path];
                    p.handle       = std::make_unique<uv_fs_event_t>();
                    p.handle->data = this;
                    p.path         = dir;
                    add_queue.push_back(&p);
                    uv_async_send(add_watch.get());
                }

                // Add our reaction here into the correct spot
                paths[dir].files[filename].reactions.push_back(ReactionMap{req.reaction, req.events});
            }

            if (std::filesystem::is_directory(path)) {
                // Initial emit for each of our files in this folder
                for (const auto& p : std::filesystem::directory_iterator(path)) {
                    // Set our thread local event details
                    FileWatch watch;
                    watch.path   = p.path();
                    watch.events = 0;

                    // Store our watch value in the local cache
                    FileWatch::FileWatchStore::value = &watch;

                    // Directly execute our reaction here
                    auto task = req.reaction->get_task();
                    if (task) {
                        powerplant.submit(std::move(task), true);
                    }

                    // Clear our local cache
                    FileWatch::FileWatchStore::value = nullptr;
                }
            }
            else {
                // Set our thread local event details
                FileWatch watch;
                watch.path   = path;
                watch.events = 0;

                // Store our watch value in the local cache
                FileWatch::FileWatchStore::value = &watch;

                // Directly execute our reaction here
                auto task = req.reaction->get_task();
                if (task) {
                    powerplant.submit(std::move(task), true);
                }

                // Clear our local cache
                FileWatch::FileWatchStore::value = nullptr;
            }
        });
    }

    FileWatcher::~FileWatcher() {
        // Make sure that the shutdown has been run
        uv_run(loop.get(), UV_RUN_NOWAIT);
        // Add close callbacks to all the handles
        uv_walk(
            loop.get(),
            [](uv_handle_t* handle, void*) { uv_close(handle, [](uv_handle_t* /* handle */) {}); },
            nullptr);
        // Run all the close callbacks
        uv_run(loop.get(), UV_RUN_NOWAIT);
        uv_loop_close(loop.get());
    }
}  // namespace module::extension
