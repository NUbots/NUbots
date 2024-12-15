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
#include <system_error>

#include "translate.hpp"

#include "extension/Configuration.hpp"
#include "extension/FileWatch.hpp"

#include "utility/strutil/strutil.hpp"

namespace module::extension {

    using ::extension::Configuration;
    using ::extension::FileWatch;
    using ::extension::FileWatchRequest;
    using Unbind = NUClear::dsl::operation::Unbind<FileWatch>;
    using utility::strutil::dedent;

    /**
     * @brief List all of the directories underneath the given directory
     *
     * @param root [in] the root directory to search under
     * @param paths [out] the directories that were found
     * @param root_path [in,out] used internally to keep track of original directory
     */
    void list_directories(const std::filesystem::path& root,
                          std::vector<std::filesystem::path>& paths,
                          const std::filesystem::path& root_path = {}) {

        // Keep track of the path to use for computing the relative path
        const std::filesystem::path& base_path = root_path.empty() ? root : root_path;

        // Traverse the current directory
        // Catch any errors to prevent exceptions
        std::error_code ec;
        for (auto const& dir_entry : std::filesystem::directory_iterator{root, ec}) {
            // If current entry is a directory and recursive mode is active, recurse into the directory
            if (std::filesystem::is_directory(dir_entry)) {
                paths.push_back(std::filesystem::relative(dir_entry.path(), base_path));
                list_directories(dir_entry, paths, base_path);
            }
        }

        // Check for any errors that occurred and report them
        if (ec) {
            NUClear::log<NUClear::ERROR>(fmt::format("Error iterating through '{}': {}", root.string(), ec.message()));
        }
    }

    /**
     * @brief List all of the files that match a specified regular expression under the specified root directory
     *
     * @param root [in] the root directory to search
     * @param re [in] the regular expression to filter files with
     * @param recursive [in] indicates if subdirectories should also be searched
     * @param paths [out] the files that match the regular expression under root
     * @param root_path [in,out] used internally to keep track of the original root directory
     */
    void list_files(const std::filesystem::path& root,
                    const std::regex& re,
                    const bool& recursive,
                    std::vector<std::filesystem::path>& paths,
                    const std::filesystem::path& root_path = {}) {

        // Keep track of the path to use for computing the relative path
        const std::filesystem::path& base_path = root_path.empty() ? root : root_path;

        // Traverse the current directory
        // Catch any errors to prevent exceptions
        std::error_code ec;
        for (auto const& dir_entry : std::filesystem::directory_iterator{root, ec}) {
            // If current entry is a directory and recursive mode is active, recurse into the directory
            if (recursive && std::filesystem::is_directory(dir_entry)) {
                list_files(dir_entry, re, recursive, paths, base_path);
            }
            // Not recursive, so don't care about directories
            else if (!std::filesystem::is_directory(dir_entry)) {

                // If the provided regex matches the relative portion of the file path add it to the vector
                if (std::regex_match(std::filesystem::relative(dir_entry.path(), root).string(), re)) {
                    // Add the path relative to the original root path
                    paths.push_back(std::filesystem::relative(dir_entry.path(), base_path));
                }
            }
        }

        // Check for any errors that occurred and report them
        if (ec) {
            NUClear::log<NUClear::ERROR>(fmt::format("Error iterating through '{}': {}", root.string(), ec.message()));
        }
    }

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

        auto exec = [&reactor](NUClear::threading::Reaction& r, const std::filesystem::path& path, const int& events) {
            // Set our thread local event details
            FileWatch watch;
            watch.path   = path;
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

        // Get the full path of the file that triggered the event
        std::array<char, 512> pathbuff{0};
        size_t size = pathbuff.size();
        ::uv_fs_event_getpath(handle, pathbuff.data(), &size);
        std::filesystem::path path(pathbuff.data());
        std::filesystem::path event_path =
            std::filesystem::absolute(std::filesystem::path(pathbuff.data()).lexically_normal());
        if (!event_path.string().ends_with(filename)) {
            event_path /= filename;
        }

        const bool renamed = (events & UV_RENAME) == UV_RENAME;
        const bool changed = (events & UV_CHANGE) == UV_CHANGE;

        /* mutex scope */ {
            // Lock our mutex as we are editing our data structure
            std::lock_guard<std::mutex> lock(reactor.watch_mutex);

            // Find the watch that corresponds to the event handle
            std::filesystem::path handle_path;
            auto reaction_it = std::find_if(reactor.watches.begin(), reactor.watches.end(), [&](const auto& reaction) {
                for (const auto& watch : reaction.second.watches) {
                    // Find the handle that matches the event handle
                    if (watch.second.handle.get() == handle) {
                        handle_path = watch.first;
                        return true;
                    }
                }
                return false;
            });
            if (reaction_it == reactor.watches.end()) {
                reactor.log<NUClear::DEBUG>(
                    fmt::format("Watch for {} has already been removed. Skipping ...", event_path.string()));
                return;
            }
            auto& current_reaction              = reactor.watches[reaction_it->first];
            auto& current_watch                 = current_reaction.watches[handle_path];
            std::filesystem::path reaction_path = std::filesystem::absolute((current_reaction.path).lexically_normal());
            reactor.log<NUClear::DEBUG>(dedent(fmt::format(
                R"(
                    Reaction: {}
                        Got event for watch {}
                        Parent path {}
                        Events:{}{}
                        Exists? {}
                )",
                reaction_it->first->identifiers->name,
                event_path.lexically_relative(std::filesystem::current_path()).string(),
                reaction_path.lexically_relative(std::filesystem::current_path()).string(),
                (events & UV_RENAME) == UV_RENAME ? " RENAMED" : "",
                (events & UV_CHANGE) == UV_CHANGE ? " CHANGED" : "",
                std::filesystem::exists(event_path))));

            // Event path has either been deleted or moved
            // If it was moved to another location being watched there will be a second RENAMED event handling the
            // adding
            std::filesystem::path rel_path = event_path.lexically_relative(reaction_path);
            if (!std::filesystem::exists(event_path)) {
                // Check to see if the event path matches a known file
                if (current_reaction.files.contains(rel_path)) {
                    reactor.log<NUClear::DEBUG>(fmt::format("\tRemoving known file {}", rel_path.string()));
                    current_reaction.files.erase(rel_path);
                    if (renamed
                        && (current_reaction.events & ::extension::FileWatch::DELETED)
                               == ::extension::FileWatch::DELETED) {
                        exec_queue.push_back(
                            FileExecTask{reaction_it->first, event_path, ::extension::FileWatch::DELETED});
                    }
                }

                // Check to see if the event path matches a watch
                std::string prefix;
                bool check_files = false;
                for (auto& watch : current_reaction.watches) {
                    if (watch.second.path == rel_path) {
                        if (!std::filesystem::exists(event_path) && watch.second.active) {
                            reactor.log<NUClear::DEBUG>(fmt::format("\tRemoving watch {}", rel_path.string()));
                            reactor.remove_queue.push_back(std::move(watch.second.handle));
                            watch.second.active = false;
                            prefix              = watch.second.path;
                            check_files         = true;
                            break;
                        }
                    }
                }

                // There are watches that need to be removed
                if (!reactor.remove_queue.empty()) {
                    auto err = ::uv_async_send(reactor.remove_watch.get());
                    if (err < 0) {
                        reactor.log<NUClear::ERROR>(
                            fmt::format("uv_async_send returned error {}: {}", err, ::uv_strerror(err)));
                    }
                }

                // Check for any known files that start with the prefix
                if (check_files) {
                    for (auto it = current_reaction.files.begin(); it != current_reaction.files.end();) {
                        if ((prefix != "." && it->string().starts_with(prefix))
                            || (prefix == "." && it->string().find('/') == std::string::npos)) {
                            reactor.log<NUClear::DEBUG>(fmt::format("\tRemoving known file {}", it->string()));
                            if (renamed
                                && (current_reaction.events & ::extension::FileWatch::DELETED)
                                       == ::extension::FileWatch::DELETED) {
                                exec_queue.push_back(FileExecTask{reaction_it->first,
                                                                  current_reaction.path / *it,
                                                                  ::extension::FileWatch::DELETED});
                            }
                            it = current_reaction.files.erase(it);
                        }
                        else {
                            ++it;
                        }
                    }
                }
            }
            // Event path is either a new file/directory (if it is a RENAMED event) or a modified file (if it is a
            // CHANGED event)
            else {
                // The event path is a file, make sure it is in the list of known files if the regex matches
                std::filesystem::path watch_path =
                    std::filesystem::absolute((current_reaction.path / current_watch.path).lexically_normal());
                if (std::filesystem::is_regular_file(event_path)
                    && std::regex_match(event_path.lexically_relative(watch_path).string(), current_reaction.re)) {

                    reactor.log<NUClear::DEBUG>(fmt::format("\tAdding {} to known files", rel_path.string()));

                    // If the file was already known about and this is a change event
                    // Or the file was not already known about and this is a rename event
                    // And in either case the reaction wants to know about this type of event
                    const bool inserted    = current_reaction.files.insert(rel_path).second;
                    const bool valid_event = (current_reaction.events & events) == events;
                    if ((inserted && renamed && valid_event) || (!inserted && changed && valid_event)) {
                        exec_queue.push_back(FileExecTask{reaction_it->first, event_path, events});
                    }
                }
                // Event path is a directory, make sure it is being watched
                // If the current reaction is not watching recursively we only need to act if the directory corresponds
                // to the root directory for the reaction
                else if (std::filesystem::is_directory(event_path)
                         && (event_path == current_reaction.path || current_reaction.recursive)) {
                    reactor.log<NUClear::DEBUG>(fmt::format("\tSetting up new watch for {}", rel_path.string()));

                    // Find all paths underneath the event path, this will return paths relative to the event path
                    std::vector<std::filesystem::path> paths;
                    if (current_reaction.recursive) {
                        list_directories(event_path, paths);

                        // Make all paths relative to the current reactions root path
                        std::transform(paths.begin(), paths.end(), paths.begin(), [&](const auto& p) {
                            return (event_path / p).lexically_relative(reaction_path);
                        });
                    }
                    paths.push_back(rel_path);

                    // Traverse all of the paths that were just found and add the missing ones
                    for (const auto& path : paths) {
                        if (!current_reaction.watches.contains(path)) {
                            auto& watch  = current_reaction.watches[path];
                            watch.path   = path;
                            watch.handle = std::make_unique<uv_fs_event_t>();
                            watch.active = false;
                        }

                        if (!current_reaction.watches[path].handle) {
                            current_reaction.watches[path].active = false;
                            current_reaction.watches[path].handle = std::make_unique<uv_fs_event_t>();
                        }
                    }

                    // Find all paths underneath the event path, this will return paths relative to the event path
                    paths.clear();
                    list_files(event_path, current_reaction.re, current_reaction.recursive, paths);
                    for (const auto& path : paths) {
                        std::filesystem::path rel_path = (event_path / path).lexically_relative(reaction_path);
                        if (current_reaction.files.insert(rel_path).second) {
                            exec_queue.push_back(
                                FileExecTask{reaction_it->first, event_path, ::extension::FileWatch::NO_OP});
                        }
                    }

                    reactor.add_queue.push_back(&current_reaction);
                }

                auto err = ::uv_async_send(reactor.add_watch.get());
                if (err < 0) {
                    reactor.log<NUClear::ERROR>(
                        fmt::format("uv_async_send returned error {}: {}", err, ::uv_strerror(err)));
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
        ::uv_loop_init(loop.get());

        // Initialise our shutdown event to stop uv
        ::uv_async_init(loop.get(), shutdown.get(), [](uv_async_t* handle) {
            // Stop the loop
            ::uv_stop(handle->loop);
        });

        // Initialise our add_watch event to add new watches
        ::uv_async_init(loop.get(), add_watch.get(), [](uv_async_t* async_handle) {
            // Grab our reactor context back
            FileWatcher& reactor = *reinterpret_cast<FileWatcher*>(async_handle->data);

            // Lock our mutex as we are editing our data structure
            std::lock_guard<std::mutex> lock(reactor.watch_mutex);

            bool error = false;
            for (auto& reaction : reactor.add_queue) {
                for (auto& watch : reaction->watches) {
                    int err = ::uv_fs_event_init(async_handle->loop, watch.second.handle.get());
                    if (err < 0) {
                        reactor.log<NUClear::ERROR>(fmt::format("uv_fs_event_init error for '{}': ({}) {}",
                                                                (reaction->path / watch.second.path).string(),
                                                                err,
                                                                ::uv_strerror(err)));
                        watch.second.active = false;
                        reactor.remove_queue.push_back(std::move(watch.second.handle));
                        error = true;
                    }
                    else {
                        err = ::uv_fs_event_start(watch.second.handle.get(),
                                                  &FileWatcher::file_watch_callback,
                                                  (reaction->path / watch.second.path).c_str(),
                                                  UV_FS_EVENT_RECURSIVE);
                        if (err < 0) {
                            reactor.log<NUClear::ERROR>(fmt::format("uv_fs_event_start error for '{}': ({}) {}",
                                                                    (reaction->path / watch.second.path).string(),
                                                                    err,
                                                                    ::uv_strerror(err)));
                            watch.second.active = false;
                            reactor.remove_queue.push_back(std::move(watch.second.handle));
                            error = true;
                        }
                        else {
                            watch.second.active = true;
                        }
                    }
                }
            }
            reactor.add_queue.clear();

            // If there were errors trigger the removal of the failed watches
            if (error) {
                auto err = ::uv_async_send(reactor.remove_watch.get());
                if (err < 0) {
                    reactor.log<NUClear::ERROR>(
                        fmt::format("uv_async_send returned error {}: {}", err, ::uv_strerror(err)));
                }
            }
        });
        add_watch->data = this;

        // Initialise our remove_watch event to remove watches
        ::uv_async_init(loop.get(), remove_watch.get(), [](uv_async_t* async_handle) {
            // Grab our reactor context back
            FileWatcher& reactor = *reinterpret_cast<FileWatcher*>(async_handle->data);

            // Lock our mutex as we are editing our data structure
            std::lock_guard<std::mutex> lock(reactor.watch_mutex);

            for (auto&& remove_event : reactor.remove_queue) {
                // Need to call `uv_close()` before memory is freed
                // Can't free memory until the close callback has been called, which is called asynchronously in the
                // next (libuv) loop iteration. It is fine to free the memory in the callback
                remove_event->data = &reactor.remove_queue;
                ::uv_close(reinterpret_cast<uv_handle_t*>(remove_event.get()), [](uv_handle_t* close_handle) {
                    auto& queue = *static_cast<std::vector<std::unique_ptr<uv_fs_event_t>>*>(close_handle->data);
                    std::erase_if(queue, [&close_handle](const auto& h) {
                        return reinterpret_cast<uv_handle_t*>(h.get()) == close_handle;
                    });
                });
            }
        });
        remove_watch->data = this;

        on<Always>().then("FileWatcher", [this] {
            if (first_loop) {
                // The first time run with no wait so if there are no events we won't get stuck
                ::uv_run(loop.get(), UV_RUN_NOWAIT);
                first_loop = false;
                emit(std::make_unique<::extension::FileWatcherReady>());
            }
            else {
                // Run our event loop
                int ret = ::uv_run(loop.get(), UV_RUN_DEFAULT);

                // Close all handles
                if (ret != 0) {
                    ::uv_walk(
                        loop.get(),
                        [](uv_handle_t* handle, void* /* arg */) { uv_close(handle, nullptr); },
                        nullptr);
                    ::uv_run(loop.get(), UV_RUN_DEFAULT);
                }
            }
        });

        // Shutdown with the system
        on<Shutdown>().then("Shutdown FileWatcher", [this] {
            // Send an event to shutdown
            auto err = ::uv_async_send(shutdown.get());
            if (err < 0) {
                log<NUClear::ERROR>(fmt::format("uv_async_send returned error {}: {}", err, ::uv_strerror(err)));
            }
        });

        on<Trigger<Unbind>>().then("Unbind FileWatch", [this](const Unbind& fw) {
            // Lock our mutex as we are editing our datastructure
            std::lock_guard<std::mutex> lock(watch_mutex);

            // Find the reaction to unbind
            for (auto& watch : watches) {
                if (watch.first->id == fw.id) {
                    // Unwatch all paths for this reaction
                    for (auto& w : watch.second.watches) {
                        log<NUClear::DEBUG>(fmt::format("Removing watch for '{}'", w.second.path.string()));
                        remove_queue.push_back(std::move(w.second.handle));
                    }

                    // Trigger the stopping of the watches
                    if (!remove_queue.empty()) {
                        auto err = ::uv_async_send(remove_watch.get());
                        if (err < 0) {
                            log<NUClear::ERROR>(
                                fmt::format("uv_async_send returned error {}: {}", err, ::uv_strerror(err)));
                        }
                    }

                    // Remove this reaction->path map
                    watches.erase(watch.first);

                    // We are done no need to look further
                    break;
                }
            }
        });

        on<Trigger<FileWatchRequest>>().then("Add FileWatch", [this](const FileWatchRequest& req) {
            // Get the real path
            std::filesystem::path path = std::filesystem::absolute(req.path);
            log<NUClear::DEBUG>(fmt::format("Adding watch for {}", path.string()));

            std::filesystem::path current_directory{std::filesystem::current_path()};
            std::regex re{"", std::regex_constants::ECMAScript};
            bool recursive = false;

            // Watching every file in a directory, assumed to be a recursive watch
            if (std::filesystem::is_directory(path)) {
                current_directory = std::filesystem::absolute(path);
                re                = std::regex("[^\\/]*?", std::regex_constants::ECMAScript);
                recursive         = true;
            }
            // Watching a specific file
            else if (std::filesystem::is_regular_file(path)) {
                current_directory = std::filesystem::absolute(path.parent_path());
                re                = std::regex(path.filename().string(), std::regex_constants::ECMAScript);
                recursive         = false;
            }
            else {
                // Check for a glob pattern and translate it to a regex pattern
                const std::string glob = path;

                // Check for a recursive glob pattern
                const size_t recursive_start = glob.find("/**/");
                recursive                    = recursive_start != std::string::npos;

                // Not allowed to have more than one recursive patterns in the glob
                if (recursive && glob.find("/**/", recursive_start + 4) != std::string::npos) {
                    throw std::runtime_error("Two recursive patterns in glob not allowed");
                }

                // Split glob pattern up into base folder(s) and glob pattern and translate glob pattern into a regex
                // pattern
                std::string re_str;
                if (recursive) {
                    current_directory =
                        std::filesystem::absolute(std::filesystem::path(glob.substr(0, recursive_start)));
                    re_str = translate(glob.substr(recursive_start + 4));
                }
                else {
                    current_directory = std::filesystem::absolute(std::filesystem::path(glob).parent_path());
                    re_str            = translate(std::filesystem::path(glob).filename());
                }

                re = std::regex(re_str, std::regex_constants::ECMAScript);
            }

            std::vector<std::pair<std::shared_ptr<NUClear::threading::Reaction>, FileWatch>> new_watches{};
            /* mutex scope */ {
                // Lock our mutex as we are editing our data structure
                std::lock_guard<std::mutex> lock(watch_mutex);

                if (!watches.contains(req.reaction)) {
                    auto& p     = watches[req.reaction];
                    p.re        = re;
                    p.path      = current_directory;
                    p.recursive = recursive;
                    p.events    = req.events;

                    // Always watch the current directory
                    std::vector<std::filesystem::path> paths{"."};
                    if (recursive) {
                        // Find all of the directories under the current directory
                        list_directories(current_directory, paths);
                    }

                    // Add the watches
                    for (const auto& path : paths) {
                        auto& watch        = p.watches[path];
                        watch.path         = path;
                        watch.handle       = std::make_unique<uv_fs_event_t>();
                        watch.handle->data = this;
                        watch.active       = false;
                    }

                    // Find all of the files that are being watched and send the initial emit for them
                    std::vector<std::filesystem::path> files;
                    list_files(p.path, p.re, p.recursive, files);

                    // Add the discovered files to the known files
                    p.files.insert(files.begin(), files.end());

                    // Record data so that an initial emit can be made for each known file
                    new_watches.resize(p.files.size());
                    std::transform(p.files.begin(), p.files.end(), new_watches.begin(), [&](const auto& file) {
                        return std::make_pair(req.reaction,
                                              extension::FileWatch{(p.path / file), ::extension::FileWatch::NO_OP});
                    });

                    add_queue.push_back(&p);
                    auto err = ::uv_async_send(add_watch.get());
                    if (err < 0) {
                        log<NUClear::ERROR>(
                            fmt::format("uv_async_send returned error {}: {}", err, ::uv_strerror(err)));
                    }
                }
            }

            // Initial emit for each new file being watched
            for (auto& watch : new_watches) {
                // Store our watch value in the local cache
                FileWatch::FileWatchStore::value = &watch.second;

                // Directly execute our reaction here
                powerplant.submit(watch.first->get_task(true));

                // Clear our local cache
                FileWatch::FileWatchStore::value = nullptr;
            }
        });

        // Must be last to ensure that libuv has already been initialised
        // If libuv is not already initialised then the resulting watch on this config file won't work
        on<Configuration>("FileWatcher.yaml").then("Configuration", [this](const Configuration& cfg) {
            // Set log level for this module
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });
    }

    FileWatcher::~FileWatcher() {
        ::uv_loop_close(loop.get());
    }

}  // namespace module::extension
