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

#ifndef MODULES_EXTENSION_FILEWATCHER_HPP
#define MODULES_EXTENSION_FILEWATCHER_HPP

#include <filesystem>
#include <map>
#include <nuclear>
#include <regex>
#include <uv.h>
#include <vector>

namespace module::extension {

    class FileWatcher : public NUClear::Reactor {
    private:
        struct WatchMap {
            /// File/directory, relative to the parent directory, being watched by this handle
            std::filesystem::path path{};
            /// Handle for filesystem events
            std::unique_ptr<uv_fs_event_t> handle{nullptr};
            /// Indicates that the handle is actively watching the path
            bool active{false};
        };

        struct Watch {
            /// List of files currently known about
            std::set<std::filesystem::path> files{};
            /// The files/directories that are being watched under this path
            std::map<std::filesystem::path, WatchMap> watches{};
            /// Directory being watched
            std::filesystem::path path{"."};
            /// Regular expression for filtering file events
            std::regex re{"", std::regex_constants::ECMAScript};
            /// Indicates whether the watched directory also includes all sub-directories
            bool recursive{false};
            /// The events that this reaction is interested in
            int events{0};
        };

        /// The storage for paths
        std::mutex watch_mutex;
        std::map<std::shared_ptr<NUClear::threading::Reaction>, Watch> watches;

        /// The event queue for adding watches
        std::vector<Watch*> add_queue;
        std::vector<std::unique_ptr<uv_fs_event_t>> remove_queue;

        /// The libuv event loop
        std::unique_ptr<uv_loop_t> loop;
        /// The libuv async handle for adding new watches
        std::unique_ptr<uv_async_t> add_watch;
        /// The libuv async handle for removing watches
        std::unique_ptr<uv_async_t> remove_watch;
        /// The libuv async handle for triggering shutdown
        std::unique_ptr<uv_async_t> shutdown;

        /// True on the first loop then turns false after the FileWatcherReady event is emitted
        bool first_loop = true;

    public:
        /// @brief Called by the powerplant to build and setup the FileWatcher reactor.
        explicit FileWatcher(std::unique_ptr<NUClear::Environment> environment);
        FileWatcher(const FileWatcher&)            = delete;
        FileWatcher(FileWatcher&&)                 = delete;
        FileWatcher& operator=(const FileWatcher&) = delete;
        FileWatcher& operator=(FileWatcher&&)      = delete;
        ~FileWatcher() override;
        static void file_watch_callback(uv_fs_event_t* handle, const char* filename, int events, int status);
    };

}  // namespace module::extension

#endif  // MODULES_EXTENSION_FILEWATCHER_HPP
