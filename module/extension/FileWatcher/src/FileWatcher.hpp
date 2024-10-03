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

#include <nuclear>
#include <uv.h>

namespace module::extension {

    class FileWatcher : public NUClear::Reactor {
    private:
        struct ReactionMap {
            // The reaction to run
            std::shared_ptr<NUClear::threading::Reaction> reaction;

            // The events it is interested in
            int events;
        };

        struct FileMap {
            // The reactions for this file
            std::vector<ReactionMap> reactions;
        };

        struct PathMap {
            // The files that are being watched, empty string is the directory
            std::map<std::string, FileMap> files;

            // The string for this path, given to the handle below
            std::string path;

            // The libuv handle for this folder
            std::unique_ptr<uv_fs_event_t> handle;
        };

        // The storage for paths
        std::map<std::string, PathMap> paths;
        std::mutex paths_mutex;

        // The event queue for adding and removing watches
        std::vector<PathMap*> add_queue;
        std::vector<std::unique_ptr<uv_fs_event_t>> remove_queue;

        // The libuv event loop
        std::unique_ptr<uv_loop_t> loop;
        std::unique_ptr<uv_async_t> add_watch;
        std::unique_ptr<uv_async_t> remove_watch;
        std::unique_ptr<uv_async_t> shutdown;

        /// True on the first loop then turns false after the FileWatcherReady event is emitted
        bool first_loop = true;

    public:
        /// @brief Called by the powerplant to build and setup the FileWatcher reactor.
        explicit FileWatcher(std::unique_ptr<NUClear::Environment> environment);
        ~FileWatcher() override;

        static void file_watch_callback(uv_fs_event_t* handle, const char* filename, int events, int status);

        // Delete the move and copy constructors and operators, because we only want one filewatcher per powerplant
        // and it shouldn't change (rule of 5)
        FileWatcher(FileWatcher& other)              = delete;
        FileWatcher(FileWatcher&& other)             = delete;
        FileWatcher& operator=(FileWatcher& other)   = delete;
        FileWatcher&& operator=(FileWatcher&& other) = delete;
    };

}  // namespace module::extension

#endif  // MODULES_EXTENSION_FILEWATCHER_HPP
