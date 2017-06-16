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

#ifndef MODULES_EXTENSION_FILEWATCHER_H
#define MODULES_EXTENSION_FILEWATCHER_H

#include <nuclear>
#include <libfswatch/c++/monitor.hpp>

namespace module {
namespace extension {

    // This is the callback that is used to handle file system events
    void fswatchCallback(const std::vector<fsw::event>& events, void* reactor);

    class FileWatcher : public NUClear::Reactor {
    private:
        std::map<std::string, std::map<std::string, std::vector<std::pair<std::shared_ptr<NUClear::threading::Reaction>, int>>>> handlers;
        std::mutex runMutex;
        std::unique_ptr<fsw::monitor> monitor;
    public:
        /// @brief Called by the powerplant to build and setup the FileWatcher reactor.
        explicit FileWatcher(std::unique_ptr<NUClear::Environment> environment);

        friend void fswatchCallback(const std::vector<fsw::event>& events, void* reactor);
    };


}  // extension
}  // module

#endif  // MODULES_EXTENSION_FILEWATCHER_H
