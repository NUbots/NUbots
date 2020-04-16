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

#ifndef EXTENSION_FILEWATCH_H
#define EXTENSION_FILEWATCH_H

#include <nuclear>

namespace extension {

struct FileWatch {
    using FileWatchStore = NUClear::dsl::store::ThreadStore<FileWatch>;

    enum Event {
        NO_OP   = 0,  // No event
        RENAMED = 1,  // File created or renamed
        CHANGED = 2   // File modified
    };

    std::string path;
    int events;

    inline operator bool() const {
        // Empty path is invalid
        return !path.empty();
    }
};

struct FileWatchRequest {
    std::string path;
    int events;
    std::shared_ptr<NUClear::threading::Reaction> reaction;
};

}  // namespace extension

// NUClear configuration extension
namespace NUClear {
namespace dsl {
    namespace operation {
        template <>
        struct DSLProxy<::extension::FileWatch> {

            template <typename DSL>
            static inline void bind(const std::shared_ptr<threading::Reaction>& reaction,
                                    const std::string& path,
                                    int events) {

                // Add our unbinder
                reaction->unbinders.emplace_back([](const threading::Reaction& r) {
                    r.reactor.emit<word::emit::Direct>(
                        std::make_unique<operation::Unbind<::extension::FileWatch>>(r.id));
                });

                // Make a request to watch our file
                auto fw      = std::make_unique<::extension::FileWatchRequest>();
                fw->path     = path;
                fw->events   = events;
                fw->reaction = reaction;

                // Send our file watcher to the extension
                reaction->reactor.powerplant.emit<word::emit::Direct>(fw);
            }

            template <typename DSL>
            static inline ::extension::FileWatch get(threading::Reaction&) {

                // Get our File Watch store value
                auto ptr = ::extension::FileWatch::FileWatchStore::value;

                // If there was something in the store
                if (ptr) {
                    return *ptr;
                }
                // Return an invalid file watch element
                else {
                    return ::extension::FileWatch{"", 0};
                }
            }
        };
    }  // namespace operation

    // FileWatch is transient
    namespace trait {
        template <>
        struct is_transient<::extension::FileWatch> : public std::true_type {};
    }  // namespace trait
}  // namespace dsl
}  // namespace NUClear

#endif  // EXTENSION_FILEWATCH_H
