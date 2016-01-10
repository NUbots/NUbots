/*
 * This file is part of the NUbots Codebase.
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
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGE_SUPPORT_FILEWATCH_H_
#define MESSAGE_SUPPORT_FILEWATCH_H_

#include <nuclear>

namespace message {
    namespace support {

        struct FileWatch {
            using FileWatchStore = NUClear::dsl::store::ThreadStore<FileWatch>;

            enum Event {
                ACCESS        = 1,
                ATTRIBUTES    = 2,
                CLOSE_WRITE   = 4,
                CLOSE_NOWRITE = 8,
                CREATE        = 16,
                DELETE        = 32,
                DELETE_SELF   = 64,
                MODIFY        = 128,
                MOVE_SELF     = 256,
                MOVED_FROM    = 512,
                MOVED_TO      = 1024,
                OPEN          = 2048,
                IGNORED       = 4096,
                ISDIR         = 8192,
                UNMOUNT       = 16384
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
    }  // support
}  // message

// NUClear configuration extension
namespace NUClear {
    namespace dsl {
        namespace operation {
            template <>
            struct DSLProxy<::message::support::FileWatch> {

                template <typename DSL, typename TFunc>
                static inline threading::ReactionHandle bind(Reactor& reactor, const std::string& label, TFunc&& callback, const std::string& path, int events) {

                    // Generate our reaction
                    auto reaction = util::generate_reaction<DSL, ::message::support::FileWatch>(reactor, label, std::forward<TFunc>(callback));

                    // Make a request to watch our file
                    auto fw = std::make_unique<::message::support::FileWatchRequest>();
                    fw->path = path;
                    fw->events = events;
                    fw->reaction = std::move(reaction);

                    threading::ReactionHandle handle(fw->reaction);

                    // Send our file watcher to the extension
                    reactor.powerplant.emit<NUClear::dsl::word::emit::Direct>(fw);

                    // Return our handles
                    return handle;
                }

                template <typename DSL>
                static inline ::message::support::FileWatch get(threading::Reaction&) {

                    // Get our File Watch store value
                    auto ptr = ::message::support::FileWatch::FileWatchStore::value;

                    // If there was something in the store
                    if(ptr) {
                        return *ptr;
                    }
                    // Return an invalid file watch element
                    else {
                        return ::message::support::FileWatch { "", 0 };
                    }
                }
            };
        }

        // FileWatch is transient
        namespace trait {
            template <>
            struct is_transient<::message::support::FileWatch> : public std::true_type {};
        }
    }
}

#endif
