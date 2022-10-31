#ifndef EXTENSION_TCP_IO_HPP
#define EXTENSION_TCP_IO_HPP

#include <nuclear>
#include "utility/tcp/Connection.hpp"

namespace extension::TCP::IO {

    struct IO : public NUClear::dsl::word::IO {
        using utility::TCP::Connection;
        template <typename DSL>
        static inline void bind(const std::shared_ptr<threading::Reaction>& reaction, Connection connection, int watch_set) {
            bind(reaction, connection.fd, watch_set);
        }
    };
}

#endif // EXTENSION_TCP_IO_HPP
