#ifndef EXTENSION_TCP_Connection_HPP
#define EXTENSION_TCP_Connection_HPP

namespace  extension::TCP {
    struct Connection {
        struct {
            in6_addr address;
            in_port_t port;
        } remote;

        struct {
            in6_addr address;
            in_port_t port;
        } local;

        fd_t fd;

        operator bool() const {
            return fd != 0;
        }
    };
}

#endif // EXTENSION_TCP_Connection_HPP
