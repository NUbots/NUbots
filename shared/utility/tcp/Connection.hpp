#ifndef UTILITY_TCP_Connection_HPP
#define UTILITY_TCP_Connection_HPP

namespace  utility::TCP {
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

        Connection(fd_t fd_, in6_addr remote_address, in_port_t remote_port) : fd(fd_) {
            remote.address = remote_address;
            remote.port = remote_port;
        }

        ~Connection() {
            if(fd > 0){
                shutdown(fd, SHUT_RDWR);
                close(fd);
            }
        }

        // Prevent shallow copies
        Connection(const Connection&) = delete;
        Connection operator=(const Connection&) = delete;
        // Move the file descriptor
        Connection(Connection&& o){
            remote = o.remote;
            local = o.local;
            fd = o.fd;
            o.fd = 0;
        }
        Connection operator=(Connection&& o){
            remote = o.remote;
            local = o.local;
            fd = o.fd;
            o.fd = 0;
        }
    };
}

#endif // UTILITY_TCP_Connection_HPP
