#ifndef UTILITY_TCP_Connection_HPP
#define UTILITY_TCP_Connection_HPP

namespace utility::TCP {
    using NUClear::fd_t;
    ///@brief An RAII object for connection to a server
    struct Connection {

        struct {
            in6_addr address;
            in_port_t port;
        } remote;

        struct {
            in6_addr address;
            in_port_t port;
        } local;

        ///@brief The internal connection file descriptor
        fd_t fd;

        ///@brief Checks if the file descriptor is valid
        operator bool() const {
            return fd > 0;
        }

        /**
         * @brief Releases the internal file descriptor and gives up ownership
         */
        [[nodiscard]] fd_t release() {
            fd_t tmp_fd = fd;
            fd          = 0;
            return tmp_fd;
        }

        /**
         * @brief Creates a connection object
         *
         * @param fd_ The file descriptor for the connection
         * @param remote_address The address of the server connected to
         * @param remote_port The port of the server connected to
         */
        Connection(const fd_t fd_, const in6_addr remote_address, const in_port_t remote_port) : fd(fd_) {
            remote.address = remote_address;
            remote.port    = remote_port;
        }

        /// @brief Shutdown and close the internal file descriptor
        ~Connection() {
            if (fd > 0) {
                shutdown(fd, SHUT_RDWR);
                close(fd);
            }
        }

        // Prevent shallow copies
        Connection(const Connection&) = delete;
        Connection& operator=(const Connection&) = delete;
        // Move the file descriptor
        Connection(Connection&& o) {
            remote = o.remote;
            local  = o.local;
            fd     = o.fd;
            o.fd   = 0;
        }
        Connection& operator=(Connection&& o) {
            remote = o.remote;
            local  = o.local;
            fd     = o.fd;
            o.fd   = 0;
            return *this;
        }
    };
}  // namespace utility::TCP

#endif  // UTILITY_TCP_Connection_HPP
