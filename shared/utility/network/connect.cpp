#include "connect.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <fmt/format.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <nuclear>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <system_error>
#include <unistd.h>


namespace utility::network {

    utility::file::FileDescriptor connect(const std::string& host, const uint16_t& port) {

        // Work out what type of connection to make
        auto sock = NUClear::util::network::resolve(host, port);

        // Open a TCP socket
        utility::file::FileDescriptor fd(::socket(sock.sock.sa_family, SOCK_STREAM, IPPROTO_TCP),
                                         [](int fd) { ::shutdown(fd, SHUT_RDWR); });
        if (!fd.valid()) {
            throw std::system_error(errno, std::system_category(), fmt::format("Unable to open a TCP socket"));
        }

        // Set keepalive on the TCP socket
        int yes = 1;
        if (::setsockopt(fd.get(), SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes)) < 0) {
            throw std::system_error(errno, std::system_category(), "Unable to enable keepalive on the socket");
        }

        // Connect the socket to the specified ip_address:port
        if (::connect(fd.get(), &sock.sock, sizeof(sock)) != 0) {
            throw std::system_error(errno,
                                    std::system_category(),
                                    fmt::format("Unable to connect to remote host at {}:{}", host, port));
        }

        // Set the socket to nonblocking
        int flags = ::fcntl(fd.get(), F_GETFL, 0);
        if (flags < 0) {
            throw std::system_error(errno,
                                    std::system_category(),
                                    fmt::format("Cannot get socket flags {}:{}", host, port));
        }
        if (::fcntl(fd.get(), F_SETFL, flags | O_NONBLOCK) != 0) {
            throw std::system_error(errno,
                                    std::system_category(),
                                    fmt::format("Cannot set socket flags {}:{}", host, port));
        }

        return fd;
    }

}  // namespace utility::network
