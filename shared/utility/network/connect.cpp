/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
