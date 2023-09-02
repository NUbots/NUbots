#ifndef UTILITY_TCP_Connect_HPP
#define UTILITY_TCP_Connect_HPP

#include <fmt/format.h>
#include <nuclear>
#include <string>
#include <system_error>
#include <type_traits>
#include <vector>

#include "utility/TCP/Connection.hpp"

namespace utility::TCP {

    static inline void ipv4_to_ipv6(in6_addr& destination, const in_addr& address) {
        // TODO check that setting the first 16 bits is correct for ipv4 in ipv6.
        memcpy(&destination, &address, sizeof(in_addr_t));
    }

    /**
     * @brief Connects to a tcp server
     *
     * @param server_address The address of the server
     * @param server_port The port of the server
     * @return The details of the connection
     */
    inline Connection connect(const std::string server_address, const std::string server_port) {
        // Hints for the connection type
        addrinfo hints{};
        memset(&hints, 0, sizeof(addrinfo));  // Defaults on what we do not explicitly set
        hints.ai_family   = AF_UNSPEC;        // IPv4 or IPv6
        hints.ai_socktype = SOCK_STREAM;      // TCP

        // Store the ip address information that we will connect to
        addrinfo* address = nullptr;

        const int error = getaddrinfo(server_address.c_str(), server_port.c_str(), &hints, &address);
        if (error != 0) {
            throw std::system_error(network_errno,
                                    std::system_category(),
                                    fmt::format("Cannot resolve server name: {}. Error {}. Error code {}",
                                                server_address,
                                                gai_strerror(error),
                                                error));
        }

        // Loop through the linked list of potential options for connecting. In order of best to worst.
        for (addrinfo* addr_ptr = address; addr_ptr != nullptr; addr_ptr = addr_ptr->ai_next) {
            const int fd = socket(addr_ptr->ai_family, addr_ptr->ai_socktype, addr_ptr->ai_protocol);
            if (fd == -1) {
                // Bad fd
                continue;
            }
            if (connect(fd, addr_ptr->ai_addr, addr_ptr->ai_addrlen) != -1) {
                // Connection successful
                // Gather required information before we free the address information
                in6_addr connected_addr  = {0};
                in_port_t connected_port = 0;
                switch (addr_ptr->ai_addr->sa_family) {
                    case AF_INET:
                        ipv4_to_ipv6(connected_addr, reinterpret_cast<sockaddr_in*>(addr_ptr->ai_addr)->sin_addr);
                        connected_port = ntohs(reinterpret_cast<sockaddr_in*>(addr_ptr->ai_addr)->sin_port);
                        break;
                    case AF_INET6:
                        connected_addr = reinterpret_cast<sockaddr_in6*>(addr_ptr->ai_addr)->sin6_addr;
                        connected_port = ntohs(reinterpret_cast<sockaddr_in6*>(addr_ptr->ai_addr)->sin6_port);
                        break;
                    default:
                        throw std::system_error(network_errno,
                                                std::system_category(),
                                                "Socket Address faimily was not AF_INET or AF_INET6");
                }
                freeaddrinfo(address);
                return Connection(fd, connected_addr, connected_port);
            }
            // Connection was not successful
            close(fd);
        }

        // No connection was successful
        freeaddrinfo(address);
        throw std::system_error(network_errno,
                                std::system_category(),
                                fmt::format("Cannot connect to server: {}:{}", server_address, server_port));
    }
}  // namespace utility::TCP

#endif  // UTILITY_TCP_Connect_HPP
