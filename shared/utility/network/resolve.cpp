#include "resolve.hpp"

#include <fmt/format.h>

namespace utility::network {

    NUClear::util::network::sock_t resolve(const std::string& address, const uint16_t& port) {
        addrinfo hints{};
        memset(&hints, 0, sizeof hints);  // make sure the struct is empty
        hints.ai_family   = AF_UNSPEC;    // don't care about IPv4 or IPv6
        hints.ai_socktype = SOCK_STREAM;  // using a tcp stream

        // Get our info on this address
        addrinfo* servinfo_ptr = nullptr;
        if (::getaddrinfo(address.c_str(), std::to_string(port).c_str(), &hints, &servinfo_ptr) != 0) {
            throw std::runtime_error(fmt::format("Failed to get address information for {}:{}", address, port));
        }

        // Check if we have any addresses to work with
        if (servinfo_ptr == nullptr) {
            throw std::runtime_error(fmt::format("Unable to find an address for {}:{}", address, port));
        }

        std::unique_ptr<addrinfo, void (*)(addrinfo*)> servinfo(servinfo_ptr, ::freeaddrinfo);

        // Clear our struct
        NUClear::util::network::sock_t target{};
        std::memset(&target, 0, sizeof(target));

        // The list is actually a linked list of valid addresses
        // The address we choose is in the following priority, IPv4, IPv6, Other
        for (addrinfo* p = servinfo.get(); p != nullptr; p = p->ai_next) {

            // If we find an IPv4 address, prefer that
            if (servinfo->ai_family == AF_INET) {

                // Clear and set our struct
                std::memcpy(&target, servinfo->ai_addr, servinfo->ai_addrlen);

                // We prefer IPv4 so use it and stop looking
                break;
            }

            // If we find an IPv6 address, hold the first one in case we don't find an IPv4 address
            if (target.sock.sa_family == AF_UNSPEC && servinfo->ai_family == AF_INET6) {

                // Clear and set our struct
                std::memcpy(&target, servinfo->ai_addr, servinfo->ai_addrlen);
            }
        }

        // Found addresses, but none of them were IPv4 or IPv6
        if (target.sock.sa_family == AF_UNSPEC) {
            throw std::runtime_error(fmt::format("Unable to find an address for {}:{}", address, port));
        }

        return target;
    }

}  // namespace utility::network
