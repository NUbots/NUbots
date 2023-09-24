#ifndef UTILITY_NETWORK_RESOLVE_HPP
#define UTILITY_NETWORK_RESOLVE_HPP

#include <nuclear>

namespace utility::network {

    /**
     * @brief Resolves a hostname and port into a socket address
     *
     * @details
     *  This function will resolve a hostname and port into a socket address.
     *  It will return a socket address that can be used to connect to the specified host and port.
     *
     * @param address the hostname or IP address to resolve
     * @param port the port to connect to
     *
     * @return a socket address that can be used to connect to the specified host and port
     */
    NUClear::util::network::sock_t resolve(const std::string& address, const uint16_t& port);

}  // namespace utility::network

#endif  // UTILITY_NETWORK_RESOLVE_HPP
