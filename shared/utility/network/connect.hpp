#ifndef UTILITY_NETWORK_TCP_CONNECT_HPP
#define UTILITY_NETWORK_TCP_CONNECT_HPP

#include <cstdint>
#include <string>

#include "utility/file/FileDescriptor.hpp"

namespace utility::network {

    /**
     * @brief Connect to a TCP socket
     *
     * @param host the host to connect to
     * @param port the port to connect to
     *
     * @return a file descriptor for the connected socket
     */
    utility::file::FileDescriptor connect(const std::string& host, const uint16_t& port);

}  // namespace utility::network

#endif  // UTILITY_NETWORK_TCP_CONNECT_HPP
