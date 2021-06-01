#ifndef UTILITY_SUPPORT_HOSTNAME_HPP
#define UTILITY_SUPPORT_HOSTNAME_HPP

#include <string>
extern "C" {
#include <unistd.h>
}

namespace utility::support {

    inline std::string getHostname() {
        char* hostname = std::getenv("ROBOT_HOSTNAME");

        if (hostname == nullptr) {
            char buffer[255];
            ::gethostname(buffer, 255);
            hostname = buffer;
        }

        return std::string(hostname);
    }

}  // namespace utility::support

#endif  // UTILITY_SUPPORT_HOSTNAME_HPP
