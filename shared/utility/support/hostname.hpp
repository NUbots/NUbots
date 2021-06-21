#ifndef UTILITY_SUPPORT_HOSTNAME_HPP
#define UTILITY_SUPPORT_HOSTNAME_HPP

#include <string>
extern "C" {
#include <unistd.h>
}

namespace utility::support {

    inline std::string getHostname() {
        char* hostname = std::getenv("ROBOT_HOSTNAME");

        if (hostname != nullptr) {
            return std::string(hostname);
        }

        char buffer[255];
        ::gethostname(buffer, 255);
        return std::string(buffer);
    }

}  // namespace utility::support

#endif  // UTILITY_SUPPORT_HOSTNAME_HPP
