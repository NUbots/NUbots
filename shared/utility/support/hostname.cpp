#include "hostname.hpp"

namespace utility::support {

    std::string getHostname() {
        char* hostname = std::getenv("ROBOT_HOSTNAME");

        if (hostname == nullptr) {
            char buffer[255];
            ::gethostname(buffer, 255);
            hostname = buffer;
        }

        return std::string(hostname);
    }

}  // namespace utility::support
