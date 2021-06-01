#ifndef UTILITY_SUPPORT_HOSTNAME_HPP
#define UTILITY_SUPPORT_HOSTNAME_HPP

#include <string>
#include <unistd.h>

namespace utility::support {

    std::string getHostname();

}  // namespace utility::support

#endif  // UTILITY_SUPPORT_HOSTNAME_HPP
