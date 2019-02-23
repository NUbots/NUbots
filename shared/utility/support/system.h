/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2017 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_SUPPORT_SYSTEM_H
#define UTILITY_SUPPORT_SYSTEM_H

extern "C" {
#include <unistd.h>
}

#include <cstdlib>
#include <regex>
#include <string>
#include <system_error>

namespace utility {
namespace support {

    /**
     * Get the hostname of the system we are running on
     *
     * @author Alex Biddulph
     */
    inline std::string getHostname() {
        // Get hostname so we can find the correct per-robot script directory.
        char host[255];
        gethostname(host, 255);
        return host;
    }

    /**
     * Get the platform name of the system we are running on
     *
     * The hostname is assumed to be of the form <platform name><robot number>
     *
     * @param hostname The hostname to extract the platform name from
     *
     * @author Alex Biddulph
     */
    inline std::string getPlatform(const std::string& hostname) {
        // It is assumed that all hostnames are in the format <platform name><robot number>,
        // such that the regular expression
        // [a-z]+[0-9]+?
        // will match all hostnames
        std::regex re("([a-z]+)([0-9]+)?");
        std::smatch match;

        if (std::regex_match(hostname, match, re)) {
            // match[0] will be the full string
            // match[1] the first match (platform name)
            // match[2] the second match (robot number)
            return match[1].str();
        }

        else {
            throw std::system_error(
                -1, std::system_category(), (fmt::format("Failed to extract platform name from '{}'.", hostname)));
        }
    }

    inline std::string getPlatform() {
        return getPlatform(getHostname());
    }

}  // namespace support
}  // namespace utility

#endif  // UTILITY_SUPPORT_SYSTEM_H
