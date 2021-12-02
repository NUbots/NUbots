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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "fileutil.hpp"

extern "C" {
#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
}

#include <iostream>
#include <iterator>
#include <sstream>
#include <stack>
#include <system_error>

#include "utility/strutil/strutil.hpp"

namespace utility::file {
    std::pair<std::string, std::string> pathSplit(const std::string& input) {

        size_t lastSlash = input.rfind('/');

        // There was no slash
        if (lastSlash == std::string::npos) {
            return {".", input};
        }
        // The slash was the last character
        if (lastSlash + 1 == input.size()) {
            // If all we had was a slash
            if (input.size() == 1) {
                return {"/", "/"};
            }
            // Otherwise remove the slash and call recursivly
            return pathSplit(input.substr(0, input.size() - 1));
        }
        // Else, the slash was not the last character
        return {input.substr(0, lastSlash), input.substr(lastSlash + 1, input.size())};
    }

}  // namespace utility::file
