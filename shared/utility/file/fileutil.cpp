/*
 * This file is part of NUBots Utility.
 *
 * NUBots Utility is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots Utility is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots Utility.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

extern "C" {
    #include <sys/stat.h>
}

#include <sstream>
#include <fstream>

namespace utility {
namespace file {
    std::string loadFromFile(const std::string& path) {
        std::ifstream data(path, std::ios::in);

        // There are a lot of nice ways to read a file into a string but this is one of the quickest.
        // See: http://stackoverflow.com/a/116228
        std::stringstream stream;
        stream << data.rdbuf();

        return stream.str();
    }

    void writeToFile(const std::string& path, const std::string& data, bool append) {
        std::ofstream file(path,
            append
                ? std::ios::out | std::ios::app
                : std::ios::out);
        file << data;
    }

    bool exists(const std::string& path) {
        // Shamelessly stolen from: http://stackoverflow.com/a/12774387/1387006
        struct stat buffer;
        return (stat (path.c_str(), &buffer) == 0);
    }
}
}
