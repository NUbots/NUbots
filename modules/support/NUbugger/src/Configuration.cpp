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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "NUbugger.h"

#include "utility/file/fileutil.h"

namespace modules {
namespace support {
    using utility::file::listDir;
    using utility::file::isDir;

    void NUbugger::sendConfigurationState() {
        std::vector<std::string> paths;             // a vector that contains all of the configuration file paths
        std::vector<std::string> directories;       // a vector that contains all the of the configuration directories
        directories.push_back("config");            // adds the config directory the the vector

        // loop through all the directories using a depth-first search
        while (!directories.empty()) {
            // retrieve the last directory in the vector, beginning with config
            auto directory = directories.back();
            // immediately remove the directory that was found
            directories.pop_back();
            // loop through every file within the directory
            for (auto&& file : listDir(directory)) {
                // specify the correct path within the directory
                auto path = directory + "/" + file;
                // check if the given path is a directory
                if (isDir(path)) {
                    // append the path to the directories vector
                    directories.push_back(path);
                } else {
                    // append the path to the paths vector
                    paths.push_back(path);
                }
            }
        }

        // search through config directory -> config/*.yaml accounting for sub-directories
        // parse yaml (load using yaml.cpp)
        // make protocol buffer tree and send over network to nubugger
    }
}
}