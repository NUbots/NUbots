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

#include "fileutil.h"

extern "C" {
    #include <sys/stat.h>
    #include <dirent.h>
}

#include <sstream>
#include <system_error>
#include <stack>

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

    bool exists(const std::string& path) {
        // Shamelessly stolen from: http://stackoverflow.com/a/12774387/1387006
        struct stat buffer;
        return (stat (path.c_str(), &buffer) == 0);
    }

    // Test if a passed path is a directory
    bool isDir(const std::string& path) {

        int status;
        struct stat st_buf;

        // Get the status of the file system object.
        status = stat(path.c_str(), &st_buf);
        if (status != 0) {
            throw std::system_error(errno, std::system_category(), "Error checking if path is file or directory");
        }

        // Return if our varible is a directory
        return S_ISDIR(st_buf.st_mode);
    }

    // List the contents of a directory
    std::vector<std::string> listDir(const std::string& path) {

        auto dir = opendir(path.c_str());
        std::vector<std::string> result;

        if(dir != nullptr) {
            for(dirent* ent = readdir(dir); ent != nullptr; ent = readdir(dir)) {

                auto file = std::string(ent->d_name);

                if(file == "." || file == "..") {
                    continue;
                }

                if(ent->d_type & DT_DIR) {
                    result.push_back(file + "/");
                }
                else {
                    result.push_back(file);
                }
            }

            closedir(dir);
        }
        else {
            // TODO Throw an error or something
        }

        return result;
    }

    std::pair<std::string, std::string> pathSplit(const std::string& input) {

        uint lastSlash = input.rfind('/');

        // There was no slash
        if(lastSlash == std::string::npos) {
            return { ".", input };
        }
        // The slash was the last character
        if(lastSlash + 1 == input.size()) {
            // If all we had was a slash
            if(input.size() == 1) {
                return { "/", "/" };
            }
            // Otherwise remove the slash and call recursivly
            else {
                return pathSplit(input.substr(0, input.size() - 1));
            }
        }
        else {
            return { input.substr(0, lastSlash), input.substr(lastSlash + 1, input.size()) };
        }
    }

    std::vector<std::string> listFiles(const std::string& directory, bool recursive) {
        // create a vector to store the files
        std::vector<std::string> files;
        // create a vector to store the directories
        std::stack<std::string> directories;
        // adds the specified directory to the vector
        directories.push(directory);
        // loop through all the directories using a depth-first search
        while (!directories.empty()) {
            // retrieve the last directory in the vector, beginning with the initial directory
            auto directory = directories.top();
            // immediately remove the directory that was found
            directories.pop();
            // loop through every file within the directory
            for (auto&& file : listDir(directory)) {
                // specify the correct path within the directory
                auto path = directory + "/" + file;
                // check if the given path is a directory
                if (isDir(path)) {
                    // check if the function is recursive
                    if (recursive) {
                        // append the path to the directories vector
                        directories.push(path);
                    }
                } else {
                    // append the path to the paths vector
                    files.push_back(path);
                }
            }
        }
        // return the list of files
        return files;
    }
}
}
