/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
    std::string loadFromFile(const std::string& path) {
        std::ifstream data(path, std::ios::in);

        // There are a lot of nice ways to read a file into a string but this is one of the quickest.
        // See: http://stackoverflow.com/a/116228
        std::stringstream stream;
        stream << data.rdbuf();

        return stream.str();
    }

    std::vector<uint8_t> readFile(const std::string& path) {
        std::ifstream data(path, std::ios::binary);

        // Stop eating new lines in binary mode
        data.unsetf(std::ios::skipws);

        // Get number of bytes in the file
        std::streampos num_bytes;
        data.seekg(0, std::ios::end);
        num_bytes = data.tellg();
        data.seekg(0, std::ios::beg);

        // Reserve capacity
        std::vector<uint8_t> vec;
        vec.reserve(num_bytes);

        // Read the data
        vec.insert(vec.begin(), std::istream_iterator<uint8_t>(data), std::istream_iterator<uint8_t>());

        return vec;
    }

    bool exists(const std::string& path) {
        // Shamelessly stolen from: http://stackoverflow.com/a/12774387/1387006
        struct stat buffer {};
        return (stat(path.c_str(), &buffer) == 0);
    }

    std::chrono::system_clock::time_point getModificationTime(const std::string& path) {
        int status = 0;
        struct stat st_buf {};

        // Get the status of the file system object.
        status = stat(path.c_str(), &st_buf);
        if (status != 0) {
            throw std::system_error(errno, std::system_category(), "Error checking if path is file or directory");
        }

        return (std::chrono::system_clock::from_time_t(st_buf.st_mtime));
    }

    // Test if a passed path is a directory
    bool isDir(const std::string& path) {

        int status = 0;
        struct stat st_buf {};

        // Get the status of the file system object.
        status = stat(path.c_str(), &st_buf);
        if (status != 0) {
            throw std::system_error(errno, std::system_category(), "Error checking if path is file or directory");
        }

        // Return if our varible is a directory
        return S_ISDIR(st_buf.st_mode);
    }

    void makeDir(const std::string& path) {
        int status = 0;
        status     = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

        if (status != 0) {
            throw std::system_error(errno, std::system_category(), "Error creating directory '" + path + "'.");
        }
    }

    // List the contents of a directory
    std::vector<std::string> listDir(const std::string& path) {

        auto* dir = opendir(path.c_str());
        std::vector<std::string> result;

        if (dir != nullptr) {
            for (dirent* ent = readdir(dir); ent != nullptr; ent = readdir(dir)) {

                auto file = std::string(ent->d_name);

                if (file == "." || file == "..") {
                    continue;
                }

                if ((ent->d_type & DT_DIR) != 0) {
                    result.push_back(file + "/");
                }
                else {
                    result.push_back(file);
                }
            }

            closedir(dir);
        }
        else {
            throw std::runtime_error("Attempted to list directory which didn't exist.");
        }

        return result;
    }

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
                }
                else {
                    // append the path to the paths vector
                    files.push_back(path);
                }
            }
        }
        // return the list of files
        return files;
    }

    bool makeDirectory(const std::string& directory, bool parent) {
        std::vector<std::string> elements;

        // Get elements of the path to create.
        if (parent) {
            elements         = utility::strutil::split(directory, '/');
            elements.front() = elements.front().empty() ? "/" : elements.front();
        }
        else {
            elements.push_back(directory);
        }

        std::string path;

        // Traverse all elements of the path.
        for (const auto& element : elements) {


            path.append(element);

            // If the current path doesn't exist, create it.
            if (!exists(path)) {
                // Create the current path element with the following permissions.
                // U = RWX
                // G = R_X
                // O = R_X
                auto status = mkdir(path.c_str(), S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

                // If we fail at any point then bail out.
                if (status != 0) {
                    return false;
                }
            }
            else if (!isDir(path)) {
                // THROW EXCEPTION!!!!
                // OK
                throw std::runtime_error("Cannot create a directory " + path + " is an ordinary file");
            }

            path.append("/");
        }

        return true;
    }

    // http://chris-sharpe.blogspot.com.au/2013/05/better-than-systemtouch.html
    void touch(const std::string& file) {
        int fd = open(file.c_str(), O_WRONLY | O_CREAT | O_NOCTTY | O_NONBLOCK, 0666);

        // Couldn't open that path.
        if (fd < 0) {
            throw std::runtime_error("Cannot open/create '" + file + "' with mode 0666.");
            return;
        }

        int rc = utimensat(AT_FDCWD, file.c_str(), nullptr, 0);

        // Failed to update timestamp.
        if (rc != 0) {
            throw std::runtime_error("Cannot update timestamp for '" + file + "'.");
            return;
        }
    }

}  // namespace utility::file
