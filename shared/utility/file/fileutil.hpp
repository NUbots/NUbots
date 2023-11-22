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

#ifndef UTILITY_FILEUTIL_HPP
#define UTILITY_FILEUTIL_HPP

#include <array>
#include <chrono>
#include <fstream>
#include <string>
#include <vector>

/**
 * @author Jake Woods
 * @author Trent Houliston
 */
namespace utility::file {
    std::string loadFromFile(const std::string& path);
    std::vector<uint8_t> readFile(const std::string& path);

    template <typename TData>
    void writeToFile(const std::string& path, const TData& data, bool append = false) {
        std::ofstream file(path, append ? std::ios::out | std::ios::app : std::ios::out | std::ios::trunc);
        file << data;
    }

    bool exists(const std::string& path);

    std::chrono::system_clock::time_point getModificationTime(const std::string& path);

    bool isDir(const std::string& path);
    void makeDir(const std::string& path);

    std::vector<std::string> listDir(const std::string& path);

    /**
     * @brief Splits a path into it's basename and dirname components.
     *
     * @param input the input string
     *
     * @return the dirname and basename in the posix style
     */
    std::pair<std::string, std::string> pathSplit(const std::string& input);

    /**
     * @author Monica Olejniczak
     * @author Trent Houliston
     *
     * Finds and returns a list of file paths given a specified directory. This function is able to include any
     * sub-directories and their file paths if recursive is set to true.
     *
     * @param directory The directory to base the search off.
     * @param recursive Whether the directories within the specified directory is searched or not.
     * @return The list of file paths within a specified directory.
     */
    std::vector<std::string> listFiles(const std::string& directory, bool recursive = false);

    bool makeDirectory(const std::string& directory, bool parent = false);

    void touch(const std::string& file);

}  // namespace utility::file
#endif
