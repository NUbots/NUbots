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
