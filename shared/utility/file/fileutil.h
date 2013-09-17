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

#ifndef UTILITY_FILEUTIL_H
#define UTILITY_FILEUTIL_H

#include <string>
#include <vector>

namespace utility {
    /**
     * TODO document
     *
     * @author Jake Woods
     * @author Trent Houliston
     */
    namespace file {
        std::string loadFromFile(const std::string& path);

        void writeToFile(const std::string& path, const std::string& data, bool append = false);

        bool exists(const std::string& path);

        bool isDir(const std::string& path);

        std::vector<std::string> listDir(const std::string& path);
    }
}
#endif
