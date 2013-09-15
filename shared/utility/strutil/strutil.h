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

#ifndef UTILITY_STRUTIL_H
#define UTILITY_STRUTIL_H

#include <string>

namespace utility {

    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    namespace strutil {

        // http://stackoverflow.com/a/874160/1387006
        inline bool endsWith(const std::string& str, const std::string& ending) {

            if (str.length() >= ending.length()) {
                return (0 == str.compare (str.length() - ending.length(), ending.length(), ending));
            }
            else {
                return false;
            }
        }

        bool startsWith(const std::string& str, const std::string& start) {

            if (str.length() >= start.length()) {
                return (0 == str.compare (0, start.length(), start));
            }
            else {
                return false;
            }
        }
    }
}
#endif
