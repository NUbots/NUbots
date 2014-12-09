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

#ifndef UTILITY_STRUTIL_H
#define UTILITY_STRUTIL_H

#include <string>
#include <algorithm>

namespace utility {

    /**
     * TODO document
     *
     * @author Trent Houliston
     * @author Alex Biddulph: added trimLeft, trimRight, trim, and removeAll functions 10-12-2012
     * @author Brendan Annable
     */
    namespace strutil {

        // see: http://en.wikipedia.org/wiki/ANSI_escape_code#Colors
        // see: http://stackoverflow.com/a/3219471/868679

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

        void trimLeft(std::string& str, const std::string& tokens) {
            str.erase(0, str.find_first_not_of(tokens));        // remove tokens from the beginning of the string.
        }

        void trimRight(std::string& str, const std::string& tokens) {
            str.erase(str.find_last_not_of(tokens), str.length() - 1);        // remove tokens from the beginning of the string.
        }

        void trim(std::string& str, const std::string& tokens) {
            trimLeft(str, tokens);
            trimRight(str, tokens);
        }

        void removeAll(std::string& str, const std::string& tokens) {
            str.erase(std::remove_if(str.begin(), str.end(),
                [&tokens](const char& c) {
                    return tokens.find(c) != std::string::npos;
                }
                ), str.end());        // remove all tokens from the string.
        }
    }
}
#endif
