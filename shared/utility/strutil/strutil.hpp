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

#ifndef UTILITY_STRUTIL_HPP
#define UTILITY_STRUTIL_HPP

#include <algorithm>
#include <string>


/**
 * @author Trent Houliston
 * @author Alex Biddulph: added vector version of endsWith ans startsWith 01-06-2021
 * @author Alex Biddulph: added trimLeft, trimRight, trim, and removeAll functions 10-12-2012
 * @author Monica Olejniczak added split functions
 * @author Brendan Annable
 */
namespace utility::strutil {

    // http://stackoverflow.com/a/874160/1387006
    inline bool endsWith(const std::string& str, const std::string& ending) {

        if (str.length() >= ending.length()) {
            return (0 == str.compare(str.length() - ending.length(), ending.length(), ending));
        }
        return false;
    }
    inline bool endsWith(const std::string& str, const std::vector<std::string>& endings) {

        for (const auto& ending : endings) {
            if (endsWith(str, ending)) {
                return true;
            }
        }
        return false;
    }

    inline bool startsWith(const std::string& str, const std::string& start) {

        if (str.length() >= start.length()) {
            return (0 == str.compare(0, start.length(), start));
        }
        return false;
    }
    inline bool startsWith(const std::string& str, const std::vector<std::string>& starts) {

        for (const auto& start : starts) {
            if (startsWith(str, start)) {
                return true;
            }
        }
        return false;
    }

    inline void trimLeft(std::string& str, const std::string& tokens) {
        str.erase(0, str.find_first_not_of(tokens));  // remove tokens from the beginning of the string.
    }

    inline void trimRight(std::string& str, const std::string& tokens) {
        str.erase(str.find_last_not_of(tokens),
                  str.length() - 1);  // remove tokens from the beginning of the string.
    }

    inline void trim(std::string& str, const std::string& tokens) {
        trimLeft(str, tokens);
        trimRight(str, tokens);
    }

    // http://stackoverflow.com/a/237280
    inline std::vector<std::string>& split(const std::string& str, char delimeter, std::vector<std::string>& elements) {
        std::stringstream ss(str);
        std::string item;
        while (std::getline(ss, item, delimeter)) {
            elements.push_back(item);
        }
        return elements;
    }

    inline std::vector<std::string> split(const std::string& string, char delimeter) {
        std::vector<std::string> elements;
        split(string, delimeter, elements);
        return elements;
    }

    inline void removeAll(std::string& str, const std::string& tokens) {
        str.erase(std::remove_if(str.begin(),
                                 str.end(),
                                 [&tokens](const char& c) { return tokens.find(c) != std::string::npos; }),
                  str.end());  // remove all tokens from the string.
    }

    inline std::string toUpper(const std::string& input) {
        std::string output = input;

        std::transform(output.begin(), output.end(), output.begin(), ::toupper);

        return output;
    }
}  // namespace utility::strutil

#endif
