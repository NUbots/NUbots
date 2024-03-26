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
