/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#include <functional>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

/**
 * @author Trent Houliston
 * @author Alex Biddulph: added vector version of ends_with ans startsWith 01-06-2021
 * @author Alex Biddulph: added trim_left, trim_right, trim, and remove_all functions 10-12-2012
 * @author Monica Olejniczak added split functions
 * @author Brendan Annable
 */
namespace utility::strutil {

    /**
     * @brief Checks if a string ends with a specific substring.
     *
     * This function compares the given string `str` with the specified `ending` substring
     * and determines whether `str` ends with `ending`. The comparison is case-sensitive.
     *
     * @param str The string to check for the ending substring.
     * @param ending The substring to search for at the end of the string.
     * @return `true` if `str` ends with `ending`, `false` otherwise.
     */
    // http://stackoverflow.com/a/874160/1387006
    [[nodiscard]] inline bool ends_with(const std::string& str, const std::string& ending) {
        return str.length() >= ending.length()
               && (0 == str.compare(str.length() - ending.length(), ending.length(), ending));
    }

    /**
     * @brief The overloaded version of ends_with which takes vectors of strings instead of single strings.
     *
     * This function compares the given string `str` with the specified `ending` substring in the vector endings
     * and determines whether `str` ends with `ending`. The comparison is case-sensitive.
     *
     * @param str The string to check for the ending substring.
     * @param ending The substring to search for at the end of the string.
     * @return `true` if `str` ends with `ending`, `false` otherwise.
     */
    // http://stackoverflow.com/a/874160/1387006
    [[nodiscard]] inline bool ends_with(const std::string& str, const std::vector<std::string>& endings) {

        for (const auto& ending : endings) {
            if (ends_with(str, ending)) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Checks if a string starts with a specific substring.
     *
     * This function compares the given string `str` with the specified `start` substring
     * and determines whether `str` starts with `start`. The comparison is case-sensitive.
     *
     * @param str The string to check for the starting substring.
     * @param start The substring to search for at the beginning of the string.
     * @return `true` if `str` starts with `start`, `false` otherwise.
     */
    [[nodiscard]] inline bool starts_with(const std::string& str, const std::string& start) {
        return str.length() >= start.length() && (0 == str.compare(0, start.length(), start));
    }

    /**
     * @brief Trims leading whitespace from a string.
     *
     * This function removes leading whitespace characters from the given string `str`
     * and returns the a new string without these characters.
     *
     * @param str The string to trim leading whitespace from.
     * @return The modified string with leading whitespace removed.
     */
    [[nodiscard]] inline std::string trim_left(const std::string& str) {
        std::string s = str;
        // remove whitespace tokens from the beginning of the string.
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
                    return std::isspace(static_cast<unsigned char>(ch)) == 0;
                }));
        return s;
    }

    /**
     * @brief Trims trailing whitespace from a string.
     *
     * This function removes trailing whitespace characters from the given string `str`
     * and returns a new string without these characters.
     *
     * @param str The string to trim trailing whitespace from.
     * @return The modified string with trailing whitespace removed.
     */
    [[nodiscard]] inline std::string trim_right(const std::string& str) {
        // remove whitespace tokens from the end of the string.
        std::string s = str;
        s.erase(
            std::find_if(s.rbegin(), s.rend(), [](int ch) { return std::isspace(static_cast<unsigned char>(ch)) == 0; })
                .base(),
            s.end());
        return s;
    }

    /**
     * @brief Trims leading and trailing whitespace from a string.
     *
     * This function removes leading and trailing whitespace characters from the given string `str`
     * and returns a new string without these characters.
     *
     * @param str The string to trim leading and trailing whitespace from.
     * @return The modified string with leading and trailing whitespace removed.
     */
    [[nodiscard]] inline std::string trim(const std::string& str) {
        std::string s = trim_left(str);
        return trim_right(s);
    }

    /**
     * @brief Splits a string into multiple substrings based on a delimiter.
     *
     * This function splits the given string `s` into multiple substrings using the specified `delimiter`.
     * The resulting substrings are stored in a vector and returned as the result.
     *
     * @param s The string to split.
     * @param delimiter The delimiter used to split the string.
     * @return A vector of substrings resulting from the split operation.
     */
    [[nodiscard]] inline std::vector<std::string> split(const std::string& s, const std::string& delimiter) {

        std::vector<std::string> res;

        // An empty string means return the original string as a single substring.
        if (s.empty()) {
            res.push_back(s);
            return res;
        }

        // An empty delimiter means split each character into a separate string.
        if (delimiter.empty()) {
            for (const char& c : s) {
                res.push_back(std::string(1, c));
            }
            return res;
        }

        size_t pos_start = 0;
        size_t pos_end   = 0;
        size_t delim_len = delimiter.length();
        std::string token;

        while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
            token     = s.substr(pos_start, pos_end - pos_start);
            pos_start = pos_end + delim_len;
            res.push_back(token);
        }

        res.push_back(s.substr(pos_start));
        return res;
    }

    /**
     * @brief Splits a string into multiple substrings based on a single character delimiter.
     *
     * This function splits the given string `s` into multiple substrings using the specified single character
     * `delimiter`. The resulting substrings are stored in a vector and returned as the result.
     *
     * @param s The string to split.
     * @param delimiter The single character delimiter used to split the string.
     * @return A vector of substrings resulting from the split operation.
     */
    [[nodiscard]] inline std::vector<std::string> split(const std::string& s, const char& delimiter) {
        return split(s, std::string(1, delimiter));
    }

    /**
     * @brief Removes all occurrences of specified characters from a string.
     *
     * This function removes all characters specified in the `tokens` string from the given `str`.
     * The modified string is returned as the result.
     *
     * @param str The string from which to remove characters.
     * @param tokens The string containing the characters to be removed.
     * @return The modified string with all specified characters removed.
     */
    [[nodiscard]] inline std::string remove_all(const std::string& str, const std::string& tokens) {
        std::string s = str;
        s.erase(std::remove_if(s.begin(),
                               s.end(),
                               [&tokens](const char& c) { return tokens.find(c) != std::string::npos; }),
                s.end());
        return s;
    }

    /**
     * @brief Converts a string to uppercase.
     *
     * This function takes the given `input` string and converts all characters to uppercase.
     * The modified string is returned as the result.
     *
     * @param input The string to convert to uppercase.
     * @return The modified string with all characters converted to uppercase.
     */
    [[nodiscard]] inline std::string to_upper(const std::string& input) {
        std::string output = input;

        std::transform(output.begin(), output.end(), output.begin(), ::toupper);

        return output;
    }

    /**
     * @brief Converts a string to lowercase.
     *
     * This function takes the given `input` string and converts all characters to lowercase.
     * The modified string is returned as the result.
     *
     * @param input The string to convert to lowercase.
     * @return The modified string with all characters converted to lowercase.
     */
    [[nodiscard]] inline std::string to_lower(const std::string& input) {
        std::string output = input;

        std::transform(output.begin(), output.end(), output.begin(), ::tolower);

        return output;
    }

    /**
     * @brief Checks if a string is one of the specified options.
     *
     * This function checks whether the given `str` is present in the `options` vector.
     * It returns `true` if `str` is found in `options`, and `false` otherwise.
     *
     * @param str The string to check.
     * @param options The vector of options to compare against.
     * @return `true` if `str` is found in `options`, `false` otherwise.
     */
    [[nodiscard]] inline bool is_one_of(const std::string& str, const std::vector<std::string>& options) {
        return std::find(options.begin(), options.end(), str) != options.end();
    }

    /**
     * @brief Joins elements of a vector into a single string using a delimiter.
     *
     * This function takes a vector `list` of elements and joins them into a single string,
     * using the specified `delimiter` between each element. The elements are converted to strings
     * using their stream insertion operator (<<).
     *
     * @tparam T The type of elements in the vector.
     * @param list The vector of elements to join.
     * @param delimiter The delimiter string used to separate the elements in the resulting string.
     * @return The joined string containing all elements of the vector.
     */
    template <typename T>
    [[nodiscard]] inline std::string join(const std::vector<T>& list, const std::string& delimiter) {
        std::stringstream stream;

        for (int i = 0; i < int(list.size()); ++i) {
            stream << list[i];
            if ((i + 1) < int(list.size())) {
                stream << delimiter;
            }
        }

        return stream.str();
    }

    /**
     * @brief Removes common leading whitespace from each line in a multiline string.
     *
     * This function finds the common leading whitespace from each line in the input string,
     * i.e., whitespace that is followed by a non-whitespace character. It then removes that
     * many characters from each input line and returns the resulting dedented string.
     *
     * @param input The multiline string to dedent.
     *
     * @return The dedented string.
     */
    inline std::string dedent(const std::string& input) {

        size_t min_leading             = std::numeric_limits<size_t>::max();
        std::vector<std::string> lines = split(input, "\n");
        for (const auto& l : lines) {
            // Count the amount of leading whitespace on non empty lines
            auto it     = std::find_if(l.begin(), l.end(), std::not_fn(std::iswspace));
            min_leading = it == l.end() ? min_leading : std::min(min_leading, size_t(std::distance(l.begin(), it)));
        }

        // Remove the common leading whitespace from each line
        for (auto& l : lines) {
            if (l.size() >= min_leading) {
                l.erase(0, min_leading);
            }
            else {
                l.clear();
            }
        }

        return join(lines, "\n");
    }

}  // namespace utility::strutil

#endif
