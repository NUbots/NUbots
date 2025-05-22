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

#include "translate.hpp"

#include <sstream>

namespace module::extension {

    std::string translate(const std::string& pattern) {
        // TODO(abiddulph): Potential upgrade
        //                  Handle globstar expressions
        //                  "/**/" ==> "((?:[^/]*(?:\/|$))*)"
        std::stringstream result;

        int curlies = 0;
        for (ssize_t i = 0; i < std::ssize(pattern); ++i) {
            ssize_t j       = 0;
            size_t closing  = 0;
            bool complement = false;
            switch (pattern[i]) {
                /* Escape characters that have no special meaning in globs but do have a special meaning in regex
                 *   ^$=(|)!+/.\
                 */
                case '^':
                case '$':
                case '=':
                case '(':
                case '|':
                case ')':
                case '!':
                case '+':
                case '/':
                case '.':
                case '\\': result << "\\" << pattern[i]; break;
                // Match any number of characters, including 0 characters, but not a /
                case '*': result << "[^\\/]*?"; break;
                // Match any single character, but not a /
                case '?': result << "[^\\/]"; break;
                // Matches any of the following
                //  1. [...] match any single character from the provided sequence, except for /
                //  2. [!...] matches any single character that is not provided in the sequence, excluding /
                //  3. [X-Y] matches any character in the range [X, Y], excluding /
                //        e.g [A-F] ==> [ABCDEF]
                //        e.g [--0] ==> [-.0] (since / cannot be matched)
                case '[': {
                    // Check for an empty sequence
                    if (pattern[i + 1] == ']') {
                        result << "\\[\\]";
                        i++;
                        break;
                    }
                    j = i + 1;
                    // We are complementing the following sequence
                    if (pattern[j] == '!') {
                        complement = true;
                        ++j;
                    }
                    // ']' is one of the characters in the sequence
                    if (pattern[j] == ']') {
                        ++j;
                    }
                    // Find the closing bracket for this sequence
                    closing = pattern.find(']', j);
                    if (closing == std::string::npos) {
                        throw std::runtime_error("Invalid pattern! Found no closing bracket for sequence starting at "
                                                 + std::to_string(i));
                    }

                    // Wrap sequence in a negative lookahead to ensure that a '/' isn't matched
                    // (?:(?![\/])<sequence>)
                    result << "(?:(?![\\/])[";
                    if (complement) {
                        result << "^";
                    }
                    result << pattern.substr(j, closing - j) << "])";

                    // Increment to the end of the current sequence
                    i = ssize_t(closing);

                    // Check for a quantifier immediately after the closing bracket
                    if (pattern[i + 1] == '*' || pattern[i + 1] == '+') {
                        result << pattern[++i];
                    }
                    break;
                }
                // Check for {...,...} options and convert to (...|...)
                case '{':
                    // Check for {}
                    if (pattern[i + 1] == '}') {
                        result << "\\{\\}";
                        i++;
                        break;
                    }
                    closing = pattern.find('}', i);
                    if (closing == std::string::npos) {
                        throw std::runtime_error("Invalid pattern! Found no closing brace for sequence starting at "
                                                 + std::to_string(i));
                    }
                    result << '(';
                    ++curlies;
                    break;
                case '}':
                    if (curlies > 0) {
                        --curlies;
                        result << ')';
                    }
                    else {
                        result << pattern[i];
                    }
                    break;
                case ',':
                    if (curlies > 0) {
                        result << '|';
                    }
                    else {
                        result << "\\" << pattern[i];
                    }
                    break;
                // Regular character, pass through unaltered
                default: result << pattern[i]; break;
            }
        }

        return result.str();
    }

}  // namespace module::extension
