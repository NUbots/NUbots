/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#ifndef MODULE_SUPPORT_LOGGING_LYRICS2LOGS_HPP
#define MODULE_SUPPORT_LOGGING_LYRICS2LOGS_HPP

#include <nuclear>

namespace module::support::logging {

    struct LyricsLine {
        int start_time_ms;
        std::string section;
        std::string by;
        std::string words;
    };

    struct Song {
        std::string title;
        std::string artist;
        std::map<int, LyricsLine> lines;
    };

    class Lyrics2Logs : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

        std::map<std::string, Song> songs;
        int current_play_time_ms = 0;

    public:
        /// @brief Called by the powerplant to build and setup the Lyrics2Logs reactor.
        explicit Lyrics2Logs(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::support::logging

#endif  // MODULE_SUPPORT_LOGGING_LYRICS2LOGS_HPP
