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
#include "Lyrics2Logs.hpp"

#include "extension/Configuration.hpp"

#include "message/support/logging/Lyrics2Logs.hpp"

namespace module::support::logging {

    using extension::Configuration;
    using message::support::logging::LyricsPlayTime;

    Lyrics2Logs::Lyrics2Logs(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Lyrics2Logs.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Lyrics2Logs.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Read the songs
            for (auto& songNode : config["songs"].as<std::vector<YAML::Node>>()) {
                Song song;
                song.title  = songNode["title"].as<std::string>();
                song.artist = songNode["artist"].as<std::string>();

                for (auto& lineNode : songNode["lines"].as<std::vector<YAML::Node>>()) {
                    LyricsLine line;
                    line.start_time_ms = lineNode["start_time_ms"].as<int>();
                    if (lineNode["section"]) {
                        line.section = lineNode["section"].as<std::string>();
                    }
                    if (lineNode["by"]) {
                        line.by = lineNode["by"].as<std::string>();
                    }
                    line.words = lineNode["words"].as<std::string>();

                    song.lines[line.start_time_ms] = line;
                }

                this->songs[song.title] = song;
            }

            /*
            // Log the songs
            for (auto& [title, song] : songs) {
                log<NUClear::TRACE>(title, "by", song.artist);
                for (auto& [start_time_ms, line] : song.lines) {
                    if (!line.section.empty()) {
                        log<NUClear::INFO>(line.section);
                    }
                    if (!line.by.empty()) {
                        log<NUClear::INFO>(line.by);
                    }
                    log(start_time_ms, "ms", line.words);
                }
            }
            */
        });

        // Setup a ticker to advance playback time
        on<Every<10, std::chrono::milliseconds>>().then([this] {
            auto msg             = std::make_unique<LyricsPlayTime>();
            msg->current_time_ms = this->current_play_time_ms;
            emit(msg);

            this->current_play_time_ms += 10;
        });

        std::vector<std::string> songs = {
            "Never Gonna Give You Up"};  // "Last Night I Had A Dream About A Hummingbird", "Strange Game", "Non-Stop"};

        // Setup individual reactions for each song
        for (auto& song_title : songs) {
            on<Trigger<LyricsPlayTime>>().then(song_title, [this, song_title](const LyricsPlayTime& msg) {
                Song song = this->songs.at(song_title);

                if (msg.current_time_ms == 0) {
                    log<NUClear::DEBUG>(song.title, "by", song.artist);
                    return;
                }

                if (!song.lines.count(msg.current_time_ms)) {
                    return;
                }

                LyricsLine line = song.lines.at(msg.current_time_ms);

                if (!line.section.empty()) {
                    log<NUClear::INFO>(line.section);
                }
                if (!line.by.empty()) {
                    log<NUClear::INFO>(line.by);
                }

                log<NUClear::WARN>(line.words);
            });
        }
    }

}  // namespace module::support::logging
