/*
 * MIT License
 *
 * Copyright (c) 2020 NUbots
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

// Uncomment this line when other test files are added
#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <fmt/format.h>
#include <fstream>
#include <iostream>
#include <nuclear>

#include "FileWatcher.hpp"

#include "extension/FileWatch.hpp"

namespace {

    struct StartTest {};

    const char* const test_file_name = "file-watcher-test.txt";
    std::vector<extension::FileWatch> file_watch_events;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
    bool watchdog_triggered;                              // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

    class TestReactor : public NUClear::Reactor {
    public:
        TestReactor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            // GIVEN a file.
            std::filesystem::path dir_path = std::filesystem::temp_directory_path() / "file-watcher-test-XXXXXX";
            std::string dir_path_string    = dir_path.string();
            std::vector<char> dir_path_chars(dir_path_string.c_str(),
                                             dir_path_string.c_str() + dir_path_string.size() + 1);
            dir_path = mkdtemp(dir_path_chars.data());

            std::filesystem::path file_path = dir_path / test_file_name;
            { std::ofstream(file_path.string()); }  // Create file and close stream at end of scope

            on<extension::FileWatch>(file_path.string(), extension::FileWatch::CHANGED)
                .then([this, file_path](const extension::FileWatch& message) {
                    UNSCOPED_INFO("File change event: " << message.path);
                    file_watch_events.push_back(message);

                    if (file_watch_events.size() >= 2) {
                        powerplant.shutdown();
                    }
                });

            on<Trigger<extension::FileWatcherReady>>().then([this] { emit(std::make_unique<StartTest>()); });
            on<Trigger<StartTest>, MainThread>().then([file_path] {
                // WHEN the file contents are modified.
                std::ofstream ofs1 = std::ofstream(file_path, std::ofstream::app);
                ofs1 << "Test\n";
                ofs1.close();
                UNSCOPED_INFO("File modified: " << file_path);
            });

            on<Watchdog<TestReactor, 1000, std::chrono::milliseconds>>().then([this] {
                UNSCOPED_INFO("Watchdog timeout.");
                watchdog_triggered = true;
                powerplant.shutdown();
            });
        }

        ~TestReactor() override {
            std::filesystem::remove_all(dir_path);  // Cleanup
        }

        // We have a non default destructor so rule of 5 is needed
        TestReactor(const TestReactor&)            = delete;
        TestReactor(TestReactor&&)                 = delete;
        TestReactor& operator=(const TestReactor&) = delete;
        TestReactor& operator=(TestReactor&&)      = delete;

    private:
        std::filesystem::path dir_path;
    };

    bool event_triggered(const std::string& file_name, extension::FileWatch::Event event) {
        auto iterator = std::find_if(file_watch_events.begin(),
                                     file_watch_events.end(),
                                     [file_name, event](extension::FileWatch const& file_watch) {
                                         std::filesystem::path file_path = file_watch.path;
                                         return file_path.filename() == file_name && file_watch.events == event;
                                     });

        return iterator != file_watch_events.end();
    }

}  // namespace

TEST_CASE("Given a file, when the file contents are modified, then change events are triggered for the file.") {

    INFO("\n === FileWatcherTestFile Start ===\n");

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant plant(config);
    plant.install<module::extension::FileWatcher>();
    plant.install<TestReactor>();

    plant.start();

    // THEN change events are triggered for the file.
    REQUIRE_FALSE(watchdog_triggered);
    REQUIRE(file_watch_events.size() == 2);
    REQUIRE(event_triggered(test_file_name, extension::FileWatch::NO_OP));    // Initial event
    REQUIRE(event_triggered(test_file_name, extension::FileWatch::CHANGED));  // Change event
}
