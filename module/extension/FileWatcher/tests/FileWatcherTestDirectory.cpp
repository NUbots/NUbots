/*
 * Copyright (C) 2013-2016 Trent Houliston <trent@houliston.me>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

// Uncomment this line when other test files are added
#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <fmt/format.h>

#include <catch.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nuclear>

#include "FileWatcher.h"
#include "extension/FileWatch.h"

namespace {

struct StartTest {};

std::string testFile1Name = "directory-watcher-test-1.txt";
std::string testFile2Name = "directory-watcher-test-2.txt";
std::vector<extension::FileWatch> fileWatchEvents;
bool watchdogTriggered;

class TestReactor : public NUClear::Reactor {
public:
    TestReactor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        // GIVEN a directory containing two files.
        std::filesystem::path dirPath = std::filesystem::temp_directory_path() / "directory-watcher-test-XXXXXX";
        std::string dirPathString     = dirPath.string();

        std::vector<char> dirPathChars(dirPathString.c_str(), dirPathString.c_str() + dirPathString.size() + 1);
        dirPath = mkdtemp(dirPathChars.data());

        std::filesystem::path file1Path = dirPath / testFile1Name;
        std::filesystem::path file2Path = dirPath / testFile2Name;

        // Create files and close streams at end of scope
        {
            std::ofstream(file1Path.string());
            std::ofstream(file2Path.string());
        }

        on<extension::FileWatch>(dirPath.string(), extension::FileWatch::CHANGED)
            .then([this, file1Path, file2Path](const extension::FileWatch& message) {
                UNSCOPED_INFO("Directory change event: " << message.path);
                fileWatchEvents.push_back(message);

                if (fileWatchEvents.size() >= 4) {
                    powerplant.shutdown();
                }
            });

        on<Startup>().then([this] { emit(std::make_unique<StartTest>()); });
        on<Trigger<StartTest>, MainThread>().then([this, file1Path, file2Path] {
            // Delay for a little bit to ensure the system is entirely started
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // WHEN the file contents are modified.
            std::ofstream ofs1 = std::ofstream(file1Path, std::ofstream::app);
            ofs1 << "Test\n";
            ofs1.close();
            UNSCOPED_INFO("File modified: " << file1Path);

            std::ofstream ofs2 = std::ofstream(file2Path, std::ofstream::app);
            ofs2 << "Test\n";
            ofs2.close();
            UNSCOPED_INFO("File modified: " << file2Path);
        });

        on<Watchdog<TestReactor, 1000, std::chrono::milliseconds>>().then([this] {
            UNSCOPED_INFO("Watchdog timeout.");
            watchdogTriggered = true;
            powerplant.shutdown();
        });
    }

    virtual ~TestReactor() {
        std::filesystem::remove_all(dirPath);  // Cleanup
    }

private:
    std::filesystem::path dirPath;
};

bool eventTriggered(std::string fileName, extension::FileWatch::Event event) {
    auto iterator = std::find_if(
        fileWatchEvents.begin(), fileWatchEvents.end(), [fileName, event](extension::FileWatch const& fileWatch) {
            std::filesystem::path filePath = fileWatch.path;
            return filePath.filename() == fileName && fileWatch.events == event;
        });

    return iterator != fileWatchEvents.end();
}

}  // namespace


TEST_CASE(
    "Given a directory containing two files, when the file contents are modified, then change events are triggered for "
    "each file.") {

    INFO("Start Test");

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant plant(config);
    plant.install<module::extension::FileWatcher>();
    plant.install<TestReactor>();

    plant.start();

    // THEN change events are triggered for each file
    REQUIRE_FALSE(watchdogTriggered);
    REQUIRE(fileWatchEvents.size() == 4);
    REQUIRE(eventTriggered(testFile1Name, extension::FileWatch::NO_OP));    // Initial event
    REQUIRE(eventTriggered(testFile1Name, extension::FileWatch::CHANGED));  // Change event
    REQUIRE(eventTriggered(testFile2Name, extension::FileWatch::NO_OP));
    REQUIRE(eventTriggered(testFile2Name, extension::FileWatch::CHANGED));
}
