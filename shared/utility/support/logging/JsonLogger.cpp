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
#include "JsonLogger.hpp"

namespace utility::support::logging {

    using NUClear::DEBUG;
    using NUClear::INFO;
    using NUClear::WARN;

    JsonLogger::JsonLogger(std::shared_ptr<NUClear::LogLevel> log_level) : log_level(std::move(log_level)) {}

    JsonLogger::~JsonLogger() {
        close();
    }

    bool JsonLogger::init(const std::string& file_path) {
        std::lock_guard<std::mutex> lock(mutex);

        // Close any existing file
        close();

        // Store the file path
        json_file_path = file_path;

        // Open the file for writing
        json_output.open(file_path);
        
        if (!json_output.is_open()) {
            NUClear::log<WARN>(*log_level, "Failed to open JSON output file: ", file_path);
            return false;
        }

        // Initialize the JSON file with an empty array
        json_output << "[]" << std::endl;
        json_output.seekp(-1, std::ios::end); // Move to just before the closing bracket
        
        enabled = true;
        NUClear::log<INFO>(*log_level, "Initialized JSON logging to: ", file_path);
        return true;
    }

    bool JsonLogger::log(const nlohmann::json& entry) {
        std::lock_guard<std::mutex> lock(mutex);
        
        if (!enabled || !json_output.is_open()) {
            return false;
        }

        try {
            // Write to file with a comma separator
            json_output << "," << entry.dump(2) << std::flush;
            return true;
        }
        catch (const std::exception& e) {
            NUClear::log<WARN>(*log_level, "Failed to write to JSON file: ", e.what());
            return false;
        }
    }

    void JsonLogger::close() {
        std::lock_guard<std::mutex> lock(mutex);
        
        if (enabled && json_output.is_open()) {
            // Close the JSON array
            json_output << "\n]" << std::endl;
            json_output.close();
            enabled = false;
            NUClear::log<INFO>(*log_level, "Closed JSON logging file: ", json_file_path);
        }
    }

    bool JsonLogger::is_enabled() const {
        return enabled && json_output.is_open();
    }

}  // namespace utility::support::logging