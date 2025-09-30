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
#ifndef UTILITY_SUPPORT_LOGGING_JSONLOGGER_HPP
#define UTILITY_SUPPORT_LOGGING_JSONLOGGER_HPP

#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <memory>
#include <nuclear>

namespace utility::support::logging {

    /**
     * @brief A utility class for managing JSON file logging with array-based format
     *
     * This class handles the lifecycle of a JSON log file that stores entries in an array format.
     * It ensures proper initialization and cleanup of the JSON file, and provides thread-safe logging capabilities.
     */
    class JsonLogger {
    public:
        /**
         * @brief Construct a new JsonLogger
         *
         * @param logger A reference to a NUClear logger to use for logging messages
         */
        explicit JsonLogger(std::shared_ptr<NUClear::LogLevel> log_level);

        /**
         * @brief Destructor ensures proper file cleanup
         */
        ~JsonLogger();

        /**
         * @brief Initialize logging to a specific file
         *
         * @param file_path The path where the JSON file should be created/opened
         * @return true if initialization was successful, false otherwise
         */
        bool init(const std::string& file_path);

        /**
         * @brief Log a JSON entry to the file
         *
         * @param entry The JSON object to log
         * @return true if logging was successful, false otherwise
         */
        bool log(const nlohmann::json& entry);

        /**
         * @brief Close the current log file
         */
        void close();

        /**
         * @brief Check if logging is currently enabled
         *
         * @return true if logging is enabled and file is open
         */
        bool is_enabled() const;

    private:
        /// @brief Output file stream for JSON logging
        std::ofstream json_output;

        /// @brief Path to the current JSON output file
        std::string json_file_path;

        /// @brief Whether JSON logging is currently enabled
        bool enabled = false;

        /// @brief Mutex for thread-safe file operations
        std::mutex mutex;

        /// @brief Reference to the NUClear logger's log level
        std::shared_ptr<NUClear::LogLevel> log_level;
    };

}  // namespace utility::support::logging

#endif  // UTILITY_SUPPORT_LOGGING_JSONLOGGER_HPP