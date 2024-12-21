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

#ifndef MODULES_SUPPORT_LOGGING_CONSOLELOGHANDLER_HPP
#define MODULES_SUPPORT_LOGGING_CONSOLELOGHANDLER_HPP

#include <mutex>
#include <nuclear>
#include <string>
#include <vector>

#include "utility/support/evil/pure_evil.hpp"

namespace module::support::logging {

    /**
     * Handles the logging of log messages to the console in a thread safe manner.
     *
     * @author Trent Houliston
     */
    class ConsoleLogHandler : public NUClear::Reactor {
    public:
        explicit ConsoleLogHandler(std::unique_ptr<NUClear::Environment> environment);

    private:
        /**
         * Print a log to the console in a thread safe manner.
         *
         * @param level         the log level of the message
         * @param reactor       the name of the reactor that emitted the message
         * @param name          the name of the reaction that emitted the message
         * @param message       the message that was emitted
         * @param stack_trace   the stack trace of the message
         */
        void print_log(const NUClear::LogLevel& level,
                       const std::string& reactor,
                       const std::string& name,
                       const std::string& message,
                       const std::vector<utility::support::evil::StackFrame>& stack_trace);

        /// A mutex for accessing the std::cerr stream
        std::mutex mutex;
    };

}  // namespace module::support::logging

#endif  // MODULES_SUPPORT_LOGGING_CONSOLELOGHANDLER_HPP
