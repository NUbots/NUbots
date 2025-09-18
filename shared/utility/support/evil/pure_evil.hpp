/*
 * MIT License
 *
 * Copyright (c) 2016 NUbots
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


#ifndef UTILITY_SUPPORT_EVIL_PUREEVIL_HPP
#define UTILITY_SUPPORT_EVIL_PUREEVIL_HPP

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace utility::support::evil {

    struct StackFrame {

        StackFrame() = default;

        StackFrame(uintptr_t pc, std::string file, int lineno, std::string function)
            : pc(pc), file(std::move(file)), lineno(lineno), function(std::move(function)) {}

        uintptr_t pc{};
        std::string file;
        int lineno{};
        std::string function;
    };

    /**
     * Get the stack trace of the last exception that happened in the current thread.
     *
     * You must ensure that you are running this function in the thread that the exception was thrown in.
     *
     * Make sure you are compiling in debug mode to get the stack trace.
     * In release mode this function will return an empty vector.
     *
     * @return the stack trace of the current thread or an empty vector if not available
     */
    std::vector<StackFrame> last_exception_stack_trace();

    /**
     * Get the name of the last exception that happened in the current thread.
     *
     * You must ensure that you are running this function in the thread that the exception was thrown in.
     *
     * Make sure you are compiling in debug mode to get the exception name.
     * In release mode this function will return "Unknown exception".
     *
     * @return the name of the last exception or "Unknown exception" if not available
     */
    std::string last_exception_name();

}  // namespace utility::support::evil

#endif  // UTILITY_SUPPORT_EVIL_PUREEVIL_HPP
