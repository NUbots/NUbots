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

#include <string>
#include <utility>
#include <vector>

// If we are not in debug mode, don't build it! Please don't build it!
#if !defined(NDEBUG) and !defined(__APPLE__)

namespace utility::support::evil {

    struct StackFrame {

        StackFrame() = default;

        StackFrame(uintptr_t pc_, std::string file_, int lineno_, std::string function_)
            : pc(pc_), file(std::move(file_)), lineno(lineno_), function(std::move(function_)) {}

        uintptr_t pc{};
        std::string file;
        int lineno{};
        std::string function;
    };

    extern thread_local std::vector<StackFrame> stack;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
    extern thread_local std::string exception_name;     // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace utility::support::evil

#endif  // !defined(NDEBUG) and !defined(__APPLE__)

#endif  // UTILITY_SUPPORT_EVIL_PUREEVIL_HPP
