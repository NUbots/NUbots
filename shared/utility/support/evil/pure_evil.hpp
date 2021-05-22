/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_SUPPORT_EVIL_PUREEVIL_HPP
#define UTILITY_SUPPORT_EVIL_PUREEVIL_HPP

#include <string>
#include <utility>
#include <vector>

// If we are not in debug mode, don't build it! Please don't build it!
#ifndef NDEBUG

namespace utility::support::evil {

    struct StackFrame {

        StackFrame() = default;

        StackFrame(uintptr_t pc_, std::string file_, int lineno_, std::string function_)
            : pc(pc_), file(std::move(std::move(file_))), lineno(lineno_), function(std::move(function_)) {}

        uintptr_t pc{};
        std::string file;
        int lineno{};
        std::string function;
    };
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    extern thread_local std::vector<StackFrame> stack;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    extern thread_local std::string exception_name;
}  // namespace utility::support::evil

#endif  // NDEBUG

#endif  // UTILITY_SUPPORT_EVIL_PUREEVIL_HPP
