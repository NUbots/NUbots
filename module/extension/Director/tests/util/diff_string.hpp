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
 * Copyright 2022 NUbots <nubots@nubots.net>
 */

#include <string>
#include <vector>

namespace util {

    /**
     * Using an LCS algorithm prints out the two sets of string (expected and actual) side by side to show the
     * differences
     *
     * @param expected  the expected series of events
     * @param actual    the actual series of events
     *
     * @return a multiline string showing a human output of the difference
     */
    std::string diff_string(const std::vector<std::string>& expected, const std::vector<std::string>& actual);

}  // namespace util
