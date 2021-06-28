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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_STRUTIL_ANSI_HPP
#define UTILITY_STRUTIL_ANSI_HPP

#include <algorithm>
#include <string>

namespace utility::strutil {

    enum class ANSICode : int {
        RESET     = 0,
        BRIGHT    = 1,
        FAINT     = 2,
        UNDERLINE = 4,
        BLINK     = 5,
        INVERSE   = 7,

        BLACK   = 30,
        RED     = 31,
        GREEN   = 32,
        YELLOW  = 33,
        BLUE    = 34,
        MAGENTA = 35,
        CYAN    = 36,
        GRAY    = 37,

        BLACK_BACKGROUND   = 40,
        RED_BACKGROUND     = 41,
        GREEN_BACKGROUND   = 42,
        YELLOW_BACKGROUND  = 43,
        BLUE_BACKGROUND    = 44,
        MAGENTA_BACKGROUND = 45,
        CYAN_BACKGROUND    = 46,
        GRAY_BACKGROUND    = 47
    };

    /**
     * TODO document
     *
     * See: http://en.wikipedia.org/wiki/ANSI_escape_code#Colors
     * See: http://stackoverflow.com/a/3219471/868679
     *
     *
     * @author Trent Houliston
     * @author Brendan Annable
     */
    template <enum ANSICode First, enum ANSICode... Codes>
    struct ANSISGR {

        struct Apply {
            Apply(std::ostream& out) : out(out) {}

            template <typename T>
            std::ostream& operator<<(const T& el) {

                // Print out the start of our escape code
                out << "\033[";

                // Print out our first code
                out << int(First);

                // Print out our remaining codes
                for (int code : std::initializer_list<int>({int(Codes)...})) {
                    out << ";" << code;
                }

                // Close our code
                out << "m";

                // Print our message and reset
                return out << el << "\033[0m";
            }

            std::ostream& out;
        };
    };

    template <enum ANSICode... Codes>
    typename ANSISGR<Codes...>::Apply operator<<(std::ostream& out, const ANSISGR<Codes...> /*codes*/) {
        return typename ANSISGR<Codes...>::Apply(out);
    }

    struct Colour {
        static constexpr ANSISGR<ANSICode::BLACK> black     = ANSISGR<ANSICode::BLACK>();
        static constexpr ANSISGR<ANSICode::RED> red         = ANSISGR<ANSICode::RED>();
        static constexpr ANSISGR<ANSICode::GREEN> green     = ANSISGR<ANSICode::GREEN>();
        static constexpr ANSISGR<ANSICode::YELLOW> yellow   = ANSISGR<ANSICode::YELLOW>();
        static constexpr ANSISGR<ANSICode::BLUE> blue       = ANSISGR<ANSICode::BLUE>();
        static constexpr ANSISGR<ANSICode::MAGENTA> magenta = ANSISGR<ANSICode::MAGENTA>();
        static constexpr ANSISGR<ANSICode::CYAN> cyan       = ANSISGR<ANSICode::CYAN>();
        static constexpr ANSISGR<ANSICode::GRAY> gray       = ANSISGR<ANSICode::GRAY>();
        static constexpr ANSISGR<ANSICode::BRIGHT, ANSICode::BLACK> brightblack =
            ANSISGR<ANSICode::BRIGHT, ANSICode::BLACK>();
        static constexpr ANSISGR<ANSICode::BRIGHT, ANSICode::RED> brightred =
            ANSISGR<ANSICode::BRIGHT, ANSICode::RED>();
        static constexpr ANSISGR<ANSICode::BRIGHT, ANSICode::GREEN> brightgreen =
            ANSISGR<ANSICode::BRIGHT, ANSICode::GREEN>();
        static constexpr ANSISGR<ANSICode::BRIGHT, ANSICode::YELLOW> brightyellow =
            ANSISGR<ANSICode::BRIGHT, ANSICode::YELLOW>();
        static constexpr ANSISGR<ANSICode::BRIGHT, ANSICode::BLUE> brightblue =
            ANSISGR<ANSICode::BRIGHT, ANSICode::BLUE>();
        static constexpr ANSISGR<ANSICode::BRIGHT, ANSICode::MAGENTA> brightmagenta =
            ANSISGR<ANSICode::BRIGHT, ANSICode::MAGENTA>();
        static constexpr ANSISGR<ANSICode::BRIGHT, ANSICode::CYAN> brightcyan =
            ANSISGR<ANSICode::BRIGHT, ANSICode::CYAN>();
        static constexpr ANSISGR<ANSICode::BRIGHT, ANSICode::GRAY> brightgray =
            ANSISGR<ANSICode::BRIGHT, ANSICode::GRAY>();
    };
}  // namespace utility::strutil
#endif
