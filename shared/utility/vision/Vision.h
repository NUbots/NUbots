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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */
#ifndef UTILITY_VISION_VISION_H
#define UTILITY_VISION_VISION_H

namespace utility {
    namespace vision {

        enum class Colour : uint8_t {
            // Main classifications
            UNCLASSIFIED = 'u',
            WHITE        = 'w',
            GREEN        = 'g',
            ORANGE       = 'o',
            YELLOW       = 'y',
            CYAN         = 'c',
            MAGENTA      = 'm',

            // Ambiguous Classifications
            WHITE_GREEN  = 'f'
        };

        struct Pixel {
            Pixel() : rgba(0) {}
            Pixel(uint32_t rgba) : rgba(rgba) {}
            Pixel(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : 
                        components({ r, g, b, a}) {}
            Pixel(uint8_t r, uint8_t g, uint8_t b) :
                        components({ r, g, b, 0}) {}
            Pixel(const Pixel& pixel) : rgba(pixel.rgba) {}

            union {
                struct {
                    union {
                        uint8_t r;
                        uint8_t y;
                    };
                    union {
                        uint8_t g;
                        uint8_t u;
                        uint8_t cb;
                    };
                    union {
                        uint8_t b;
                        uint8_t v;
                        uint8_t cr;
                    };

                    uint8_t a;
                } components;

                uint32_t rgba;
            };
        };

    } // vision
} // utility

#endif // MESSAGE_VISION_VISION_H
